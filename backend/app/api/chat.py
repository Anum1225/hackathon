from fastapi import APIRouter, Depends, HTTPException
from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy import select
from ..database import get_db
from ..vector_db import get_qdrant_client
from ..schemas import ChatRequest, ChatResponse
from ..models import UserProfile
from ..config import get_settings
from openai import AsyncOpenAI

router = APIRouter()
settings = get_settings()
aclient = AsyncOpenAI(api_key=settings.OPENAI_API_KEY)

@router.post("/chat", response_model=ChatResponse)
async def chat_endpoint(request: ChatRequest, db: AsyncSession = Depends(get_db)):
    # 1. Get Personalization Context
    personalization_instruction = ""
    if request.user_id:
        result = await db.execute(select(UserProfile).where(UserProfile.user_id == request.user_id))
        profile = result.scalars().first()
        if profile:
            personalization_instruction = f"""
            The user has the following background:
            - Software: {profile.software_background}
            - Hardware: {profile.hardware_background}
            - Preferred Language: {profile.preferred_language}
            Tailor the answer to this level of expertise.
            """

    # 2. Context Retrieval (RAG)
    # If selected_text is present, prioritize it
    context_text = ""
    sources = []
    
    qdrant = get_qdrant_client()
    
    # Generate embedding for search
    try:
        if request.selected_text:
            context_text += f"User selected specifically this text to ask about: '{request.selected_text}'\n\n"
        
        # Search for relevant documents based on the user's message
        search_query = request.message
        if request.selected_text:
            search_query = f"{request.selected_text} {request.message}"

        embedding_resp = await aclient.embeddings.create(
            input=search_query,
            model="text-embedding-3-small"
        )
        query_vector = embedding_resp.data[0].embedding

        search_results = qdrant.search(
            collection_name="textbook",
            query_vector=query_vector,
            limit=3
        )
        
        for hit in search_results:
            context_text += f"---\nSource: {hit.payload.get('source', 'Unknown')}\n{hit.payload.get('text', '')}\n"
            sources.append(hit.payload.get('source', 'Unknown'))
            
    except Exception as e:
        print(f"Vector search failed: {e}")
        # Continue without context or with minimal context
        context_text += "\n[Note: Unable to retrieve textbook context at this moment. Answering based on general knowledge.]\n"

    # 3. Generate Response
    system_prompt = f"""You are an AI teaching assistant for a Physical AI & Humanoid Robotics course.
    Use the provided context to answer the student's question.
    If the context doesn't contain the answer, use your general knowledge but mention that it's not in the text.
    
    {personalization_instruction}
    
    Context:
    {context_text}
    """

    messages = [{"role": "system", "content": system_prompt}]
    # Add history
    for msg in request.history:
        # OpenAI requires 'assistant', not 'bot'
        role = "assistant" if msg.get("role") == "bot" else msg.get("role")
        messages.append({"role": role, "content": msg.get("content")})
    
    messages.append({"role": "user", "content": request.message})

    try:
        response = await aclient.chat.completions.create(
            model="gpt-4o",
            messages=messages,
            temperature=0.7
        )
        answer = response.choices[0].message.content
    except Exception as e:
        print(f"OpenAI Chat Completion failed: {e}")
        answer = "I'm sorry, I'm having trouble generating a response right now. Please check the backend logs for details."


    
    return ChatResponse(response=answer, sources=sources)
