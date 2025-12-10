from fastapi import APIRouter, Depends, HTTPException
from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy import select
from ..database import get_db
from ..schemas import PersonalizeRequest, PersonalizeResponse, TranslateRequest, TranslateResponse
from ..models import UserProfile
from ..config import get_settings
from openai import AsyncOpenAI

router = APIRouter()
settings = get_settings()
aclient = AsyncOpenAI(api_key=settings.OPENAI_API_KEY)

@router.post("/personalize", response_model=PersonalizeResponse)
async def personalize_content(request: PersonalizeRequest, db: AsyncSession = Depends(get_db)):
    try:
        result = await db.execute(select(UserProfile).where(UserProfile.user_id == request.user_id))
        profile = result.scalars().first()
        
        if not profile:
            background_info = "The user is a student learning Physical AI."
        else:
            background_info = f"""
            User Background:
            - Software Expertise: {profile.software_background}
            - Hardware Access: {profile.hardware_background}
            - Preferred Language: {profile.preferred_language}
            """

        system_prompt = f"""You are an expert technical editor.
        Rewrite the following textbook content to be more engaging and relevant to the specific user based on their background.
        
        {background_info}
        
        If they know Python, use Python analogies. If they are beginners, simplify concepts.
        Keep the core technical accuracy but adjust the tone and examples.
        """

        response = await aclient.chat.completions.create(
            model="gpt-4o",
            messages=[
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": request.content}
            ]
        )
        
        return PersonalizeResponse(personalized_content=response.choices[0].message.content)
    except Exception as e:
        print(f"Personalize error: {e}")
        return PersonalizeResponse(personalized_content=request.content)

@router.post("/translate", response_model=TranslateResponse)
async def translate_content(request: TranslateRequest):
    try:
        system_prompt = f"You are a professional translator. Translate the technical text into {request.target_lang}. Preserve technical terms where appropriate (e.g. ROS 2, Python) but explain if necessary."
        
        response = await aclient.chat.completions.create(
            model="gpt-4o",
            messages=[
                {"role": "system", "content": system_prompt},
                {"role": "user", "content": request.text}
            ]
        )

        return TranslateResponse(translated_text=response.choices[0].message.content)
    except Exception as e:
        print(f"Translate error: {e}")
        return TranslateResponse(translated_text=request.text)
