import os
import glob
from uuid import uuid4
from qdrant_client import QdrantClient
from qdrant_client.http import models
from openai import OpenAI
from dotenv import load_dotenv

# Load env variables from backend/.env
# We assume we run this from project root or backend root
# Let's try to load from backend/.env explicitly
load_dotenv(dotenv_path="backend/.env")

QDRANT_URL = os.getenv("QDRANT_URL")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
OPENAI_API_KEY = os.getenv("OPENAI_API_KEY")

if not QDRANT_URL or not OPENAI_API_KEY:
    print("Error: Missing Environment Variables. Please ensure backend/.env has keys.")
    exit(1)

client = OpenAI(api_key=OPENAI_API_KEY)
qdrant = QdrantClient(url=QDRANT_URL, api_key=QDRANT_API_KEY)

COLLECTION_NAME = "textbook"

def setup_collection():
    collections = qdrant.get_collections()
    exists = any(c.name == COLLECTION_NAME for c in collections.collections)
    
    if not exists:
        print(f"Creating collection {COLLECTION_NAME}...")
        qdrant.create_collection(
            collection_name=COLLECTION_NAME,
            vectors_config=models.VectorParams(size=1536, distance=models.Distance.COSINE),
        )
    else:
        print(f"Collection {COLLECTION_NAME} already exists.")

def get_embedding(text):
    response = client.embeddings.create(
        input=text,
        model="text-embedding-3-small"
    )
    return response.data[0].embedding

def ingest_docs():
    # Assume script is run from project root, so docs are in docs/
    doc_files = glob.glob("docs/**/*.md", recursive=True) + glob.glob("docs/**/*.mdx", recursive=True)
    
    print(f"Found {len(doc_files)} files to ingest.")
    
    points = []
    
    for file_path in doc_files:
        with open(file_path, "r", encoding="utf-8") as f:
            content = f.read()
            
        # Simple splitting by header or chunk size
        # For hackathon, just split by paragraphs or roughly 1000 chars
        chunks = [content[i:i+2000] for i in range(0, len(content), 2000)]
        
        for i, chunk in enumerate(chunks):
            if len(chunk.strip()) < 50:
                continue
                
            print(f"Processing {file_path} chunk {i}...")
            vector = get_embedding(chunk)
            
            payload = {
                "source": file_path,
                "text": chunk,
                "chunk_index": i
            }
            
            points.append(models.PointStruct(
                id=str(uuid4()),
                vector=vector,
                payload=payload
            ))
            
            if len(points) >= 20:
                qdrant.upsert(
                    collection_name=COLLECTION_NAME,
                    points=points
                )
                points = []
                print("Upserted batch.")

    if points:
        qdrant.upsert(
            collection_name=COLLECTION_NAME,
            points=points
        )
        print("Upserted final batch.")

if __name__ == "__main__":
    setup_collection()
    ingest_docs()
    print("Ingestion complete!")
