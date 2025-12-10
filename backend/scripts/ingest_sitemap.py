import os
import requests
from bs4 import BeautifulSoup
from uuid import uuid4
from qdrant_client import QdrantClient
from qdrant_client.http import models
from openai import OpenAI
from dotenv import load_dotenv

# Load env variables
load_dotenv(os.path.join(os.path.dirname(__file__), "../.env"))

QDRANT_URL = os.getenv("QDRANT_URL")
QDRANT_API_KEY = os.getenv("QDRANT_API_KEY")
OPENAI_API_KEY = os.getenv("OPENAI_API_KEY")

if not QDRANT_URL or not OPENAI_API_KEY:
    print("Error: Missing Environment Variables.")
    exit(1)

client = OpenAI(api_key=OPENAI_API_KEY)
qdrant = QdrantClient(url=QDRANT_URL, api_key=QDRANT_API_KEY)

COLLECTION_NAME = "textbook"

# Map URLs to local files
URL_TO_FILE = {
    "https://panaversity.org/auth/login": None, 
    "https://panaversity.org/markdown-page": None,
    "https://panaversity.org/docs/hardware-lab": "docs/hardware-lab.md", 
    "https://panaversity.org/docs/intro": "docs/intro.md",
    "https://panaversity.org/docs/week-01-02-intro": "docs/week-01-02-intro.md",
    "https://panaversity.org/docs/week-03-05-ros2": "docs/week-03-05-ros2.md",
    "https://panaversity.org/docs/week-06-07-gazebo": "docs/week-06-07-gazebo.md",
    "https://panaversity.org/docs/week-08-10-isaac": "docs/week-08-10-isaac.md",
    "https://panaversity.org/docs/week-11-12-humanoid": "docs/week-11-12-humanoid.md",
    "https://panaversity.org/docs/week-13-conversational": "docs/week-13-conversational.md",
    "https://panaversity.org/": "docs/intro.md"
}

def get_embedding(text):
    response = client.embeddings.create(
        input=text,
        model="text-embedding-3-small"
    )
    return response.data[0].embedding

def fetch_and_parse(url):
    try:
        print(f"Fetching {url}...")
        resp = requests.get(url, timeout=10)
        if resp.status_code != 200:
            print(f"Failed to fetch {url}: {resp.status_code}")
            return ""
        
        soup = BeautifulSoup(resp.text, 'lxml')
        for script in soup(["script", "style", "nav", "footer"]):
            script.decompose()

        text = soup.get_text()
        lines = (line.strip() for line in text.splitlines())
        chunks = (phrase.strip() for line in lines for phrase in line.split("  "))
        text = '\n'.join(chunk for chunk in chunks if chunk)
        return text
    except Exception as e:
        print(f"Error fetching {url}: {e}")
        return ""

def ingest_urls():
    points = []
    
    for url, filepath in URL_TO_FILE.items():
        content = ""
        # Try local file first
        if filepath and os.path.exists(filepath):
            print(f"Ingesting {filepath} as {url}...")
            with open(filepath, 'r', encoding='utf-8') as f:
                content = f.read()
        else:
            # Fallback to fetch
            # Skip if explicitly None
            if filepath is None:
                print(f"Skipping {url} (Excluded)")
                continue

            content = fetch_and_parse(url)
            if not content:
                print(f"Skipping {url} (Values to fetch/read failed)")
                continue

        # Chunking: 1000 chars with overlap in case of plain text
        # If markdown, could use better splitting but text splitting is fine for MVP
        chunk_size = 1000
        overlap = 100
        
        text_len = len(content)
        start = 0
        
        chunk_idx = 0
        while start < text_len:
            end = start + chunk_size
            chunk = content[start:end]
            
            if len(chunk) < 50:
                break
                
            vector = get_embedding(chunk)
            
            payload = {
                "source": url,
                "text": chunk,
                "chunk_index": chunk_idx
            }
            
            points.append(models.PointStruct(
                id=str(uuid4()),
                vector=vector,
                payload=payload
            ))
            
            start += (chunk_size - overlap)
            chunk_idx += 1
            
            if len(points) >= 20:
                qdrant.upsert(
                    collection_name=COLLECTION_NAME,
                    points=points
                )
                points = []
                print(f"Upserted batch for {url}")

    if points:
        qdrant.upsert(
            collection_name=COLLECTION_NAME,
            points=points
        )
        print("Upserted final batch.")

def setup_collection():
    try:
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
    except Exception as e:
        print(f"Error checking/creating collection: {e}")

if __name__ == "__main__":
    setup_collection()
    ingest_urls()
    print("Sitemap Ingestion complete!")
