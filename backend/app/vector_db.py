from qdrant_client import QdrantClient
from .config import get_settings

settings = get_settings()

def get_qdrant_client():
    return QdrantClient(
        url=settings.QDRANT_URL,
        api_key=settings.QDRANT_API_KEY
    )
