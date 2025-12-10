from pydantic_settings import BaseSettings, SettingsConfigDict
from functools import lru_cache
import os

class Settings(BaseSettings):
    NEON_DATABASE_URL: str
    QDRANT_URL: str
    QDRANT_API_KEY: str
    OPENAI_API_KEY: str
    
    # Calculate absolute path to .env file (one directory up from this file)
    model_config = SettingsConfigDict(
        env_file=os.path.join(os.path.dirname(__file__), "../.env"),
        env_file_encoding='utf-8',
        extra='ignore'
    )

@lru_cache
def get_settings():
    return Settings()
