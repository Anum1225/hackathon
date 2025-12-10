from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from .config import get_settings
from .api import auth, chat, content

app = FastAPI(title="Physical AI Textbook Backend")

app.add_middleware(
    CORSMiddleware,
    allow_origins=["http://localhost:3000", "http://127.0.0.1:3000"], 
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

from .database import engine, Base
from .models import User, UserProfile # Ensure models are registered

@app.on_event("startup")
async def init_tables():
    async with engine.begin() as conn:
        await conn.run_sync(Base.metadata.create_all)

app.include_router(auth.router, prefix="/api/auth", tags=["Auth"])
app.include_router(chat.router, prefix="/api", tags=["Chat"])
app.include_router(content.router, prefix="/api", tags=["Content"])

@app.get("/")
def read_root():
    return {"message": "Physical AI Textbook API is running"}

@app.get("/health")
def health_check():
    return {"status": "healthy"}
