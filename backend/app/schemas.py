from pydantic import BaseModel
from typing import Optional

class UserProfileBase(BaseModel):
    software_background: Optional[str] = None
    hardware_background: Optional[str] = None
    preferred_language: Optional[str] = "en"

class UserProfileCreate(UserProfileBase):
    pass

class UserProfileResponse(UserProfileBase):
    user_id: str

    class Config:
        from_attributes = True

class ChatRequest(BaseModel):
    message: str
    selected_text: Optional[str] = None
    history: list[dict] = []
    user_id: Optional[str] = None
    translate_to_urdu: bool = False
    personalize: bool = False

class ChatResponse(BaseModel):
    response: str
    sources: list[str] = []

class PersonalizeRequest(BaseModel):
    content: str
    user_id: str

class PersonalizeResponse(BaseModel):
    personalized_content: str

class TranslateRequest(BaseModel):
    text: str
    target_lang: str = "ur"

class TranslateResponse(BaseModel):
    translated_text: str

class UserSignup(BaseModel):
    name: str
    email: str
    password: str

class UserLogin(BaseModel):
    email: str
    password: str

class AuthResponse(BaseModel):
    user_id: str
    name: str
    email: str
