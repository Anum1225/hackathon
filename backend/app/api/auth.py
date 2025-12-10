from fastapi import APIRouter, Depends, HTTPException, status
from sqlalchemy.ext.asyncio import AsyncSession
from sqlalchemy import select
from ..database import get_db
from ..models import UserProfile, User
from ..schemas import UserProfileCreate, UserProfileResponse, UserSignup, UserLogin, AuthResponse
from uuid import uuid4

router = APIRouter()

@router.post("/signup", response_model=AuthResponse)
async def signup(user: UserSignup, db: AsyncSession = Depends(get_db)):
    # Check if user exists
    result = await db.execute(select(User).where(User.email == user.email))
    existing_user = result.scalars().first()
    if existing_user:
        raise HTTPException(status_code=400, detail="Email already registered")
    
    new_user = User(
        id=str(uuid4()),
        name=user.name,
        email=user.email,
        password=user.password, # Plain text for Hackathon MVP
        emailVerified=False
    )
    db.add(new_user)
    await db.commit()
    await db.refresh(new_user)
    
    return AuthResponse(user_id=new_user.id, name=new_user.name, email=new_user.email)

@router.post("/login", response_model=AuthResponse)
async def login(user: UserLogin, db: AsyncSession = Depends(get_db)):
    result = await db.execute(select(User).where(User.email == user.email))
    existing_user = result.scalars().first()
    
    if not existing_user or existing_user.password != user.password:
        raise HTTPException(status_code=401, detail="Invalid credentials")
        
    return AuthResponse(user_id=existing_user.id, name=existing_user.name, email=existing_user.email)

@router.post("/profile/{user_id}", response_model=UserProfileResponse)
async def create_or_update_profile(
    user_id: str,
    profile: UserProfileCreate,
    db: AsyncSession = Depends(get_db)
):
    # Check if user exists (Optional, depending on if we sync better-auth user table)
    # result = await db.execute(select(User).where(User.id == user_id))
    # user = result.scalars().first()
    # if not user:
    #     raise HTTPException(status_code=404, detail="User not found")

    result = await db.execute(select(UserProfile).where(UserProfile.user_id == user_id))
    existing_profile = result.scalars().first()

    if existing_profile:
        existing_profile.software_background = profile.software_background
        existing_profile.hardware_background = profile.hardware_background
        existing_profile.preferred_language = profile.preferred_language
    else:
        new_profile = UserProfile(
            user_id=user_id,
            software_background=profile.software_background,
            hardware_background=profile.hardware_background,
            preferred_language=profile.preferred_language
        )
        db.add(new_profile)
    
    await db.commit()
    
    # Return the updated/new profile
    # Use refresh to ensure we have the latest
    if existing_profile:
        await db.refresh(existing_profile)
        return existing_profile
    else:
        await db.refresh(new_profile)
        return new_profile

@router.get("/profile/{user_id}", response_model=UserProfileResponse)
async def get_profile(user_id: str, db: AsyncSession = Depends(get_db)):
    result = await db.execute(select(UserProfile).where(UserProfile.user_id == user_id))
    profile = result.scalars().first()
    if not profile:
        raise HTTPException(status_code=404, detail="Profile not found")
    return profile
