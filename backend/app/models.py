from sqlalchemy import Column, Integer, String, Boolean, ForeignKey, DateTime, Text
from sqlalchemy.orm import relationship
from sqlalchemy.sql import func
from .database import Base

class User(Base):
    # This table mimics/links to what better-auth uses. 
    # For now, we assume we can read from it or link to it.
    __tablename__ = "user"

    id = Column(String, primary_key=True)  # Better-auth uses string IDs
    name = Column(String)
    email = Column(String, unique=True, index=True)
    emailVerified = Column(Boolean, default=False)
    image = Column(String, nullable=True)
    password = Column(String, nullable=True) # Added for simple auth
    createdAt = Column(DateTime(timezone=True), server_default=func.now())
    updatedAt = Column(DateTime(timezone=True), onupdate=func.now())

    profile = relationship("UserProfile", back_populates="user", uselist=False)

class UserProfile(Base):
    __tablename__ = "user_profile"

    id = Column(Integer, primary_key=True, index=True)
    user_id = Column(String, ForeignKey("user.id"), unique=True)
    software_background = Column(Text, nullable=True) # e.g. "Experienced in Python, new to ROS"
    hardware_background = Column(Text, nullable=True) # e.g. "Has Jetson Nano"
    preferred_language = Column(String, default="en") # en or ur

    user = relationship("User", back_populates="profile")
