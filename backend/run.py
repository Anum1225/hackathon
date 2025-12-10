import uvicorn
import os
from dotenv import load_dotenv

if __name__ == "__main__":
    # Ensure env is loaded
    load_dotenv("app/.env") # Try local or let config handle it
    
    print("Starting Physical AI Textbook Backend...")
    # Run the app
    # reload=True for development convenience
    uvicorn.run("app.main:app", host="0.0.0.0", port=8000, reload=True)
