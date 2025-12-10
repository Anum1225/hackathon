from sqlalchemy.ext.asyncio import create_async_engine, AsyncSession
from sqlalchemy.orm import sessionmaker, declarative_base
from .config import get_settings

settings = get_settings()

from sqlalchemy.engine.url import make_url

settings = get_settings()

# Handle potential issues with SSL and asyncpg on Windows
url = make_url(settings.NEON_DATABASE_URL)
connect_args = {}

# If sslmode is present, move it to connect_args or handle it
if 'sslmode' in url.query:
    ssl_mode = url.query['sslmode']
    # Removing sslmode from query as we will handle it via connect_args if needed
    # For asyncpg, 'verify-full' or 'require' usually maps to ssl='require' or manual SSLContext
    # But usually simple 'ssl'='require' works for Neon. 
    # We strip it from the URL to prevent SQLAlchemy from passing it blindly.
    # We also strip channel_binding if present.
    
    # Create a new query dict without problematic keys
    new_query = {k: v for k, v in url.query.items() if k not in ['sslmode', 'channel_binding']}
    url = url._replace(query=new_query)
    
    import ssl
    # Create explicit SSL context to avoid Windows asyncpg hangs
    ctx = ssl.create_default_context()
    ctx.check_hostname = False
    ctx.verify_mode = ssl.CERT_NONE
    connect_args["ssl"] = ctx

engine = create_async_engine(url, echo=True, connect_args=connect_args)

AsyncSessionLocal = sessionmaker(
    engine, class_=AsyncSession, expire_on_commit=False
)

Base = declarative_base()

async def get_db():
    async with AsyncSessionLocal() as session:
        yield session
