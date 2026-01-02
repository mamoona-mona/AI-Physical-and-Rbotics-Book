"""Main FastAPI application."""
from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from app.config import get_settings
from app.api.routes import router as api_router
from app.api.auth import router as auth_router

settings = get_settings()

app = FastAPI(
    title="Physical AI & Humanoid Robotics API",
    description="RAG chatbot API for the interactive textbook",
    version="1.0.0",
)

# CORS configuration
app.add_middleware(
    CORSMiddleware,
    allow_origins=[settings.frontend_url, "http://localhost:3000", "http://localhost:5173"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# Include API routes
app.include_router(api_router, prefix="/api/v1")
app.include_router(auth_router, prefix="/api/v1")


@app.on_event("startup")
async def startup():
    """Startup event handler."""
    # Try to initialize database tables (optional)
    try:
        from app.db.connection import Base, async_engine
        async with async_engine.begin() as conn:
            await conn.run_sync(Base.metadata.create_all)
    except Exception as e:
        print(f"[INFO] Database not available: {e}")
        print("[INFO] Running in demo mode without database")


@app.get("/")
async def root():
    """Root endpoint."""
    return {
        "name": "Physical AI & Humanoid Robotics API",
        "version": "1.0.0",
        "docs": "/docs",
    }
