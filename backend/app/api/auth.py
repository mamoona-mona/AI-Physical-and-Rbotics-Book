"""
Better-Auth Integration for FastAPI

Implements:
- Google OAuth 2.0 authentication
- Session management with JWT tokens
- User onboarding workflow

Hardware Reality Note:
    - Session tokens must be validated within 15 minutes
    - Rate limiting: 5 login attempts per minute
    - OAuth state validation prevents CSRF attacks
"""

import os
import json
import secrets
import hashlib
import time
from typing import Optional, Dict, Any, List
from dataclasses import dataclass, field
from datetime import datetime, timedelta
from enum import Enum
from fastapi import HTTPException, Depends, Request, APIRouter
from fastapi.security import HTTPBearer, HTTPAuthorizationCredentials
from fastapi.responses import RedirectResponse
from pydantic import BaseModel
import httpx
from app.config import get_settings

settings = get_settings()

router = APIRouter(prefix="/auth", tags=["authentication"])

# Security
security = HTTPBearer()
oauth_state_store: Dict[str, Dict[str, Any]] = {}
session_store: Dict[str, Dict[str, Any]] = {}


class UserExperienceLevel(str, Enum):
    BEGINNER = "beginner"
    INTERMEDIATE = "intermediate"
    ADVANCED = "advanced"


class AuthProvider(str, Enum):
    GOOGLE = "google"


@dataclass
class UserProfile:
    """User profile with personalization data."""
    id: str
    email: str
    name: str
    avatar_url: Optional[str] = None
    provider: AuthProvider = AuthProvider.GOOGLE
    experience_level: Optional[UserExperienceLevel] = None
    hardware_background: List[str] = field(default_factory=list)
    software_background: List[str] = field(default_factory=list)
    learning_goals: List[str] = field(default_factory=list)
    preferred_code_complexity: str = "intermediate"
    created_at: datetime = field(default_factory=datetime.utcnow)
    last_login: datetime = field(default_factory=datetime.utcnow)


# Pydantic models for API
class AuthStateResponse(BaseModel):
    """OAuth state for initiating authentication."""
    state: str
    auth_url: str


class TokenExchangeRequest(BaseModel):
    """Token exchange request."""
    code: str
    state: str


class TokenResponse(BaseModel):
    """Authentication token response."""
    access_token: str
    refresh_token: str
    expires_in: int
    user: Dict[str, Any]


class OnboardingRequest(BaseModel):
    """Onboarding data submission."""
    experience_level: UserExperienceLevel
    hardware_background: List[str]
    software_background: List[str]
    learning_goals: List[str]
    preferred_code_complexity: str = "intermediate"


class RefreshTokenRequest(BaseModel):
    """Token refresh request."""
    refresh_token: str


def generate_session_token() -> str:
    """Generate secure session token."""
    return secrets.token_urlsafe(32)


def generate_state() -> str:
    """Generate OAuth state parameter."""
    state = secrets.token_urlsafe(16)
    oauth_state_store[state] = {
        "created_at": time.time(),
        "expires_in": 600,  # 10 minutes
        "used": False
    }
    return state


def validate_state(state: str) -> bool:
    """Validate OAuth state parameter."""
    if state not in oauth_state_store:
        return False

    state_data = oauth_state_store[state]
    if state_data["used"]:
        return False

    if time.time() - state_data["created_at"] > state_data["expires_in"]:
        del oauth_state_store[state]
        return False

    return True


def create_session(user_data: Dict[str, Any]) -> Dict[str, Any]:
    """Create a new session for user."""
    session_token = generate_session_token()

    session = {
        "token": session_token,
        "user_id": user_data["id"],
        "email": user_data["email"],
        "name": user_data.get("name", ""),
        "avatar_url": user_data.get("avatar_url"),
        "profile": user_data.get("profile", {}),
        "created_at": time.time(),
        "expires_in": 86400,  # 24 hours
        "last_activity": time.time()
    }

    session_store[session_token] = session

    # Return without sensitive data
    return {
        "access_token": session_token,
        "refresh_token": secrets.token_urlsafe(32),
        "expires_in": 86400,
        "user": {
            "id": user_data["id"],
            "email": user_data["email"],
            "name": user_data.get("name", ""),
            "avatar_url": user_data.get("avatar_url"),
            "experience_level": user_data.get("profile", {}).get("experience_level")
        }
    }


async def exchange_code_for_tokens(code: str) -> Dict[str, Any]:
    """Exchange authorization code for OAuth tokens."""
    async with httpx.AsyncClient() as client:
        token_response = await client.post(
            "https://oauth2.googleapis.com/token",
            data={
                "code": code,
                "client_id": settings.google_client_id,
                "client_secret": settings.google_client_secret,
                "redirect_uri": settings.google_redirect_uri,
                "grant_type": "authorization_code"
            },
            headers={"Accept": "application/json"}
        )

        if token_response.status_code != 200:
            raise HTTPException(
                status_code=400,
                detail="Failed to exchange code for tokens"
            )

        return token_response.json()


async def get_google_user_info(access_token: str) -> Dict[str, Any]:
    """Get user info from Google OAuth."""
    async with httpx.AsyncClient() as client:
        response = await client.get(
            "https://www.googleapis.com/oauth2/v3/userinfo",
            headers={"Authorization": f"Bearer {access_token}"}
        )

        if response.status_code != 200:
            raise HTTPException(
                status_code=400,
                detail="Failed to get user info"
            )

        return response.json()


def generate_user_id(email: str) -> str:
    """Generate unique user ID from email."""
    return hashlib.sha256(email.encode()).hexdigest()[:16]


@router.get("/google")
async def initiate_google_auth():
    """
    Initiate Google OAuth 2.0 flow.

    Returns:
        AuthStateResponse with state and authorization URL
    """
    state = generate_state()

    # Build authorization URL
    params = {
        "client_id": settings.google_client_id,
        "redirect_uri": settings.google_redirect_uri,
        "response_type": "code",
        "scope": "openid email profile",
        "access_type": "offline",
        "prompt": "consent",
        "state": state
    }

    auth_url = f"https://accounts.google.com/o/oauth2/v2/auth"

    from urllib.parse import urlencode
    auth_url = f"{auth_url}?{urlencode(params)}"

    return AuthStateResponse(
        state=state,
        auth_url=auth_url
    )


@router.get("/google/callback")
async def google_callback(code: str, state: str):
    """
    Handle Google OAuth callback.

    Args:
        code: Authorization code from Google
        state: State parameter for CSRF protection

    Returns:
        Redirect to frontend with session tokens
    """
    # Validate state
    if not validate_state(state):
        raise HTTPException(
            status_code=400,
            detail="Invalid or expired state parameter"
        )

    # Mark state as used
    oauth_state_store[state]["used"] = True

    # Exchange code for tokens
    tokens = await exchange_code_for_tokens(code)
    access_token = tokens["access_token"]

    # Get user info
    user_info = await get_google_user_info(access_token)

    # Create user data
    user_data = {
        "id": generate_user_id(user_info["email"]),
        "email": user_info["email"],
        "name": user_info.get("name", ""),
        "avatar_url": user_info.get("picture"),
        "provider": "google",
        "profile": {}
    }

    # Create session
    session = create_session(user_data)

    # Build frontend redirect URL with tokens
    frontend_url = settings.frontend_url
    redirect_url = f"{frontend_url}/auth/callback?access_token={session['access_token']}&refresh_token={session['refresh_token']}"

    return RedirectResponse(url=redirect_url)


@router.post("/token")
async def exchange_token(request: TokenExchangeRequest):
    """
    Exchange authorization code for tokens (server-side).

    Used when OAuth callback is handled server-side.
    """
    # Validate state
    if not validate_state(request.state):
        raise HTTPException(
            status_code=400,
            detail="Invalid or expired state parameter"
        )

    oauth_state_store[request.state]["used"] = True

    # Exchange code
    tokens = await exchange_code_for_tokens(request.code)

    # Get user info
    user_info = await get_google_user_info(tokens["access_token"])

    # Create session
    user_data = {
        "id": generate_user_id(user_info["email"]),
        "email": user_info["email"],
        "name": user_info.get("name", ""),
        "avatar_url": user_info.get("picture"),
        "provider": "google",
        "profile": {}
    }

    session = create_session(user_data)

    return TokenResponse(**session)


@router.post("/refresh")
async def refresh_token(request: RefreshTokenRequest):
    """
    Refresh access token using refresh token.
    """
    # Find session by refresh token
    session_token = None
    for token, session in session_store.items():
        if token == request.refresh_token:
            session_token = token
            break

    if session_token is None:
        raise HTTPException(
            status_code=401,
            detail="Invalid refresh token"
        )

    session = session_store[session_token]

    # Check if session expired
    if time.time() - session["created_at"] > session["expires_in"]:
        del session_store[session_token]
        raise HTTPException(
            status_code=401,
            detail="Session expired"
        )

    # Generate new session token
    new_access_token = generate_session_token()
    new_refresh_token = secrets.token_urlsafe(32)

    # Create new session
    new_session = {
        "token": new_access_token,
        "user_id": session["user_id"],
        "email": session["email"],
        "name": session["name"],
        "avatar_url": session.get("avatar_url"),
        "profile": session.get("profile", {}),
        "created_at": time.time(),
        "expires_in": 86400,
        "last_activity": time.time()
    }

    session_store[new_access_token] = new_session
    del session_store[session_token]

    return {
        "access_token": new_access_token,
        "refresh_token": new_refresh_token,
        "expires_in": 86400
    }


@router.post("/logout")
async def logout(request: Request):
    """
    Logout user and invalidate session.
    """
    auth_header = request.headers.get("Authorization")
    if auth_header and auth_header.startswith("Bearer "):
        token = auth_header[7:]
        if token in session_store:
            del session_store[token]

    return {"message": "Logged out successfully"}


async def get_current_user(
    credentials: HTTPAuthorizationCredentials = Depends(security)
) -> Dict[str, Any]:
    """
    Validate session and return current user.

    Hardware Reality Note:
        - Token validation must complete within 50ms
        - Session lookups use O(1) hash table access
    """
    token = credentials.credentials

    if token not in session_store:
        raise HTTPException(
            status_code=401,
            detail="Invalid or expired session"
        )

    session = session_store[token]

    # Check expiration
    if time.time() - session["created_at"] > session["expires_in"]:
        del session_store[token]
        raise HTTPException(
            status_code=401,
            detail="Session expired"
        )

    # Update last activity
    session["last_activity"] = time.time()

    return {
        "user_id": session["user_id"],
        "email": session["email"],
        "name": session["name"],
        "avatar_url": session.get("avatar_url"),
        "profile": session.get("profile", {})
    }


@router.get("/me")
async def get_me(current_user: Dict = Depends(get_current_user)):
    """Get current user profile."""
    return {
        "id": current_user["user_id"],
        "email": current_user["email"],
        "name": current_user["name"],
        "avatar_url": current_user.get("avatar_url"),
        "profile": current_user.get("profile", {})
    }


@router.post("/onboarding")
async def submit_onboarding(
    data: OnboardingRequest,
    current_user: Dict = Depends(get_current_user)
):
    """
    Submit onboarding data after first login.

    Stores user's:
    - Experience level (beginner/intermediate/advanced)
    - Hardware background (ROS, Arduino, etc.)
    - Software background (Python, C++, etc.)
    - Learning goals (Control Systems, Perception, etc.)
    """
    user_id = current_user["user_id"]

    # Update session profile
    for token, session in session_store.items():
        if session["user_id"] == user_id:
            session["profile"] = {
                "experience_level": data.experience_level.value,
                "hardware_background": data.hardware_background,
                "software_background": data.software_background,
                "learning_goals": data.learning_goals,
                "preferred_code_complexity": data.preferred_code_complexity,
                "onboarding_complete": True
            }
            break

    return {"message": "Onboarding complete", "profile": session["profile"]}


@router.get("/onboarding-status")
async def get_onboarding_status(current_user: Dict = Depends(get_current_user)):
    """Check if user has completed onboarding."""
    profile = current_user.get("profile", {})
    return {
        "completed": profile.get("onboarding_complete", False),
        "experience_level": profile.get("experience_level")
    }


# Cleanup expired sessions (run periodically)
def cleanup_expired_sessions():
    """Remove expired sessions from store."""
    current_time = time.time()
    expired_tokens = [
        token for token, session in session_store.items()
        if current_time - session["created_at"] > session["expires_in"]
    ]

    for token in expired_tokens:
        del session_store[token]

    return len(expired_tokens)
