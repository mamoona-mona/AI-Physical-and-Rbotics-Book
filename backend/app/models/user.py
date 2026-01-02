"""SQLAlchemy models for application."""
from datetime import datetime
from sqlalchemy import Column, Integer, String, DateTime, Boolean, Text, JSON
from app.db.connection import Base


class User(Base):
    """User model for authentication and tracking."""

    __tablename__ = "users"

    id = Column(Integer, primary_key=True, index=True)
    email = Column(String(255), unique=True, index=True, nullable=False)
    name = Column(String(255), nullable=True)
    image = Column(String(512), nullable=True)
    created_at = Column(DateTime, default=datetime.utcnow)
    updated_at = Column(DateTime, default=datetime.utcnow, onupdate=datetime.utcnow)
    is_active = Column(Boolean, default=True)

    # Better-Auth fields
    better_auth_id = Column(String(255), unique=True, index=True, nullable=True)

    # Onboarding Questionnaire Data
    experience_level = Column(String(20), nullable=True)  # beginner, intermediate, advanced

    # Hardware Background - specific options
    hardware_background = Column(String(50), nullable=True)  # "NVIDIA GPU", "Jetson", "None"

    # Software Background - specific options
    software_background = Column(String(50), nullable=True)  # "ROS2", "Python", "None"

    # Topic Interests (array stored as JSON)
    topic_interests = Column(JSON, nullable=True)  # ["Control Systems", "Computer Vision", etc.]

    # Learning Goals
    learning_goals = Column(Text, nullable=True)  # Free text: "I want to build humanoid robots"

    # Preferred Code Complexity
    code_complexity = Column(String(20), nullable=True)  # beginner, intermediate, advanced

    # Show/Hide UI hints based on experience
    show_advanced_code = Column(Boolean, default=False)
    show_math_details = Column(Boolean, default=True)
    show_practical_examples = Column(Boolean, default=True)

    # Session tracking
    last_active = Column(DateTime, nullable=True)
    chat_sessions_count = Column(Integer, default=0)

    def __repr__(self):
        return f"<User(id={self.id}, email='{self.email}', name='{self.name}')>"

    def get_personalization_context(self) -> dict:
        """Get personalization context for RAG responses."""
        return {
            "experience_level": self.experience_level,
            "hardware_background": self.hardware_background or "None",
            "software_background": self.software_background or "None",
            "interests": self.topic_interests or [],
            "learning_goals": self.learning_goals,
            "code_complexity": self.code_complexity or "intermediate",
        }

    def should_show_advanced_content(self) -> bool:
        """Check if user should see advanced content."""
        return self.experience_level in ["intermediate", "advanced"] or self.show_advanced_code

    def get_ui_preferences(self) -> dict:
        """Get UI preferences based on user profile."""
        return {
            "show_advanced_code": self.should_show_advanced_content(),
            "show_math_details": self.show_math if hasattr(self, 'show_math') else self.show_math_details,
            "show_practical_examples": self.show_practical_examples,
            "code_language_preference": self._get_code_language(),
        }

    def _get_code_language(self) -> str:
        """Get preferred code language based on background."""
        if self.software_background == "C++":
            return "cpp"
        return "python"
