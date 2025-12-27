from typing import Dict, List, Optional
from datetime import datetime, timedelta
import uuid
import logging

logger = logging.getLogger(__name__)

class QuerySession:
    """Represents a user's query session."""

    def __init__(self, session_id: str):
        self.session_id = session_id
        self.created_at = datetime.utcnow()
        self.queries: List[Dict] = []
        self.last_activity = self.created_at

    def add_query(self, query_text: str, response: Dict):
        """Add a query to the session."""
        query_record = {
            'id': str(uuid.uuid4()),
            'query': query_text,
            'response': response,
            'timestamp': datetime.utcnow().isoformat()
        }
        self.queries.append(query_record)
        self.last_activity = datetime.utcnow()
        logger.info(f"Added query to session {self.session_id}")

    def get_recent_queries(self, count: int = 5) -> List[Dict]:
        """Get the most recent queries from the session."""
        return self.queries[-count:]

class SessionService:
    """Service for managing user query sessions."""

    def __init__(self, session_timeout_minutes: int = 30):
        self.sessions: Dict[str, QuerySession] = {}
        self.session_timeout = timedelta(minutes=session_timeout_minutes)

    def create_session(self) -> QuerySession:
        """Create a new query session."""
        session_id = str(uuid.uuid4())
        session = QuerySession(session_id)
        self.sessions[session_id] = session
        logger.info(f"Created new session: {session_id}")
        return session

    def get_session(self, session_id: str) -> Optional[QuerySession]:
        """Get a session by ID, cleaning up expired sessions."""
        self._cleanup_expired_sessions()

        if session_id in self.sessions:
            session = self.sessions[session_id]
            # Update last activity
            session.last_activity = datetime.utcnow()
            return session

        return None

    def add_query_to_session(self, session_id: str, query_text: str, response: Dict):
        """Add a query to a session."""
        session = self.get_session(session_id)
        if session:
            session.add_query(query_text, response)
        else:
            logger.warning(f"Attempted to add query to non-existent session: {session_id}")

    def _cleanup_expired_sessions(self):
        """Remove expired sessions."""
        now = datetime.utcnow()
        expired_sessions = []

        for session_id, session in self.sessions.items():
            if now - session.last_activity > self.session_timeout:
                expired_sessions.append(session_id)

        for session_id in expired_sessions:
            del self.sessions[session_id]
            logger.info(f"Cleaned up expired session: {session_id}")

    def get_recent_queries(self, session_id: str, count: int = 5) -> List[Dict]:
        """Get recent queries for a session."""
        session = self.get_session(session_id)
        if session:
            return session.get_recent_queries(count)
        return []

# Global session service instance
session_service = SessionService()