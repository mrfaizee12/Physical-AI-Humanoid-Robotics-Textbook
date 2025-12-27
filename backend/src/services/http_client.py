import httpx
import asyncio
import logging
from typing import Dict, Any, Optional
from pydantic import BaseModel

logger = logging.getLogger(__name__)

class HTTPClient:
    """HTTP client for making requests to external services."""

    def __init__(self, base_url: str, timeout: int = 30):
        """
        Initialize the HTTP client.

        Args:
            base_url: Base URL for the API
            timeout: Request timeout in seconds
        """
        self.base_url = base_url.rstrip('/')
        self.timeout = timeout
        self.client = httpx.AsyncClient(timeout=httpx.Timeout(timeout))

    async def get(self, endpoint: str, headers: Optional[Dict[str, str]] = None, params: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        """
        Make a GET request to the specified endpoint.

        Args:
            endpoint: API endpoint to call
            headers: Optional request headers
            params: Optional query parameters

        Returns:
            Response data as dictionary
        """
        url = f"{self.base_url}{endpoint}"
        try:
            response = await self.client.get(url, headers=headers, params=params)
            response.raise_for_status()
            return response.json()
        except httpx.HTTPStatusError as e:
            logger.error(f"HTTP error {e.response.status_code} for GET {url}: {e}")
            raise
        except httpx.RequestError as e:
            logger.error(f"Request error for GET {url}: {e}")
            raise

    async def post(self, endpoint: str, headers: Optional[Dict[str, str]] = None, json_data: Optional[Dict[str, Any]] = None) -> Dict[str, Any]:
        """
        Make a POST request to the specified endpoint.

        Args:
            endpoint: API endpoint to call
            headers: Optional request headers
            json_data: JSON payload for the request

        Returns:
            Response data as dictionary
        """
        url = f"{self.base_url}{endpoint}"
        try:
            response = await self.client.post(url, headers=headers, json=json_data)
            response.raise_for_status()
            return response.json()
        except httpx.HTTPStatusError as e:
            logger.error(f"HTTP error {e.response.status_code} for POST {url}: {e}")
            raise
        except httpx.RequestError as e:
            logger.error(f"Request error for POST {url}: {e}")
            raise

    async def close(self):
        """Close the HTTP client."""
        await self.client.aclose()

    async def __aenter__(self):
        """Async context manager entry."""
        return self

    async def __aexit__(self, exc_type, exc_val, exc_tb):
        """Async context manager exit."""
        await self.close()