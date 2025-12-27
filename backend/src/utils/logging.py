import logging
import sys
from datetime import datetime
from typing import Any, Dict


def setup_logging():
    """
    Set up logging configuration for the application.
    """
    # Create a custom formatter
    formatter = logging.Formatter(
        '%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )

    # Create a handler that writes to stdout
    handler = logging.StreamHandler(sys.stdout)
    handler.setFormatter(formatter)

    # Configure the root logger
    root_logger = logging.getLogger()
    root_logger.setLevel(logging.INFO)
    root_logger.addHandler(handler)

    # Set specific loggers to WARNING to reduce noise
    logging.getLogger("uvicorn").setLevel(logging.WARNING)
    logging.getLogger("qdrant_client").setLevel(logging.WARNING)
    logging.getLogger("httpx").setLevel(logging.WARNING)


def get_logger(name: str) -> logging.Logger:
    """
    Get a logger instance with the specified name.
    """
    return logging.getLogger(name)


def log_api_call(user_id: str, endpoint: str, query: str, response_length: int = 0):
    """
    Log an API call with relevant information.
    """
    logger = get_logger("api")
    logger.info(f"API Call - User: {user_id}, Endpoint: {endpoint}, Query Length: {len(query)}, Response Length: {response_length}")


def log_error(error: Exception, context: str = ""):
    """
    Log an error with context information.
    """
    logger = get_logger("error")
    logger.error(f"Error in {context}: {str(error)}", exc_info=True)


def log_performance(operation: str, duration: float, details: Dict[str, Any] = None):
    """
    Log performance metrics for an operation.
    """
    logger = get_logger("performance")
    details_str = f" - Details: {details}" if details else ""
    logger.info(f"Performance - Operation: {operation}, Duration: {duration:.3f}s{details_str}")