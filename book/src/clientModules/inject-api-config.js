// Client module to inject API configuration into window object
// This runs before other client code and makes the API URL available in browser

// Set the API URL from environment variables that were available during build
// For production, this can be replaced with the actual production URL
window.RAG_API_URL = 'https://faizananjum-rag-chatbot.hf.space'; // Default for development