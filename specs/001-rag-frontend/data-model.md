# Data Model: Frontend Integration for RAG Agent

## Entities

### UserQuery
- **id**: string (unique identifier for the query session)
- **content**: string (the natural language question from the user)
- **timestamp**: datetime (when the query was submitted)
- **status**: enum ["pending", "processing", "completed", "error"] (current state of the query)
- **userId**: string (optional, for session tracking)

### GroundedResponse
- **id**: string (unique identifier matching the query id)
- **content**: string (the response text from the RAG agent)
- **citations**: array of CitationReference objects (sources used in the response)
- **confidence**: number (0-1, confidence score of the response)
- **timestamp**: datetime (when the response was generated)
- **queryId**: string (reference to the original query)

### CitationReference
- **id**: string (unique identifier for the citation)
- **text**: string (the cited text from the source)
- **sourceUrl**: string (URL to the source document/section)
- **pageReference**: string (page number or section reference)
- **context**: string (surrounding context of the citation)

### QuerySession
- **id**: string (unique session identifier)
- **startTime**: datetime (when the session started)
- **queries**: array of UserQuery objects (queries made in this session)
- **active**: boolean (whether the session is still active)
- **userId**: string (optional, for tracking user sessions)

## Validation Rules

### UserQuery Validation
- content: Required, minimum 3 characters, maximum 1000 characters
- status: Must be one of the allowed enum values
- timestamp: Must be in ISO 8601 format

### GroundedResponse Validation
- content: Required, minimum 1 character
- citations: Array must contain 0 or more CitationReference objects
- confidence: Must be between 0 and 1
- queryId: Must reference an existing UserQuery

### CitationReference Validation
- sourceUrl: Must be a valid URL format
- pageReference: Required if source is from textbook content
- text: Required, minimum 1 character

## State Transitions

### UserQuery States
```
pending → processing → completed
              ↓
            error
```

- A query starts in "pending" state when submitted
- Moves to "processing" when sent to the RAG agent
- Transitions to "completed" when a response is received
- Moves to "error" if there's a failure during processing

### QuerySession States
```
active → inactive (on session end)
```

- A session is "active" while the user is interacting
- Becomes "inactive" when the session ends (timeout or user action)