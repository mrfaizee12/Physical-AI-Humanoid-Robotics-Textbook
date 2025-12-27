---
id: 1
title: Floating Chatbot Implementation
stage: red
date: 2025-12-20
surface: agent
model: claude-sonnet-4-5-20250929
feature: rag-frontend
branch: 001-real-rag-pipeline
user: unknown
command: manual
labels: ["rag", "chatbot", "ui", "docusaurus", "frontend"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - book/src/components/FloatingChatbot/index.tsx
 - book/src/pages/index.tsx
tests:
 - none
---

## Prompt

Replace the inline "Ask about the Textbook" UI with a standard floating chatbot.

Requirements:
1. Add a floating chat icon fixed at bottom-right.
2. On click, open/close a chatbot panel.
3. Chat panel must include:
   - Header: "Ask about the Textbook"
   - Message history (user + assistant)
   - Input + Send button
4. Show clickable suggested questions on open:
   - "What is Physical AI?"
   - "What is this book about?"
   - "Explain Module 1"
   - "Explain Module 2"
5. Clicking a suggestion sends it automatically.
6. Render real responses from RAG API (content + citations).

Constraints:
- Use existing React/Docusaurus setup.
- No backend or API schema changes.
- Minimal, clean chatbot-style UI.

Acceptance:
- Floating icon visible.
- Chat opens like a real chatbot.
- Suggestions visible.
- RAG answers display correctly.

## Response snapshot

Successfully implemented a floating chatbot component that replaces the inline "Ask about the Textbook" UI. The implementation includes all required features: a floating chat icon at the bottom-right, a collapsible chat panel with header, message history display for both user and assistant messages, input field with send button, suggested questions that appear when the chat opens, and integration with the existing RAG API for real responses with citations.

## Outcome

- ✅ Impact: Replaced inline RAG component with floating chatbot UI, improving user experience with persistent access to textbook Q&A
- 🧪 Tests: none
- 📁 Files: book/src/components/FloatingChatbot/index.tsx, book/src/pages/index.tsx
- 🔁 Next prompts: none
- 🧠 Reflection: The implementation successfully maintains all existing RAG functionality while providing a more user-friendly interface with persistent access.

## Evaluation notes (flywheel)

- Failure modes observed: none
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Add additional suggested questions or improve UI styling