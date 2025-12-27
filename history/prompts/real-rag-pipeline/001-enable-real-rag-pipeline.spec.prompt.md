---
id: 1
title: enable-real-rag-pipeline
stage: spec
date: 2025-12-19
surface: agent
model: Claude Sonnet 4.5
feature: real-rag-pipeline
branch: 001-real-rag-pipeline
user: Claude Code User
command: /sp.specify
labels: [rag, pipeline, textbook, ai, backend]
links:
  spec: G:\textbook\specs\001-real-rag-pipeline\spec.md
  ticket: null
  adr: null
  pr: null
files:
 - G:\textbook\specs\001-real-rag-pipeline\spec.md
 - G:\textbook\specs\001-real-rag-pipeline\checklists\requirements.md
tests:
 - No tests created at specification stage
---

## Prompt

Enable the **real RAG pipeline** so the chatbot returns **actual textbook-based answers**.

**Goal**
Remove all mock responses and activate Qdrant + Cohere + Gemini end-to-end.

**Requirements**
- `/api/rag/query` must:
  1. Create embeddings with **Cohere**
  2. Retrieve chunks from **Qdrant** (`rag_embedding`)
  3. Send context to **Gemini** via OpenAI-compatible SDK
  4. Return real answers + citations
- If no context found, return exactly:
  **"I don't know based on the textbook."**
- **Do not change** response schema.

**LLM (MANDATORY)**
- Base URL: `https://generativelanguage.googleapis.com/v1beta/openai/`
- Model: `gemini-2.0-flash`
- API key: `GEMINI_API_KEY` (env)

**Env (do not hardcode)**
`QDRANT_URL`, `QDRANT_API_KEY`, `QDRANT_COLLECTION_NAME=rag_embedding`,
`COHERE_API_KEY`, `RAG_AGENT_URL`, `TARGET_WEBSITE_URL`, `SITEMAP_URL`

## Response snapshot

Created comprehensive feature specification for the real RAG pipeline that enables textbook-based answers by integrating Qdrant, Cohere, and Gemini. The specification includes user scenarios, functional requirements, and success criteria while maintaining backward compatibility.

## Outcome

- ✅ Impact: Created complete feature specification for real RAG pipeline implementation
- 🧪 Tests: No tests created at specification stage
- 📁 Files: Created spec.md and requirements checklist
- 🔁 Next prompts: Ready for planning phase with /sp.plan
- 🧠 Reflection: Specification addresses all requirements with proper security and compatibility considerations

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): PASS - All specification sections completed
- Prompt variant (if applicable): None
- Next experiment (smallest change to try): Proceed to planning phase