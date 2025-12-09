---
id: 0001
title: Update Project Constitution for Book and Chatbot
stage: constitution
date: 2025-12-08
surface: agent
model: claude-sonnet-4-5-20250929
feature: none
branch: master
user: [USER]
command: /sp.constitution
labels: ["constitution", "initial-setup"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - .specify/memory/constitution.md
tests:

---

## Prompt

Project: Book + Embedded RAG Chatbot on Physical AI & Humanoid Robotics

Principles:
- Accuracy from official docs + peer-reviewed robotics/AI sources
- Clear instructional style for senior CS/AI learners
- Fully reproducible with Spec-Kit Plus + Claude Code
- Book and chatbot content must stay consistent

Standards:
- Sources: ≥40% peer-reviewed; rest from official frameworks (ROS, NVIDIA, OpenAI)
- Code must run via Claude Code and follow Spec-Kit constraints
- Use IEEE citations; no uncited claims

Book Requirements:
- Built with Docusaurus and deployed to GitHub Pages
- Includes modular lessons + capstone + reproducible code
- Page chunks ≤1,500 tokens for RAG compatibility

Chatbot Requirements:
- Built with FastAPI + ChatKit/OpenAI + Neon Postgres + Qdrant
- Answers strictly from book content unless override
- Must support user-selected text querying

Success Criteria:
- Full reproducible deployment (book + chatbot)
- Working humanoid simulation demo (ROS 2 + Gazebo/Isaac)
- Capstone must show: voic

## Response snapshot

The project constitution has been updated to version 1.0.0, incorporating the provided principles, standards, requirements, and success criteria for the "Book + Embedded RAG Chatbot on Physical AI & Humanoid Robotics" project. Several templates and guidance documents are flagged for manual review to ensure consistency.

## Outcome

- ✅ Impact: Project constitution updated with initial principles, standards, requirements, and success criteria.
- 🧪 Tests: No tests were run or added.
- 📁 Files: One file was modified: .specify/memory/constitution.md
- 🔁 Next prompts: Review and update dependent templates as per the Sync Impact Report. Clarify capstone functionality.
- 🧠 Reflection: Initial constitution successfully generated and written. Manual follow-up for dependent files and a clarification are pending.

## Evaluation notes (flywheel)

- Failure modes observed: Bash script for PHR creation not found, resorted to agent-native tools.
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Improve PHR creation robustnes by always checking for script existence first or providing a more explicit agent-native fallback.
