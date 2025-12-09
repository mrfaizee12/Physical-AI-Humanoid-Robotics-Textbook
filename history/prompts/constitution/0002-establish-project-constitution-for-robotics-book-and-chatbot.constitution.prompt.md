---
id: 0002
title: Establish Project Constitution for Robotics Book and Chatbot
stage: constitution
date: 2025-12-09
surface: agent
model: gemini
feature: none
branch: 001-robotics-ros2-module
user: n/a
command: /sp.constitution
labels: [constitution, setup]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
 - .specify/memory/constitution.md
tests:
 - "none"
---

## Prompt

/sp.constitution Project: Book + Embedded RAG Chatbot on Physical AI & Humanoid Robotics

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
- Capstone must show: voice → plan → navigate → perceive → manipulate
- No undocumented claims; no broken code

## Response snapshot

The project constitution at `.specify/memory/constitution.md` was updated to version 1.0.0. The new constitution incorporates the user's specified principles, standards, and requirements for the "Book + Embedded RAG Chatbot on Physical AI & Humanoid Robotics" project. A sync report was prepended to the file.

## Outcome

- ✅ Impact: High. Established the foundational governance document for the project.
- 🧪 Tests: No tests were run.
- 📁 Files: Modified 1 file: `.specify/memory/constitution.md`.
- 🔁 Next prompts: The user can now proceed with other development tasks, knowing the project's guiding principles are documented.
- 🧠 Reflection: The process followed the spec-driven development model, starting with the definition of the project's constitution.

## Evaluation notes (flywheel)

- Failure modes observed: The shell script for PHR creation failed as expected on a Windows environment, and the agent correctly fell back to the native file creation method.
- Graders run and results (PASS/FAIL): n/a
- Prompt variant (if applicable): n/a
- Next experiment (smallest change to try): n/a
