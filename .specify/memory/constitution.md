<!--
Sync Impact Report:
- Version change: 0.1.0 -> 1.0.0
- Description: Initial population of the project constitution with principles, standards, and governance for the "Book + Embedded RAG Chatbot on Physical AI & Humanoid Robotics" project.
- Sections Added:
  - Core Principles
  - Standards and Requirements
  - Governance
- Templates requiring updates:
  - ✅ .specify/templates/plan-template.md (No changes needed)
  - ✅ .specify/templates/spec-template.md (No changes needed)
  - ✅ .specify/templates/tasks-template.md (No changes needed)
  - ✅ README.md (No changes needed)
-->
# Book + Embedded RAG Chatbot on Physical AI & Humanoid Robotics Constitution

## Core Principles

### I. Content Accuracy
All content must be grounded in official documentation and peer-reviewed sources from the robotics and AI fields. Uncited claims are not permitted. All sources must be cited using IEEE format.

### II. Instructional Clarity
The material must be presented in a clear, instructional style suitable for senior computer science and AI learners. The content should be structured into modular lessons, a capstone project, and include reproducible code examples.

### III. Complete Reproducibility
All projects, code, and examples must be fully reproducible using Spec-Kit Plus and the specified AI coding assistants. Code must be runnable and adhere to the constraints defined in the Spec-Kit.

### IV. Content Consistency
The content of the book and the knowledge base of the RAG chatbot must be synchronized and consistent.

## Standards and Requirements

### Source Code and Citations
- A minimum of 40% of sources must be from peer-reviewed academic papers.
- The remainder of sources will come from official frameworks (e.g., ROS, NVIDIA Isaac, OpenAI).
- All claims and information must be accompanied by IEEE-style citations.

### Book Requirements
- The book will be built using Docusaurus and deployed to GitHub Pages.
- Content will be organized into modular lessons, a capstone project, and reproducible code.
- To ensure compatibility with the RAG system, individual pages (documents) will be chunked to a maximum of 1,500 tokens per chunk.

### Chatbot Requirements
- The chatbot will be built with a stack including FastAPI, ChatKit/OpenAI for the front-end and API, Neon Postgres for metadata, and Qdrant for the vector store.
- The chatbot's responses must be strictly derived from the book's content, with an explicit override mechanism for exceptional cases.
- The chatbot must support querying against user-selected text sections.

### Success Criteria & Definition of Done
- A full, reproducible deployment of both the Docusaurus book and the RAG chatbot is mandatory.
- A working humanoid simulation demo using ROS 2 and Gazebo/Isaac Sim must be produced.
- The capstone project must demonstrate a full "voice -> plan -> navigate -> perceive -> manipulate" pipeline.
- The final product must contain no undocumented claims or broken code.

## Governance

Amendments to this constitution require documentation, approval, and a migration plan for any affected components. All development activities, including pull requests and reviews, must verify compliance with these principles. Complexity in implementation must be explicitly justified against these principles.

**Version**: 1.0.0 | **Ratified**: 2025-12-09 | **Last Amended**: 2025-12-09