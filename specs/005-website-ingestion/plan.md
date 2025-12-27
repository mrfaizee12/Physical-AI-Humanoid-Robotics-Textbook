# Implementation Plan: Website Ingestion, Embeddings, and Vector DB Indexing

**Branch**: `005-website-ingestion` | **Date**: 2025-12-10 | **Spec**: [G:\textbook\specs\005-website-ingestion\spec.md](G:\textbook\specs\005-website-ingestion\spec.md)
**Input**: Feature specification from `/specs/005-website-ingestion/spec.md`

**Note**: This template is filled in by the `/sp.plan` command. See `.specify/templates/commands/plan.md` for the execution workflow.

## Summary

Implementation of a complete ingestion pipeline that fetches all Docusaurus book pages from the provided deployment link (https://physical-ai-humanoid-robotics-textb-nu.vercel.app/), extracts text content by reading URLs from the sitemap at https://physical-ai-humanoid-robotics-textb-nu.vercel.app/sitemap.xml, chunks it appropriately, generates Cohere embeddings, and stores them in Qdrant Cloud (rag_embedding collection). The pipeline will be contained in a single main.py file within a UV-managed backend project, with functions for URL discovery, text extraction, chunking, embedding, and vector storage.

## Technical Context

**Language/Version**: Python 3.11
**Primary Dependencies**: requests, beautifulsoup4, cohere, qdrant-client, python-dotenv, lxml
**Storage**: Qdrant Cloud (vector database)
**Testing**: pytest (for validation)
**Target Platform**: Linux server
**Project Type**: single (backend script)
**Performance Goals**: Process 500 pages within 30 minutes
**Constraints**: <100MB memory usage during processing, must handle rate limits from Cohere API
**Scale/Scope**: 500 pages, 10,000 text chunks, 1M vectors

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

### Compliance Verification

1. **Content Accuracy** ✅ - The system will process existing Docusaurus content from the specified deployment link (https://physical-ai-humanoid-robotics-textb-nu.vercel.app/), ensuring all ingested content is from the official source.

2. **Instructional Clarity** ✅ - The ingestion pipeline will maintain clear, modular structure with proper documentation and error handling.

3. **Complete Reproducibility** ✅ - The backend script will be fully reproducible with clear setup instructions and dependency management via UV.

4. **Content Consistency** ✅ - The vector database will maintain consistency with the original Docusaurus content through proper metadata tracking.

### Requirements Verification

- **Source Code and Citations** - N/A for this ingestion pipeline (the pipeline code itself doesn't require citations)
- **Book Requirements** - The pipeline will respect the 1,500 token chunking limit as specified in the constitution
- **Chatbot Requirements** - The pipeline will prepare content for the RAG system using Qdrant as specified
- **Success Criteria** - The pipeline will support the overall project goal of a working RAG system

### Gate Status: PASSED

## Project Structure

### Documentation (this feature)

```text
specs/[###-feature]/
├── plan.md              # This file (/sp.plan command output)
├── research.md          # Phase 0 output (/sp.plan command)
├── data-model.md        # Phase 1 output (/sp.plan command)
├── quickstart.md        # Phase 1 output (/sp.plan command)
├── contracts/           # Phase 1 output (/sp.plan command)
└── tasks.md             # Phase 2 output (/sp.tasks command - NOT created by /sp.plan)
```

### Source Code (repository root)

```text
backend/
├── pyproject.toml       # UV project configuration
├── uv.lock             # Dependency lock file
├── .env.example        # Environment variables template
├── main.py             # Main ingestion pipeline script
└── .env                # Local environment variables (gitignored)
```

**Structure Decision**: Single backend script approach selected to meet user requirements for a single main.py file containing all functionality. The backend folder will be created with UV project management for proper Python environment and dependencies. The sitemap at https://physical-ai-humanoid-robotics-textb-nu.vercel.app/sitemap.xml will be used to discover URLs from the target site https://physical-ai-humanoid-robotics-textb-nu.vercel.app/.

## Complexity Tracking

> **Fill ONLY if Constitution Check has violations that must be justified**

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| [e.g., 4th project] | [current need] | [why 3 projects insufficient] |
| [e.g., Repository pattern] | [specific problem] | [why direct DB access insufficient] |

### Post-Design Verification

After implementing the design, the solution continues to meet all constitution requirements:
1. **Content Accuracy** ✅ - The pipeline processes content from the official deployment link (https://physical-ai-humanoid-robotics-textb-nu.vercel.app/)
2. **Instructional Clarity** ✅ - The single-file design with clear function interfaces maintains clarity
3. **Complete Reproducibility** ✅ - The UV-managed project with clear setup instructions ensures reproducibility
4. **Content Consistency** ✅ - The pipeline maintains consistency with original content through proper metadata tracking
