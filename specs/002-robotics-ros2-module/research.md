# Research & Decisions for Book Architecture

This document outlines the key architectural and technical decisions made for the Docusaurus-based book, based on the requirements in the feature specification and best practices.

## 1. Book Structure and Navigation

The book's structure will be organized for a clear, linear learning path, primarily for "Module 1: Robotic Nervous System (ROS 2)".

- **Decision**: A hierarchical folder structure will be used within the `book/docs` directory. Each module will be a top-level folder, and chapters within it will be sub-folders or individual files.

- **Rationale**: This approach allows Docusaurus to auto-generate the sidebar navigation, reducing manual configuration. It also provides a logical content structure that mirrors the learning path.

- **Implementation**:
    -   The content for this module will live in `book/docs/01-ros2-basics/`.
    -   Files will be named `01-communication.md`, `02-rclpy-control.md`, etc.
    -   The `sidebar_position` front-matter attribute will be used in each file to enforce the correct chapter order.
    -   The `sidebar_label` attribute will be used for user-friendly names in the navigation sidebar (e.g., "1. Communication Basics").

## 2. Diagramming Strategy

Diagrams must be reproducible, version-controllable, and consistent with the site's theme.

- **Decision**:
    1.  **Primary Tool**: **Mermaid.js** will be the default tool for creating all standard diagrams, including flowcharts, sequence diagrams, and architectural graphs.
    2.  **Secondary Tool**: For custom illustrations or complex diagrams not supported by Mermaid, **SVG** will be used.

- **Rationale**:
    -   Mermaid is text-based, making it perfectly suited for version control. Docusaurus has native support for Mermaid, which includes automatic theming for light/dark modes.
    -   SVGs are also text-based and can be imported as React components in MDX files, allowing for styling via CSS and ensuring they are also version-controlled.

- **Implementation**:
    -   Enable the `@docusaurus/theme-mermaid` in `docusaurus.config.ts`.
    -   Mermaid syntax will be placed inside ````mermaid` fenced code blocks in Markdown files.
    -   Any custom SVGs will be stored in `book/static/img/` and embedded using the `<img />` tag or imported as React components if styling is required.

## 3. Versioning Strategy

The book's content is tightly coupled with the RAG chatbot's knowledge base. The versioning strategy must ensure they remain synchronized.

- **Decision**: Docusaurus's built-in documentation versioning feature will be used. Documentation versions will be tied to specific software releases of the overall project (including the chatbot).

- **Rationale**: This is the standard, recommended approach for versioning Docusaurus sites. It ensures that users (and the RAG agent) can access documentation corresponding to a specific, stable state of the project.

- **Implementation**:
    -   The content in the `book/docs/` directory will always represent the **"next"** (unreleased) version of the book.
    -   When a new project release is created (e.g., `v1.1.0`), the command `npm run docusaurus docs:version 1.1.0` will be executed. This snapshots the current `/docs` content into a versioned folder.
    -   The RAG chatbot's indexing pipeline will be configured to **only** ingest content from the latest *stable* versioned path (e.g., `/docs/1.1.0/...`), not from `/docs/next/...`. This prevents the chatbot from learning from in-progress or unreleased content.
