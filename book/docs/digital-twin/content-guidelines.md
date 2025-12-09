---
sidebar_position: 99
title: "Content Guidelines"
---

# Content Guidelines for Digital Twin Module

This document outlines the content guidelines for the Digital Twin module to ensure consistency and compliance with RAG chunking limits.

## RAG Chunking Limits

Each documentation page must contain **≤1,500 tokens** to maintain compatibility with the RAG (Retrieval-Augmented Generation) system. This ensures that:

- Content is chunked appropriately for the chatbot knowledge base
- Information remains coherent within each chunk
- Performance is maintained during retrieval operations

## Content Structure

To maintain pages under the 1,500 token limit:

1. **Break down complex topics** into smaller, focused sections
2. **Use progressive disclosure** - introduce concepts gradually
3. **Include examples with explanations** within the same page when possible
4. **Link to related content** rather than including everything in one place

## Token Counting

While exact token counting varies by model, you can estimate tokens by:

- Counting approximately 1 token per 4 characters
- Counting 1 token per common word
- Using online token counting tools for verification

## Writing Style

- Use clear, concise language
- Follow IEEE citation format for all references
- Include code examples with proper syntax highlighting
- Provide step-by-step instructions for complex procedures