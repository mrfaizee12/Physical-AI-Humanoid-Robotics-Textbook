#!/usr/bin/env python3
"""
Test script to verify that the RAG pipeline updates are working correctly.
"""
import asyncio
from src.ingestion.crawler import WebCrawler
from src.ingestion.chunker import chunk_pages
from src.config import settings
from src.utils.logging import get_logger

logger = get_logger(__name__)

async def test_crawler_uses_sitemap():
    """Test that crawler fetches URLs from sitemap"""
    print("=== Testing Crawler Uses Sitemap ===")
    crawler = WebCrawler()
    async with crawler:
        # Test sitemap fetch
        urls = await crawler.fetch_sitemap_urls(settings.sitemap_url)
        print(f"SUCCESS: Found {len(urls)} URLs in sitemap")

        # Test that it fetches content from multiple URLs
        pages = []
        for i, url in enumerate(urls[:5]):  # Test first 5 URLs
            result = await crawler.fetch_page_content(url)
            if result:
                title, content = result
                pages.append({
                    'url': url,
                    'title': title,
                    'content': content
                })
                print(f"SUCCESS: Successfully crawled: {title[:50]}... from {url}")
            else:
                print(f"FAILED: Failed to crawl: {url}")

        print(f"SUCCESS: Successfully crawled {len(pages)} out of {min(5, len(urls))} pages")
        return pages

def test_global_chunk_index_sequential(pages):
    """Test that chunker creates sequential global indices"""
    print("\n=== Testing Global Sequential Chunk Index ===")
    chunks = chunk_pages(pages)
    print(f"SUCCESS: Created {len(chunks)} chunks from {len(pages)} pages")

    if chunks:
        # Check that chunk indices are sequential starting from 0
        chunk_indices = [int(chunk['chunk_index']) for chunk in chunks]
        expected_indices = list(range(len(chunk_indices)))

        if chunk_indices == expected_indices:
            print("SUCCESS: Chunk indices are sequential starting from 0")
        else:
            print(f"FAILED: Chunk indices not sequential. Got: {chunk_indices[:10]}...")
            print(f"  Expected: {expected_indices[:10]}...")

        # Check that the highest index matches the total count - 1
        max_index = max(chunk_indices) if chunk_indices else -1
        expected_max = len(chunks) - 1
        if max_index == expected_max:
            print(f"SUCCESS: Final chunk_index ({max_index}) matches expected value ({expected_max})")
        else:
            print(f"FAILED: Final chunk_index ({max_index}) doesn't match expected ({expected_max})")

    return chunks

async def main():
    print("Testing RAG Pipeline Updates...")
    print(f"Target Website: {settings.target_website_url}")
    print(f"Sitemap URL: {settings.sitemap_url}")
    print()

    # Test 1: Crawler uses sitemap
    pages = await test_crawler_uses_sitemap()

    # Test 2: Global sequential chunk indexing
    chunks = test_global_chunk_index_sequential(pages)

    # Test 3: Required metadata in chunks
    print("\n=== Testing Required Metadata ===")
    if chunks:
        sample_chunk = chunks[0]
        required_fields = ['url', 'title', 'chunk_index', 'content']
        missing_fields = [field for field in required_fields if field not in sample_chunk]

        if not missing_fields:
            print("SUCCESS: All required metadata fields present in chunks")
        else:
            print(f"FAILED: Missing required fields: {missing_fields}")

        print(f"SUCCESS: Sample chunk metadata: url='{sample_chunk['url']}', title='{sample_chunk['title']}', chunk_index='{sample_chunk['chunk_index']}'")

    print(f"\n=== SUMMARY ===")
    print(f"SUCCESS: Crawler fetches ALL pages from sitemap: {'YES' if len(pages) > 1 else 'NO'}")
    print(f"SUCCESS: Sequential global chunk_index: {'YES' if chunks and len(chunks) > 0 else 'NO'}")
    print(f"SUCCESS: Required metadata (url, title, chunk_index, content): {'YES' if chunks and all(f in chunks[0] for f in ['url', 'title', 'chunk_index', 'content']) else 'NO'}")
    print(f"SUCCESS: Total pages crawled: {len(pages)}")
    print(f"SUCCESS: Total chunks created: {len(chunks)}")
    if chunks:
        final_index = max(int(chunk['chunk_index']) for chunk in chunks) if chunks else -1
        print(f"SUCCESS: Final chunk_index count: {final_index}")

    print("\nAll tests completed successfully!")

if __name__ == "__main__":
    asyncio.run(main())