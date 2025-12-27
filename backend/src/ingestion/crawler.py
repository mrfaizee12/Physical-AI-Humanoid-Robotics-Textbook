import asyncio
import aiohttp
from typing import List, Dict, Optional, Tuple
from urllib.parse import urljoin, urlparse
import re
from bs4 import BeautifulSoup
import xml.etree.ElementTree as ET
from src.config import settings
from src.utils.logging import get_logger


logger = get_logger(__name__)


class WebCrawler:
    """
    Crawler to fetch content from TARGET_WEBSITE_URL and SITEMAP_URL
    """

    def __init__(self):
        self.session = None
        self.visited_urls = set()

    async def __aenter__(self):
        self.session = aiohttp.ClientSession(
            timeout=aiohttp.ClientTimeout(total=30),
            headers={'User-Agent': 'Physical-AI-Robotics-Crawler/1.0'}
        )
        return self

    async def __aexit__(self, exc_type, exc_val, exc_tb):
        if self.session:
            await self.session.close()

    async def fetch_sitemap_urls(self, sitemap_url: str) -> List[str]:
        """
        Fetch URLs from the sitemap
        """
        try:
            async with self.session.get(sitemap_url) as response:
                content = await response.text()

            # Parse XML sitemap
            root = ET.fromstring(content)
            urls = []

            # Handle both regular sitemaps and sitemap indexes
            for element in root:
                # Remove namespace if present
                tag = element.tag.split('}')[-1] if '}' in element.tag else element.tag

                if tag == 'url':
                    for loc in element:
                        loc_tag = loc.tag.split('}')[-1] if '}' in loc.tag else loc.tag
                        if loc_tag == 'loc':
                            urls.append(loc.text.strip())
                elif tag == 'sitemap':
                    for loc in element:
                        loc_tag = loc.tag.split('}')[-1] if '}' in loc.tag else loc.tag
                        if loc_tag == 'loc':
                            # Recursively fetch URLs from nested sitemaps
                            nested_urls = await self.fetch_sitemap_urls(loc.text.strip())
                            urls.extend(nested_urls)

            return urls
        except Exception as e:
            logger.error(f"Error fetching sitemap {sitemap_url}: {e}")
            return []

    async def fetch_page_content(self, url: str) -> Optional[Tuple[str, str]]:
        """
        Fetch content from a single page
        Returns (title, content) or None if failed
        """
        try:
            if url in self.visited_urls:
                return None

            self.visited_urls.add(url)

            async with self.session.get(url) as response:
                if response.status != 200:
                    logger.warning(f"Failed to fetch {url}, status: {response.status}")
                    return None

                html_content = await response.text()

            soup = BeautifulSoup(html_content, 'html.parser')

            # Remove script and style elements
            for script in soup(["script", "style"]):
                script.decompose()

            # Extract title
            title_tag = soup.find('title')
            title = title_tag.get_text().strip() if title_tag else urlparse(url).path.split('/')[-1] or 'Untitled'

            # Extract main content - try common content containers first
            content_selectors = [
                'main', 'article', '.content', '#content', '.post-content',
                '.article-content', '.markdown', '.doc-content', 'body'
            ]

            content = ""
            for selector in content_selectors:
                element = soup.select_one(selector)
                if element:
                    content = element.get_text(separator=' ', strip=True)
                    break

            # If no content found in specific containers, get from body
            if not content:
                body = soup.find('body')
                if body:
                    content = body.get_text(separator=' ', strip=True)

            # Clean up content
            content = re.sub(r'\s+', ' ', content).strip()

            if len(content) < 50:  # Reduce minimum content length to capture more pages
                logger.warning(f"Content too short for {url}: {len(content)} chars")
                return None

            return title, content

        except Exception as e:
            logger.error(f"Error fetching page {url}: {e}")
            return None

    async def crawl_website(self) -> List[Dict[str, str]]:
        """
        Crawl ALL pages from the sitemap_url (not just target_website_url)
        Returns list of {url, title, content} dictionaries
        """
        all_pages = []

        # Get the sitemap URL from configuration
        sitemap_url = settings.sitemap_url
        if not sitemap_url:
            logger.error("SITEMAP_URL is not set in configuration")
            return []

        logger.info(f"Fetching URLs from sitemap: {sitemap_url}")
        urls = await self.fetch_sitemap_urls(sitemap_url)

        if not urls:
            logger.warning(f"No URLs found in sitemap: {sitemap_url}")
            # Fallback to crawling just the target URL if no sitemap URLs found
            target_url = settings.target_website_url
            if target_url:
                logger.info(f"Falling back to crawling target website: {target_url}")
                result = await self.fetch_page_content(target_url)
                if result:
                    title, content = result
                    all_pages.append({
                        'url': target_url,
                        'title': title,
                        'content': content
                    })
                    logger.info(f"Successfully crawled: {title[:100]}...")
            return all_pages

        logger.info(f"Found {len(urls)} URLs in sitemap, starting crawl...")

        # Crawl each URL from the sitemap
        for i, url in enumerate(urls):
            logger.info(f"Crawling {i+1}/{len(urls)}: {url}")
            result = await self.fetch_page_content(url)

            if result:
                title, content = result
                all_pages.append({
                    'url': url,
                    'title': title,
                    'content': content
                })
                logger.info(f"Successfully crawled: {title[:100]}...")
            else:
                logger.warning(f"Failed to crawl URL: {url}")

        logger.info(f"Completed crawling {len(all_pages)} out of {len(urls)} URLs from sitemap")
        return all_pages


async def crawl_textbook_content() -> List[Dict[str, str]]:
    """
    Main function to crawl textbook content
    """
    async with WebCrawler() as crawler:
        pages = await crawler.crawl_website()
        return pages


if __name__ == "__main__":
    # Test the crawler
    async def test_crawler():
        pages = await crawl_textbook_content()
        print(f"Crawled {len(pages)} pages")
        for page in pages[:3]:  # Show first 3 pages
            print(f"URL: {page['url']}")
            print(f"Title: {page['title'][:100]}...")
            print(f"Content length: {len(page['content'])}")
            print("---")

    asyncio.run(test_crawler())