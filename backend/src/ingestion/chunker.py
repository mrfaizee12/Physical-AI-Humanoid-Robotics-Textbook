import re
from typing import List, Dict
from src.utils.logging import get_logger


logger = get_logger(__name__)


class TextChunker:
    """
    Text chunker to split content into fixed-size chunks with sequential chunk_index
    """

    def __init__(self, min_chunk_size: int = 500, max_chunk_size: int = 800):
        self.min_chunk_size = min_chunk_size
        self.max_chunk_size = max_chunk_size

    def estimate_tokens(self, text: str) -> int:
        """
        Estimate number of tokens in text (rough approximation: 1 token ~ 4 chars)
        """
        return len(text) // 4

    def split_by_sentences(self, text: str) -> List[str]:
        """
        Split text into sentences while preserving sentence boundaries
        """
        # Split by sentence endings, but keep the punctuation
        sentences = re.split(r'(?<=[.!?])\s+', text)
        # Remove empty strings and strip whitespace
        sentences = [s.strip() for s in sentences if s.strip()]
        return sentences

    def chunk_text(self, text: str, url: str, title: str) -> List[Dict[str, str]]:
        """
        Split text into fixed-size chunks with sequential chunk_index starting from 0
        Returns list of {url, title, chunk_index, content} dictionaries
        """
        return self.chunk_text_global_index(text, url, title, 0)

    def chunk_text_global_index(self, text: str, url: str, title: str, start_index: int = 0) -> List[Dict[str, str]]:
        """
        Split text into fixed-size chunks with sequential chunk_index starting from a given index
        Enhanced to extract module and section information from the URL and content structure
        Returns list of {url, title, chunk_index, content, module, section} dictionaries
        """
        if not text.strip():
            return []

        sentences = self.split_by_sentences(text)
        chunks = []
        current_chunk = ""
        chunk_index = start_index

        # Extract module and section information from URL
        module_info = self._extract_module_info_from_url(url)

        for sentence in sentences:
            # Check if adding this sentence would exceed max chunk size
            test_chunk = current_chunk + " " + sentence if current_chunk else sentence

            if self.estimate_tokens(test_chunk) <= self.max_chunk_size:
                current_chunk = test_chunk
            else:
                # If current chunk is already larger than min size, save it
                if self.estimate_tokens(current_chunk) >= self.min_chunk_size:
                    chunks.append({
                        'url': url,
                        'title': title,
                        'chunk_index': str(chunk_index),
                        'content': current_chunk.strip(),
                        'module': module_info.get('module', ''),
                        'section': module_info.get('section', ''),
                        'subsection': module_info.get('subsection', '')
                    })
                    chunk_index += 1
                    current_chunk = sentence
                else:
                    # If current chunk is too small, force split at max size
                    if current_chunk:
                        # Split current content into max_size pieces
                        sub_chunks = self._force_chunk(current_chunk, self.max_chunk_size)
                        for sub_chunk in sub_chunks:
                            chunks.append({
                                'url': url,
                                'title': title,
                                'chunk_index': str(chunk_index),
                                'content': sub_chunk.strip(),
                                'module': module_info.get('module', ''),
                                'section': module_info.get('section', ''),
                                'subsection': module_info.get('subsection', '')
                            })
                            chunk_index += 1
                    current_chunk = sentence

        # Add the last chunk if it has content
        if current_chunk.strip():
            chunks.append({
                'url': url,
                'title': title,
                'chunk_index': str(chunk_index),
                'content': current_chunk.strip(),
                'module': module_info.get('module', ''),
                'section': module_info.get('section', ''),
                'subsection': module_info.get('subsection', '')
            })

        logger.info(f"Split content from {url} into {len(chunks)} chunks starting from index {start_index}")
        return chunks

    def _extract_module_info_from_url(self, url: str) -> Dict[str, str]:
        """
        Extract module and section information from the URL structure
        Example: https://physical-ai-humanoid-robotics-textb-nu.vercel.app/module-2/section-3
        would return {'module': 'Module 2', 'section': 'Section 3'}
        """
        import re
        from urllib.parse import urlparse

        parsed_url = urlparse(url)
        path_parts = [part for part in parsed_url.path.split('/') if part]

        module_info = {'module': '', 'section': '', 'subsection': ''}

        # Look for patterns like module-1, module_1, module1, etc.
        for i, part in enumerate(path_parts):
            # Match module patterns (module-1, module_1, module1, etc.)
            module_match = re.search(r'(?:module)[-_]?(\d+)', part, re.IGNORECASE)
            if module_match:
                module_info['module'] = f"Module {module_match.group(1)}"

            # Match section patterns (section-1, section_1, section1, etc.)
            section_match = re.search(r'(?:section)[-_]?(\d+)', part, re.IGNORECASE)
            if section_match and not module_info['section']:
                module_info['section'] = f"Section {section_match.group(1)}"

            # Match subsection patterns
            subsection_match = re.search(r'(?:subsection|sub-section)[-_]?(\d+)', part, re.IGNORECASE)
            if subsection_match:
                module_info['subsection'] = f"Subsection {subsection_match.group(1)}"

        return module_info

    def _force_chunk(self, text: str, max_size: int) -> List[str]:
        """
        Force split text into chunks of max_size characters when sentence boundaries don't work
        """
        if len(text) <= max_size:
            return [text]

        chunks = []
        start = 0

        while start < len(text):
            end = start + max_size

            # If we're not at the end, try to break at a word boundary
            if end < len(text):
                # Look for the last space within the chunk
                space_pos = text.rfind(' ', start, end)
                if space_pos > start + max_size // 2:  # Only break if it's not too early
                    end = space_pos

            chunks.append(text[start:end].strip())
            start = end

            # If we didn't find a good word boundary, just break at max_size
            if start < len(text) and start == end - max_size:
                start = end

        return [chunk for chunk in chunks if chunk.strip()]


def chunk_pages(pages: List[Dict[str, str]]) -> List[Dict[str, str]]:
    """
    Process a list of pages and chunk them with globally sequential chunk_index
    """
    chunker = TextChunker()
    all_chunks = []
    global_chunk_index = 0

    for page in pages:
        url = page['url']
        title = page['title']
        content = page['content']

        logger.info(f"Chunking page: {title[:100]}...")

        chunks = chunker.chunk_text_global_index(content, url, title, global_chunk_index)

        # Update the global index for the next page
        global_chunk_index += len(chunks)
        all_chunks.extend(chunks)

    logger.info(f"Total chunks created: {len(all_chunks)} from {len(pages)} pages")
    return all_chunks


if __name__ == "__main__":
    # Test the chunker
    test_content = """
    This is a sample text for testing the chunker. It contains multiple sentences that should be properly split.
    The chunker should respect sentence boundaries while maintaining the target token size.
    Each chunk should have a sequential index starting from 0 for each page.
    Additional content to make the text longer and test the chunking logic properly.
    """ * 10  # Repeat to make it longer

    chunker = TextChunker()
    chunks = chunker.chunk_text(test_content, "https://example.com/test", "Test Page")

    print(f"Created {len(chunks)} chunks:")
    for i, chunk in enumerate(chunks):
        print(f"Chunk {i} (Index {chunk['chunk_index']}): {len(chunk['content'])} chars")
        print(f"Content preview: {chunk['content'][:100]}...")
        print("---")

    # Test the global indexing method
    print("\nTesting global indexing (starting from index 5):")
    chunks_global = chunker.chunk_text_global_index(test_content, "https://example.com/test", "Test Page", 5)

    print(f"Created {len(chunks_global)} chunks with global indexing:")
    for i, chunk in enumerate(chunks_global):
        print(f"Chunk {i} (Index {chunk['chunk_index']}): {len(chunk['content'])} chars")
        print(f"Content preview: {chunk['content'][:100]}...")
        print("---")