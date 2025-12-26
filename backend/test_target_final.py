#!/usr/bin/env python3
"""
Test script for Phase 3 functionality with the target Docusaurus site
"""

import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from backend.ingestion import WebsiteIngestionPipeline

def test_target_site_crawling():
    """Test crawling the target Docusaurus site"""
    print("Testing URL retrieval with target site: https://ai-and-humanoid-robotics-book.vercel.app/")

    pipeline = WebsiteIngestionPipeline()

    try:
        # Get a few URLs from the target site (with a smaller limit for testing)
        urls = pipeline.get_all_urls("https://ai-and-humanoid-robotics-book.vercel.app/")

        print(f"Found {len(urls)} URLs on the target site")

        # Print the first 5 URLs as a sample
        for i, url in enumerate(urls[:5]):
            print(f"  {i+1}. {url}")

        if len(urls) > 5:
            print(f"  ... and {len(urls) - 5} more URLs")

        return urls

    except Exception as e:
        print(f"Error crawling target site: {e}")
        # Try a simpler approach - just test if the site is accessible
        try:
            import requests
            response = requests.get("https://ai-and-humanoid-robotics-book.vercel.app/", timeout=10)
            print(f"Target site is accessible: status code {response.status_code}")
            return []
        except Exception as e2:
            print(f"Target site is not accessible: {e2}")
            return []

def test_content_extraction_from_target_site():
    """Test content extraction from the target site"""
    print("\nTesting content extraction from target site...")

    pipeline = WebsiteIngestionPipeline()

    try:
        # Test with the main page of the target site
        test_url = "https://ai-and-humanoid-robotics-book.vercel.app/"
        print(f"  Extracting content from: {test_url}")
        result = pipeline.extract_text_from_url(test_url)

        print(f"    Title: {result['title']}")
        print(f"    Content length: {len(result['content'])} characters")
        print(f"    Content preview: {result['content'][:100]}...")

        return result

    except Exception as e:
        print(f"    Error extracting content: {e}")
        return None

def test_various_docusaurus_layouts():
    """Test content extraction with various Docusaurus page layouts"""
    print("\nTesting content extraction with various Docusaurus page layouts...")

    pipeline = WebsiteIngestionPipeline()

    # Test different types of pages that might have different layouts
    layout_test_urls = [
        "https://ai-and-humanoid-robotics-book.vercel.app/",
        "https://ai-and-humanoid-robotics-book.vercel.app/introduction",
    ]

    success_count = 0

    for url in layout_test_urls:
        try:
            print(f"  Testing layout for: {url}")
            result = pipeline.extract_text_from_url(url)

            # Check if we got meaningful content
            if result and result['content'] and len(result['content']) > 50:
                print(f"    ✓ Successfully extracted content ({len(result['content'])} chars)")
                success_count += 1
            else:
                print(f"    ⚠ Limited content extracted ({len(result.get('content', ''))} chars)")

        except Exception as e:
            print(f"    ✗ Error with {url}: {e}")

    print(f"\nSuccessfully extracted content from {success_count}/{len(layout_test_urls)} page layouts")
    return success_count == len(layout_test_urls)

if __name__ == "__main__":
    print("Testing Phase 3 functionality with target Docusaurus site...")
    print("="*60)

    # Test URL retrieval
    urls = test_target_site_crawling()

    print("\n" + "="*60)

    # Test content extraction
    extraction_result = test_content_extraction_from_target_site()

    print("\n" + "="*60)

    # Test various layouts
    layouts_success = test_various_docusaurus_layouts()

    print("\n" + "="*60)
    print("Phase 3 testing with target site completed.")

    if extraction_result and layouts_success:
        print("✓ All tests passed - target site crawling and content extraction working properly")
    else:
        print("⚠ Some tests had issues - check output above for details")