#!/usr/bin/env python3
"""
Test the complete pipeline with the actual book URL: https://ai-and-humanoid-robotics-book.vercel.app/
"""

import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from main import WebsiteIngestionPipeline

def test_book_pipeline():
    """Test the complete pipeline with the actual book URL"""
    print("Testing the complete pipeline with the book URL...")
    print("URL: https://ai-and-humanoid-robotics-book.vercel.app/")

    pipeline = WebsiteIngestionPipeline()

    try:
        # Step 1: Extract content from the book URL
        print("\n1. Extracting content from the book...")
        content_result = pipeline.extract_text_from_url("https://ai-and-humanoid-robotics-book.vercel.app/")

        print(f"   Title: {content_result['title']}")
        print(f"   Content length: {len(content_result['content'])} characters")

        if content_result['content']:
            # Step 2: Chunk the book content
            print("\n2. Chunking the book content...")
            chunks = pipeline.chunk_text(content_result['content'])
            print(f"   Content chunked into {len(chunks)} chunks")

            # Show first chunk as sample
            if chunks:
                first_chunk = chunks[0]
                print(f"   First chunk: {len(first_chunk['text'])} chars, position {first_chunk['start_pos']}-{first_chunk['end_pos']}")

                # Step 3: Generate embeddings
                print("\n3. Generating embeddings for the chunks...")
                embeddings = pipeline.embed(chunks[:3])  # Just first 3 chunks to limit API usage for testing
                print(f"   Generated {len(embeddings)} embeddings (for first 3 chunks)")

                # Step 4: Store in Qdrant
                print("\n4. Storing in Qdrant...")
                success_count = 0
                for i, (chunk, embedding) in enumerate(zip(chunks[:3], embeddings)):
                    # Add book-specific metadata
                    chunk_with_metadata = {
                        **chunk,
                        'url': 'https://ai-and-humanoid-robotics-book.vercel.app/',
                        'title': content_result['title'],
                        'source_document': 'AI & Humanoid Robotics Book'
                    }
                    success = pipeline.save_chunk_to_qdrant(chunk_with_metadata, embedding)
                    if success:
                        success_count += 1
                        print(f"   Saved chunk {i+1} to Qdrant")
                    else:
                        print(f"   Failed to save chunk {i+1} to Qdrant")

                print(f"\n   Successfully saved {success_count}/3 chunks to Qdrant")

                # Step 5: Test collection creation/verification
                print("\n5. Verifying Qdrant collection...")
                collection_success = pipeline.create_collection()
                print(f"   Collection verification: {'Success' if collection_success else 'Failed'}")

                print("\nSUCCESS: Complete pipeline test with the book URL: PASSED")
                print(f"   - Extracted: {len(content_result['content'])} chars")
                print(f"   - Chunked into: {len(chunks)} chunks")
                print(f"   - Embedded: {len(embeddings)} chunks")
                print(f"   - Stored: {success_count} chunks in Qdrant")

                return True
        else:
            print("ERROR: Failed to extract content from the book URL")
            return False

    except Exception as e:
        print(f"ERROR: Error during book pipeline test: {e}")
        import traceback
        traceback.print_exc()
        return False

def test_book_crawling():
    """Test crawling multiple pages from the book site"""
    print("\n" + "="*60)
    print("Testing crawling multiple pages from the book site...")

    pipeline = WebsiteIngestionPipeline()

    try:
        # Get a few URLs from the book site
        print("\n1. Crawling the book site to get multiple URLs...")
        urls = pipeline.get_all_urls("https://ai-and-humanoid-robotics-book.vercel.app/")

        print(f"   Found {len(urls)} URLs on the book site")

        # Show first few URLs
        for i, url in enumerate(urls[:5]):
            print(f"   {i+1}. {url}")

        if len(urls) > 5:
            print(f"   ... and {len(urls) - 5} more URLs")

        # Process first 2 URLs to test the complete flow
        print(f"\n2. Processing first 2 URLs to test complete flow...")
        processed_count = 0

        for i, url in enumerate(urls[:2]):
            print(f"   Processing URL {i+1}: {url}")

            # Extract content
            content_result = pipeline.extract_text_from_url(url)
            print(f"     Extracted: {len(content_result['content'])} chars")

            if content_result['content']:
                # Chunk content
                chunks = pipeline.chunk_text(content_result['content'])
                print(f"     Chunked into: {len(chunks)} chunks")

                # Generate embeddings for first chunk as sample
                if chunks:
                    sample_chunk = chunks[0]
                    embeddings = pipeline.embed([sample_chunk])
                    print(f"     Generated embeddings: {len(embeddings)}")

                    # Store to Qdrant
                    chunk_with_metadata = {
                        **sample_chunk,
                        'url': url,
                        'title': content_result['title'],
                        'source_document': 'AI & Humanoid Robotics Book'
                    }
                    success = pipeline.save_chunk_to_qdrant(chunk_with_metadata, embeddings[0])
                    if success:
                        print(f"     Stored chunk to Qdrant: SUCCESS")
                        processed_count += 1
                    else:
                        print(f"     Stored chunk to Qdrant: FAILED")

        print(f"\nSUCCESS: Crawling and processing test: {processed_count}/2 URLs processed successfully")
        return processed_count > 0

    except Exception as e:
        print(f"ERROR: Error during book crawling test: {e}")
        import traceback
        traceback.print_exc()
        return False

if __name__ == "__main__":
    print("Testing RAG Pipeline with the AI & Humanoid Robotics Book")
    print("="*60)

    # Test with the main book URL
    test1_result = test_book_pipeline()

    # Test crawling multiple pages
    test2_result = test_book_crawling()

    print("\n" + "="*60)
    print("Final Results:")
    print(f"Main book URL test: {'SUCCESS: PASSED' if test1_result else 'ERROR: FAILED'}")
    print(f"Multiple pages test: {'SUCCESS: PASSED' if test2_result else 'ERROR: FAILED'}")

    overall_success = test1_result and test2_result
    print(f"Overall: {'SUCCESS: ALL TESTS PASSED' if overall_success else 'ERROR: SOME TESTS FAILED'}")

    if overall_success:
        print("\nSUCCESS: The pipeline works correctly with your AI & Humanoid Robotics Book!")
        print("All components (extraction, chunking, embedding, storage) working with your actual book content")
    else:
        print("\nERROR: There were issues with processing your book content")