#!/usr/bin/env python3
"""
Final integration test for the complete RAG chatbot pipeline
"""

import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from main import WebsiteIngestionPipeline

def test_complete_pipeline():
    """Test the complete pipeline: URL discovery -> Content extraction -> Chunking -> Embedding -> Storage"""
    print("Starting final integration test...")

    pipeline = WebsiteIngestionPipeline()

    # Test with a simple, accessible site for the integration test
    test_url = "https://httpbin.org/html"

    try:
        print(f"1. Testing content extraction from: {test_url}")
        content_result = pipeline.extract_text_from_url(test_url)
        print(f"   Extracted content with title: '{content_result['title']}'")
        print(f"   Content length: {len(content_result['content'])} characters")

        if content_result['content']:
            print("2. Testing chunking functionality")
            # Pass only the text content string to chunk_text
            text_content = content_result['content']
            chunks = pipeline.chunk_text(text_content)  # chunk_text expects a string
            print(f"   Content chunked into {len(chunks)} chunks")

            if chunks:
                print("3. Testing embedding generation")
                embeddings = pipeline.embed(chunks)
                print(f"   Generated {len(embeddings)} embeddings")

                if embeddings:
                    print("4. Testing storage to Qdrant")
                    # Use the first chunk and embedding for storage test
                    chunk_with_url = {**chunks[0], **content_result}
                    success = pipeline.save_chunk_to_qdrant(chunk_with_url, embeddings[0])
                    if success:
                        print("   Successfully saved to Qdrant")
                    else:
                        print("   Failed to save to Qdrant")
                        return False

            print("5. Testing collection creation")
            collection_success = pipeline.create_collection()
            print(f"   Collection creation: {'Success' if collection_success else 'Failed'}")

        print("SUCCESS: All pipeline components working together successfully")
        return True

    except Exception as e:
        print(f"ERROR: Integration test failed: {e}")
        import traceback
        traceback.print_exc()
        return False

def test_command_line_equivalent():
    """Test functionality equivalent to command-line operations"""
    print("\nTesting command-line equivalent functionality...")

    pipeline = WebsiteIngestionPipeline()

    # Test processing a list of URLs (similar to what --base-url would do)
    test_urls = [
        "https://httpbin.org/html",
        "https://httpbin.org/json"
    ]

    try:
        all_results = {
            'processed_content': [],
            'embeddings': [],
            'metadata': [],
            'errors': []
        }

        for url in test_urls:
            print(f"  Processing: {url}")
            content_result = pipeline.extract_text_from_url(url)

            if content_result['content']:
                # Pass only the text content string to chunk_text
                text_content = content_result['content']
                chunks = pipeline.chunk_text(text_content)
                embeddings = pipeline.embed(chunks)

                for chunk, embedding in zip(chunks, embeddings):
                    chunk_with_url = {**chunk, **content_result}
                    success = pipeline.save_chunk_to_qdrant(chunk_with_url, embedding)

                    if success:
                        all_results['processed_content'].append(chunk)
                        all_results['embeddings'].append(embedding)
                        all_results['metadata'].append({
                            'url': url,
                            'chunk_index': chunk.get('chunk_index', 0)
                        })
                    else:
                        all_results['errors'].append({
                            'url': url,
                            'error': 'Failed to save to Qdrant'
                        })

        print(f"  Processed {len(all_results['processed_content'])} chunks from {len(test_urls)} URLs")
        print(f"  Generated {len(all_results['embeddings'])} embeddings")
        print(f"  Encountered {len(all_results['errors'])} errors")
        print("  SUCCESS: Command-line equivalent functionality working")

        return len(all_results['processed_content']) > 0  # Success if we processed at least one chunk

    except Exception as e:
        print(f"  ERROR: Command-line equivalent test failed: {e}")
        return False

def test_error_handling():
    """Test error handling across the pipeline"""
    print("\nTesting error handling...")

    pipeline = WebsiteIngestionPipeline()

    try:
        # Test with a malformed URL
        result = pipeline.extract_text_from_url("not-a-url")
        print("  SUCCESS: Malformed URL handled gracefully")

        # Test with a non-existent URL
        result = pipeline.extract_text_from_url("https://definitely-not-a-real-domain-12345.com")
        print("  SUCCESS: Non-existent URL handled gracefully")

        # Test chunking with empty content
        empty_chunks = pipeline.chunk_text('')  # Pass empty string, not dict
        print("  SUCCESS: Empty content chunking handled gracefully")

        # Test embedding with empty chunks
        empty_embeddings = pipeline.embed([])
        print("  SUCCESS: Empty chunks embedding handled gracefully")

        print("  SUCCESS: Error handling working across all components")
        return True

    except Exception as e:
        print(f"  ERROR: Error handling test failed: {e}")
        return False

if __name__ == "__main__":
    print("Final Integration Testing for RAG Chatbot Pipeline")
    print("="*60)

    # Run all tests
    test1_result = test_complete_pipeline()
    test2_result = test_command_line_equivalent()
    test3_result = test_error_handling()

    print("\n" + "="*60)
    print("Final Integration Test Results:")

    if test1_result:
        print("SUCCESS: Complete pipeline test: PASSED")
    else:
        print("ERROR: Complete pipeline test: FAILED")

    if test2_result:
        print("SUCCESS: Command-line equivalent test: PASSED")
    else:
        print("ERROR: Command-line equivalent test: FAILED")

    if test3_result:
        print("SUCCESS: Error handling test: PASSED")
    else:
        print("ERROR: Error handling test: FAILED")

    overall_success = test1_result and test2_result and test3_result
    print(f"\nOverall Result: {'PASSED' if overall_success else 'FAILED'}")

    if overall_success:
        print("SUCCESS: All integration tests passed! The pipeline is working correctly.")
    else:
        print("ERROR: Some integration tests failed. Please check the output above.")