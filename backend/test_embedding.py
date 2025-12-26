#!/usr/bin/env python3
"""
Test script for Phase 5 functionality: Embedding generation and storage
"""

import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from backend.ingestion import WebsiteIngestionPipeline

def test_embedding_generation():
    """Test embedding generation with sample text chunks"""
    print("Testing embedding generation with sample text chunks...")

    pipeline = WebsiteIngestionPipeline()

    # Sample text chunks for testing
    sample_chunks = [
        {
            'text': "This is the first sample text chunk for embedding testing.",
            'url': 'https://example.com/page1',
            'title': 'Sample Page 1',
            'chunk_index': 0,
            'start_pos': 0,
            'end_pos': 59
        },
        {
            'text': "This is the second sample text chunk for embedding testing.",
            'url': 'https://example.com/page2',
            'title': 'Sample Page 2',
            'chunk_index': 1,
            'start_pos': 60,
            'end_pos': 119
        },
        {
            'text': "This is the third sample text chunk for embedding testing with slightly more content to make it longer.",
            'url': 'https://example.com/page3',
            'title': 'Sample Page 3',
            'chunk_index': 2,
            'start_pos': 120,
            'end_pos': 230
        }
    ]

    print(f"Generating embeddings for {len(sample_chunks)} text chunks...")

    try:
        embeddings = pipeline.embed(sample_chunks)
        print(f"Successfully generated {len(embeddings)} embeddings")

        # Check that embeddings have the expected dimensions (768 for Cohere multilingual model)
        if len(embeddings) > 0:
            first_embedding = embeddings[0]
            print(f"First embedding has {len(first_embedding)} dimensions")

            # Validate vector dimensions
            expected_dims = 768
            if len(first_embedding) == expected_dims:
                print(f"✓ Vector dimensions are correct: {len(first_embedding)} (expected {expected_dims})")
            else:
                print(f"✗ Vector dimensions are incorrect: {len(first_embedding)} (expected {expected_dims})")

        return embeddings, sample_chunks

    except Exception as e:
        print(f"Error generating embeddings: {e}")
        return [], sample_chunks

def test_collection_creation():
    """Test Qdrant collection creation"""
    print("\nTesting Qdrant collection creation...")

    pipeline = WebsiteIngestionPipeline()

    try:
        success = pipeline.create_collection()
        if success:
            print(f"✓ Collection '{pipeline.collection_name}' created or already exists")
        else:
            print(f"✗ Failed to create collection '{pipeline.collection_name}'")
        return success
    except Exception as e:
        print(f"Error creating collection: {e}")
        return False

def test_save_to_qdrant():
    """Test saving a chunk with embedding to Qdrant"""
    print("\nTesting saving to Qdrant...")

    pipeline = WebsiteIngestionPipeline()

    # First create the collection
    collection_success = pipeline.create_collection()
    if not collection_success:
        print("✗ Cannot test saving to Qdrant - collection creation failed")
        return False

    # Create a sample chunk and embedding
    sample_chunk = {
        'text': "This is a sample text chunk for testing Qdrant storage.",
        'url': 'https://example.com/test',
        'title': 'Test Page',
        'chunk_index': 0,
        'start_pos': 0,
        'end_pos': 54
    }

    # Generate a sample embedding (in practice, this would come from the embed function)
    # For testing purposes, we'll create a fake 768-dimensional embedding
    import random
    sample_embedding = [random.random() for _ in range(768)]

    try:
        success = pipeline.save_chunk_to_qdrant(sample_chunk, sample_embedding)
        if success:
            print("✓ Successfully saved chunk to Qdrant")
        else:
            print("✗ Failed to save chunk to Qdrant")
        return success
    except Exception as e:
        print(f"Error saving to Qdrant: {e}")
        return False

def test_full_integration():
    """Test the full integration of embedding and storage"""
    print("\nTesting full integration (embed + save to Qdrant)...")

    pipeline = WebsiteIngestionPipeline()

    # Create collection
    collection_success = pipeline.create_collection()
    if not collection_success:
        print("✗ Cannot test full integration - collection creation failed")
        return False

    # Sample text chunks
    sample_chunks = [
        {
            'text': "This is the first text chunk for full integration testing.",
            'url': 'https://example.com/full-test1',
            'title': 'Full Integration Test 1',
            'chunk_index': 0,
            'start_pos': 0,
            'end_pos': 58
        },
        {
            'text': "This is the second text chunk for full integration testing.",
            'url': 'https://example.com/full-test2',
            'title': 'Full Integration Test 2',
            'chunk_index': 1,
            'start_pos': 59,
            'end_pos': 118
        }
    ]

    try:
        # Generate embeddings
        embeddings = pipeline.embed(sample_chunks)
        print(f"✓ Generated {len(embeddings)} embeddings")

        # Save each chunk with its embedding to Qdrant
        saved_count = 0
        for i, (chunk, embedding) in enumerate(zip(sample_chunks, embeddings)):
            success = pipeline.save_chunk_to_qdrant(chunk, embedding)
            if success:
                saved_count += 1
                print(f"  ✓ Saved chunk {i+1} to Qdrant")
            else:
                print(f"  ✗ Failed to save chunk {i+1} to Qdrant")

        print(f"✓ Successfully saved {saved_count}/{len(sample_chunks)} chunks to Qdrant")
        return saved_count == len(sample_chunks)

    except Exception as e:
        print(f"Error in full integration test: {e}")
        return False

if __name__ == "__main__":
    print("Testing Phase 5: Embedding Generation and Storage")
    print("="*60)

    # Run all tests
    embeddings, chunks = test_embedding_generation()
    collection_test = test_collection_creation()
    save_test = test_save_to_qdrant()
    integration_test = test_full_integration()

    print("\n" + "="*60)
    print("Phase 5 testing completed.")

    if embeddings and collection_test and save_test and integration_test:
        print("✓ All Phase 5 tests passed - embedding generation and storage working properly")
    else:
        print("✗ Some Phase 5 tests failed - check output above for details")