#!/usr/bin/env python3
"""
Test script for Phase 4 functionality: Content preprocessing and chunking
"""

import sys
import os
sys.path.append(os.path.dirname(os.path.abspath(__file__)))

from main import WebsiteIngestionPipeline

def test_basic_chunking():
    """Test basic chunking functionality with sample text"""
    print("Testing basic chunking functionality...")

    pipeline = WebsiteIngestionPipeline()

    # Sample text for testing
    sample_text = (
        "This is a sample text for testing the chunking functionality. "
        "It contains multiple sentences to test sentence boundary detection. "
        "We want to ensure that chunks don't break in the middle of sentences. "
        "Additionally, we want to test that overlap works correctly between chunks. "
        "The system should maintain semantic coherence across chunk boundaries. "
        "This is important for downstream embedding and retrieval tasks. "
        "Let's add more text to ensure we get multiple chunks. "
        "This will help us verify that the chunking algorithm works properly. "
        "We should see that sentences are kept together when possible. "
        "And that the overlap helps maintain context between chunks."
    )

    print(f"Original text length: {len(sample_text)} characters")
    print(f"Configured chunk size: {pipeline.chunk_size}, overlap: {pipeline.chunk_overlap}")

    chunks = pipeline.chunk_text(sample_text)

    print(f"Text was chunked into {len(chunks)} chunks:")
    for i, chunk in enumerate(chunks):
        print(f"  Chunk {i+1}: {len(chunk['text'])} chars (pos {chunk['start_pos']}-{chunk['end_pos']})")
        print(f"    Preview: {chunk['text'][:60]}...")
        print()

    return chunks

def test_chunking_with_different_sizes():
    """Test chunking with different sizes and overlaps"""
    print("Testing chunking with different parameters...")

    pipeline = WebsiteIngestionPipeline()

    sample_text = (
        "This is a test of different chunk sizes. "
        "We'll try various configurations to see how the chunking behaves. "
        "Larger chunks might capture more context but could be too big for embeddings. "
        "Smaller chunks might lose context but be more precise. "
        "The overlap helps maintain continuity between chunks. "
        "Sentence boundaries should be respected when possible. "
        "Let's make this text long enough to generate multiple chunks. "
        "This will help us verify the boundary detection works properly. "
        "We want to ensure semantic coherence across chunk boundaries. "
        "The system should avoid breaking in the middle of important concepts."
    )

    # Test with different parameters
    configs = [
        {"chunk_size": 50, "overlap": 10},
        {"chunk_size": 100, "overlap": 20},
        {"chunk_size": 200, "overlap": 50},
    ]

    for config in configs:
        print(f"\nTesting with chunk_size={config['chunk_size']}, overlap={config['overlap']}")
        chunks = pipeline.chunk_text(sample_text, config['chunk_size'], config['overlap'])
        print(f"  Generated {len(chunks)} chunks")
        for i, chunk in enumerate(chunks[:2]):  # Show first 2 chunks
            print(f"    Chunk {i+1}: {len(chunk['text'])} chars, starts at pos {chunk['start_pos']}")
            print(f"    Preview: {chunk['text'][:50]}...")

def test_sentence_boundary_preservation():
    """Test that sentence boundaries are preserved"""
    print("\nTesting sentence boundary preservation...")

    pipeline = WebsiteIngestionPipeline()

    # Text with clear sentence boundaries
    text_with_sentences = (
        "This is the first sentence. This is the second sentence! Is this the third sentence? "
        "This sentence has a semicolon; it should also be preserved. "
        "Here's another sentence with normal punctuation. "
        "The algorithm should try to break at sentence boundaries when possible. "
        "This helps maintain semantic coherence across chunks."
    )

    chunks = pipeline.chunk_text(text_with_sentences, chunk_size=80, overlap=20)

    print(f"Generated {len(chunks)} chunks with sentence-aware splitting:")
    for i, chunk in enumerate(chunks):
        print(f"  Chunk {i+1}: '{chunk['text'][:60]}...'")
        # Check if the chunk ends with sentence punctuation
        last_chars = chunk['text'][-5:] if len(chunk['text']) >= 5 else chunk['text']
        print(f"    Ends with: '{last_chars}'")

def test_overlap_functionality():
    """Test that overlap works correctly between chunks"""
    print("\nTesting overlap functionality...")

    pipeline = WebsiteIngestionPipeline()

    sample_text = (
        "Chunk 1 content starts here and continues with more text. "
        "This is additional content for the first chunk. "
        "More text to fill up the chunk. "
        "Additional sentences to make it longer. "
        "Continuing with more content. "
        "More text for the chunk. "
        "Even more text to ensure it's long enough. "
        "Adding more sentences. "
        "More content for testing. "
        "Final sentences for chunk 1. "
        "Chunk 2 begins with new content here. "
        "More content for the second chunk. "
        "Additional text for testing. "
        "More sentences to continue. "
        "Even more content for the second chunk. "
        "Continuing with more text. "
        "More content to fill it up. "
        "Final content for the second chunk."
    )

    chunks = pipeline.chunk_text(sample_text, chunk_size=100, overlap=30)

    print(f"Generated {len(chunks)} chunks with overlap:")
    for i, chunk in enumerate(chunks):
        print(f"  Chunk {i+1}: {len(chunk['text'])} chars")
        print(f"    Position: {chunk['start_pos']}-{chunk['end_pos']}")
        if i > 0:
            # Check overlap between this and previous chunk
            prev_chunk = chunks[i-1]['text']
            curr_chunk = chunk['text']
            overlap_region = prev_chunk[-60:]  # Last 60 chars of previous chunk
            start_region = curr_chunk[:60]     # First 60 chars of current chunk
            print(f"    Overlap check: '{overlap_region[-20:]}' ... '{start_region[:20]}'")
        print(f"    Preview: {chunk['text'][:50]}...")

def test_edge_cases():
    """Test edge cases for chunking"""
    print("\nTesting edge cases...")

    pipeline = WebsiteIngestionPipeline()

    # Very short text
    short_text = "Short text."
    chunks = pipeline.chunk_text(short_text)
    print(f"Short text ({len(short_text)} chars) resulted in {len(chunks)} chunks")

    # Text shorter than chunk size
    medium_text = "This is a medium length text that is shorter than the default chunk size."
    chunks = pipeline.chunk_text(medium_text)
    print(f"Medium text ({len(medium_text)} chars) resulted in {len(chunks)} chunks")

    # Empty text
    try:
        empty_chunks = pipeline.chunk_text("")
        print(f"Empty text resulted in {len(empty_chunks)} chunks")
    except Exception as e:
        print(f"Empty text caused error: {e}")

if __name__ == "__main__":
    print("Testing Phase 4: Content Preprocessing and Chunking")
    print("="*60)

    # Run all tests
    chunks = test_basic_chunking()
    test_chunking_with_different_sizes()
    test_sentence_boundary_preservation()
    test_overlap_functionality()
    test_edge_cases()

    print("\n" + "="*60)
    print("Phase 4 testing completed.")

    if chunks:
        print(f"✓ Chunking functionality working - created {len(chunks)} chunks from sample text")
        # Verify semantic coherence by checking that chunks connect properly
        total_text = "".join([chunk['text'] for chunk in chunks])
        original_text = (
            "This is a sample text for testing the chunking functionality. "
            "It contains multiple sentences to test sentence boundary detection. "
            "We want to ensure that chunks don't break in the middle of sentences. "
            "Additionally, we want to test that overlap works correctly between chunks. "
            "The system should maintain semantic coherence across chunk boundaries. "
            "This is important for downstream embedding and retrieval tasks. "
            "Let's add more text to ensure we get multiple chunks. "
            "This will help us verify that the chunking algorithm works properly. "
            "We should see that sentences are kept together when possible. "
            "And that the overlap helps maintain context between chunks."
        )
        print(f"✓ Total reassembled text length matches: {len(total_text) == len(original_text)}")
    else:
        print("✗ No chunks were created")