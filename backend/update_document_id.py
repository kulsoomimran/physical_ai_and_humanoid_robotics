#!/usr/bin/env python3
"""
Script to update existing Qdrant records with document_id field without re-ingesting data.

This script connects to Qdrant and updates existing records to include a document_id field
in the payload, where document_id is derived from the source_url to maintain consistency.
"""

import os
import logging
import hashlib
import uuid
from typing import List
from qdrant_client import QdrantClient
from qdrant_client.http import models
from qdrant_client.http.models import PointStruct
from dotenv import load_dotenv

# Load environment variables from .env file
load_dotenv()

# Configure logging
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger(__name__)


class QdrantDocumentIdUpdater:
    """
    A class to update existing Qdrant records with a document_id field.
    """

    def __init__(self):
        """
        Initialize the updater with Qdrant client.
        """
        # Qdrant configuration
        self.qdrant_url = os.getenv("QDRANT_URL")
        self.qdrant_api_key = os.getenv("QDRANT_API_KEY")
        self.collection_name = os.getenv("COLLECTION_NAME", "book_content")

        if not self.qdrant_url:
            raise ValueError("QDRANT_URL environment variable is required")

        # Initialize Qdrant client
        self.client = QdrantClient(
            url=self.qdrant_url,
            api_key=self.qdrant_api_key
        )

        logger.info(f"Qdrant client initialized for collection: {self.collection_name}")

    def generate_document_id(self, source_url: str) -> str:
        """
        Generate a deterministic document_id based on the source_url.
        This ensures consistency if the same URL is processed multiple times.
        """
        if not source_url:
            # Generate a random UUID if no source_url is provided
            return str(uuid.uuid4())

        # Create a deterministic ID based on the source URL
        # Using SHA256 hash of the URL to create a consistent document ID
        hash_object = hashlib.sha256(source_url.encode('utf-8'))
        hex_dig = hash_object.hexdigest()
        # Take first 32 characters and format as UUID
        return str(uuid.UUID(hex_dig[:32]))

    def get_all_points(self, batch_size: int = 1000) -> List[models.PointStruct]:
        """
        Retrieve all points from the collection in batches.
        """
        all_points = []
        offset = None
        total_points = 0

        logger.info(f"Starting to retrieve all points from collection '{self.collection_name}'...")

        while True:
            # Retrieve points in batches
            points, next_offset = self.client.scroll(
                collection_name=self.collection_name,
                limit=batch_size,
                offset=offset,
                with_payload=True,
                with_vectors=True
            )

            if not points:
                break

            all_points.extend(points)
            total_points += len(points)

            logger.info(f"Retrieved {total_points} points so far...")

            if next_offset is None:
                break
            offset = next_offset

        logger.info(f"Total points retrieved: {total_points}")
        return all_points

    def update_points_with_document_id(self, points: List[models.PointStruct]) -> int:
        """
        Update points to include document_id in their payload if not already present.
        """
        updated_count = 0
        points_to_update = []

        for point in points:
            payload = point.payload or {}

            # Check if document_id already exists
            if 'document_id' not in payload or not payload.get('document_id', '').strip():
                # Generate document_id based on source_url
                source_url = payload.get('source_url', '')
                document_id = self.generate_document_id(source_url)

                # Update the payload with document_id
                updated_payload = payload.copy()
                updated_payload['document_id'] = document_id

                # Create a new point with updated payload
                if hasattr(point, 'vectors') and point.vectors is not None:
                    # Named vectors case
                    updated_point = models.PointStruct(
                        id=point.id,
                        vectors=point.vectors,
                        payload=updated_payload
                    )
                else:
                    # Single vector case
                    updated_point = models.PointStruct(
                        id=point.id,
                        vector=point.vector,
                        payload=updated_payload
                    )

                points_to_update.append(updated_point)
                updated_count += 1

                if updated_count % 100 == 0:
                    logger.info(f"Prepared {updated_count} points for update...")

        # Update all points in batches
        if points_to_update:
            logger.info(f"Updating {len(points_to_update)} points with document_id...")

            # Process in batches of 100 to avoid overwhelming the server
            batch_size = 100
            for i in range(0, len(points_to_update), batch_size):
                batch = points_to_update[i:i + batch_size]

                try:
                    self.client.upsert(
                        collection_name=self.collection_name,
                        points=batch
                    )

                    processed = min(i + batch_size, len(points_to_update))
                    logger.info(f"Updated {processed}/{len(points_to_update)} points...")

                except Exception as e:
                    logger.error(f"Error updating batch starting at index {i}: {e}")
                    raise

        return updated_count

    def run_update(self):
        """
        Run the complete update process.
        """
        logger.info("Starting document_id update process...")

        # Get all points from the collection
        all_points = self.get_all_points()

        if not all_points:
            logger.info("No points found in the collection.")
            return 0

        # Update points with document_id
        updated_count = self.update_points_with_document_id(all_points)

        logger.info(f"Document ID update process completed. Updated {updated_count} points.")
        return updated_count


def main():
    """
    Main function to run the document_id update process.
    """
    try:
        updater = QdrantDocumentIdUpdater()
        updated_count = updater.run_update()

        print(f"\nUpdate completed successfully! Updated {updated_count} records with document_id.")
        print("You can now use the retrieve.py script without the 'Document ID must exist and be non-empty' error.")

    except Exception as e:
        logger.error(f"Error during document_id update: {e}")
        print(f"Error: {e}")
        return 1

    return 0


if __name__ == "__main__":
    exit(main())