#!/usr/bin/env python3
"""
Main entry point for the RAG validation system.
Orchestrates all validation components and produces comprehensive reports.
"""
import argparse
import sys
import os
import json
import time
from datetime import datetime
from typing import Dict, Any, List
import logging

# Add the project root to the path to import modules
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../..'))

from backend.rag_validation.config_loader import ConfigLoader
from backend.rag_validation.embedding_validator import EmbeddingValidator
from backend.rag_validation.storage_validator import StorageValidator
from backend.rag_validation.retrieval_validator import RetrievalValidator
from backend.rag_validation.error_validator import ErrorValidator
from backend.rag_validation.performance_validator import PerformanceValidator
from backend.rag_validation.validation_report import ValidationReportGenerator, ValidationResult
from backend.rag_validation.logger import validation_logger, setup_logger
from backend.rag_validation.test_data import TestDataGenerator


class RAGValidationOrchestrator:
    """
    Main orchestrator for the RAG validation system.
    Coordinates all validation modules and produces comprehensive reports.
    """

    def __init__(self, config_loader: ConfigLoader = None):
        """
        Initialize the validation orchestrator.

        Args:
            config_loader: Configuration loader instance
        """
        self.config_loader = config_loader or ConfigLoader()
        self.config = self.config_loader.get_config()

        # Initialize all validation modules
        self.embedding_validator = EmbeddingValidator(self.config_loader)
        self.storage_validator = StorageValidator(self.config_loader)
        self.retrieval_validator = RetrievalValidator(self.config_loader)
        self.error_validator = ErrorValidator(self.config_loader)
        self.performance_validator = PerformanceValidator(self.config_loader)

        # Initialize report generator
        self.report_generator = ValidationReportGenerator()

        validation_logger.info("RAGValidationOrchestrator initialized successfully")

    def run_embedding_validation(self) -> Dict[str, Any]:
        """
        Run the embedding validation component.

        Returns:
            Dictionary with embedding validation results
        """
        validation_logger.info("Starting embedding validation...")

        start_time = time.time()
        try:
            # Run complete embedding validation
            result = self.embedding_validator.run_complete_validation()
            duration = time.time() - start_time

            # Add to report
            success = result.get("overall_success", {}).get("all_valid", False)
            self.report_generator.add_test_result(
                name="Embedding Validation",
                description="Validate Cohere embedding generation pipeline",
                result=ValidationResult.PASS if success else ValidationResult.FAIL,
                details=result,
                duration_ms=duration * 1000,
                component="embedding"
            )

            validation_logger.info(f"Embedding validation completed in {duration:.2f}s")
            return result

        except Exception as e:
            duration = time.time() - start_time
            validation_logger.error(f"Embedding validation failed: {str(e)}")

            # Add failure to report
            self.report_generator.add_test_result(
                name="Embedding Validation",
                description="Validate Cohere embedding generation pipeline",
                result=ValidationResult.FAIL,
                details={"error": str(e)},
                duration_ms=duration * 1000,
                component="embedding"
            )

            return {"success": False, "error": str(e)}

    def run_storage_validation(self) -> Dict[str, Any]:
        """
        Run the storage validation component.

        Returns:
            Dictionary with storage validation results
        """
        validation_logger.info("Starting storage validation...")

        start_time = time.time()
        try:
            # Run complete storage validation
            result = self.storage_validator.run_complete_storage_validation()
            duration = time.time() - start_time

            # Add to report
            success = result.get("overall_success", {}).get("all_valid", False)
            self.report_generator.add_test_result(
                name="Storage Validation",
                description="Validate Qdrant storage and metadata validation",
                result=ValidationResult.PASS if success else ValidationResult.FAIL,
                details=result,
                duration_ms=duration * 1000,
                component="storage"
            )

            validation_logger.info(f"Storage validation completed in {duration:.2f}s")
            return result

        except Exception as e:
            duration = time.time() - start_time
            validation_logger.error(f"Storage validation failed: {str(e)}")

            # Add failure to report
            self.report_generator.add_test_result(
                name="Storage Validation",
                description="Validate Qdrant storage and metadata validation",
                result=ValidationResult.FAIL,
                details={"error": str(e)},
                duration_ms=duration * 1000,
                component="storage"
            )

            return {"success": False, "error": str(e)}

    def run_retrieval_validation(self) -> Dict[str, Any]:
        """
        Run the retrieval validation component.

        Returns:
            Dictionary with retrieval validation results
        """
        validation_logger.info("Starting retrieval validation...")

        start_time = time.time()
        try:
            # Run complete retrieval validation
            result = self.retrieval_validator.run_complete_retrieval_validation()
            duration = time.time() - start_time

            # Add to report
            success = result.get("overall_success", {}).get("all_queries_processed", False)
            self.report_generator.add_test_result(
                name="Retrieval Validation",
                description="Validate similarity-based retrieval from Qdrant",
                result=ValidationResult.PASS if success else ValidationResult.FAIL,
                details=result,
                duration_ms=duration * 1000,
                component="retrieval"
            )

            validation_logger.info(f"Retrieval validation completed in {duration:.2f}s")
            return result

        except Exception as e:
            duration = time.time() - start_time
            validation_logger.error(f"Retrieval validation failed: {str(e)}")

            # Add failure to report
            self.report_generator.add_test_result(
                name="Retrieval Validation",
                description="Validate similarity-based retrieval from Qdrant",
                result=ValidationResult.FAIL,
                details={"error": str(e)},
                duration_ms=duration * 1000,
                component="retrieval"
            )

            return {"success": False, "error": str(e)}

    def run_error_validation(self) -> Dict[str, Any]:
        """
        Run the error handling validation component.

        Returns:
            Dictionary with error validation results
        """
        validation_logger.info("Starting error handling validation...")

        start_time = time.time()
        try:
            # Run complete error validation
            result = self.error_validator.run_complete_error_validation()
            duration = time.time() - start_time

            # Add to report
            success = result.get("overall_success", {}).get("error_classification_working", False)
            self.report_generator.add_test_result(
                name="Error Handling Validation",
                description="Validate error detection and handling mechanisms",
                result=ValidationResult.PASS if success else ValidationResult.FAIL,
                details=result,
                duration_ms=duration * 1000,
                component="error_handling"
            )

            validation_logger.info(f"Error handling validation completed in {duration:.2f}s")
            return result

        except Exception as e:
            duration = time.time() - start_time
            validation_logger.error(f"Error handling validation failed: {str(e)}")

            # Add failure to report
            self.report_generator.add_test_result(
                name="Error Handling Validation",
                description="Validate error detection and handling mechanisms",
                result=ValidationResult.FAIL,
                details={"error": str(e)},
                duration_ms=duration * 1000,
                component="error_handling"
            )

            return {"success": False, "error": str(e)}

    def run_performance_validation(self) -> Dict[str, Any]:
        """
        Run the performance validation component.

        Returns:
            Dictionary with performance validation results
        """
        validation_logger.info("Starting performance validation...")

        start_time = time.time()
        try:
            # Run complete performance validation
            result = self.performance_validator.run_complete_performance_validation()
            duration = time.time() - start_time

            # Add to report
            success = result.get("overall_success", {}).get("all_tests_passed", False)
            self.report_generator.add_test_result(
                name="Performance Validation",
                description="Validate retrieval latency and performance metrics",
                result=ValidationResult.PASS if success else ValidationResult.FAIL,
                details=result,
                duration_ms=duration * 1000,
                component="performance"
            )

            validation_logger.info(f"Performance validation completed in {duration:.2f}s")
            return result

        except Exception as e:
            duration = time.time() - start_time
            validation_logger.error(f"Performance validation failed: {str(e)}")

            # Add failure to report
            self.report_generator.add_test_result(
                name="Performance Validation",
                description="Validate retrieval latency and performance metrics",
                result=ValidationResult.FAIL,
                details={"error": str(e)},
                duration_ms=duration * 1000,
                component="performance"
            )

            return {"success": False, "error": str(e)}

    def run_complete_validation(self, components: List[str] = None) -> Dict[str, Any]:
        """
        Run complete validation across all components.

        Args:
            components: List of components to validate (default: all components)
                       Options: ['embedding', 'storage', 'retrieval', 'error', 'performance']

        Returns:
            Dictionary with complete validation results
        """
        if components is None:
            components = ['embedding', 'storage', 'retrieval', 'error', 'performance']

        validation_logger.info(f"Starting complete validation for components: {components}")

        start_time = time.time()
        results = {}

        # Run each requested validation component
        if 'embedding' in components:
            results['embedding'] = self.run_embedding_validation()

        if 'storage' in components:
            results['storage'] = self.run_storage_validation()

        if 'retrieval' in components:
            results['retrieval'] = self.run_retrieval_validation()

        if 'error' in components:
            results['error'] = self.run_error_validation()

        if 'performance' in components:
            results['performance'] = self.run_performance_validation()

        total_duration = time.time() - start_time

        # Generate comprehensive report
        json_report_path = self.report_generator.generate_json_report()
        csv_report_path = self.report_generator.generate_csv_report()
        summary_report_path = self.report_generator.generate_summary_report()

        # Print summary to console
        self.report_generator.print_summary()

        final_result = {
            "success": True,
            "components_validated": components,
            "results": results,
            "total_duration_seconds": total_duration,
            "report_paths": {
                "json": json_report_path,
                "csv": csv_report_path,
                "summary": summary_report_path
            }
        }

        validation_logger.info(f"Complete validation finished in {total_duration:.2f}s")
        return final_result

    def run_validation_pipeline(self, options: Dict[str, Any]) -> Dict[str, Any]:
        """
        Run the validation pipeline based on provided options.

        Args:
            options: Dictionary with validation options

        Returns:
            Dictionary with pipeline results
        """
        components = options.get('components', ['embedding', 'storage', 'retrieval', 'error', 'performance'])
        return self.run_complete_validation(components)


def parse_command_line_args():
    """
    Parse command line arguments for the validation system.

    Returns:
        Parsed arguments object
    """
    parser = argparse.ArgumentParser(
        description="RAG Validation System - Validates Cohere embeddings, Qdrant storage, retrieval, error handling, and performance"
    )

    parser.add_argument(
        '--components',
        nargs='+',
        choices=['embedding', 'storage', 'retrieval', 'error', 'performance', 'all'],
        default=['all'],
        help='Validation components to run (default: all)'
    )

    parser.add_argument(
        '--config-file',
        type=str,
        default=None,
        help='Path to configuration file'
    )

    parser.add_argument(
        '--output-dir',
        type=str,
        default='validation_reports',
        help='Directory to save validation reports (default: validation_reports)'
    )

    parser.add_argument(
        '--verbose',
        action='store_true',
        help='Enable verbose logging'
    )

    parser.add_argument(
        '--test-data-file',
        type=str,
        default=None,
        help='Path to file with test data for validation'
    )

    return parser.parse_args()


def main():
    """
    Main entry point for the RAG validation system.
    """
    # Parse command line arguments
    args = parse_command_line_args()

    # Set up logging based on verbose flag
    log_level = logging.DEBUG if args.verbose else logging.INFO
    setup_logger('rag_validation', level=log_level)

    # Determine which components to validate
    if 'all' in args.components:
        components = ['embedding', 'storage', 'retrieval', 'error', 'performance']
    else:
        components = args.components

    # Create config loader
    config_loader = ConfigLoader()

    # Create orchestrator
    orchestrator = RAGValidationOrchestrator(config_loader)

    # Update report generator output directory if specified
    if args.output_dir:
        orchestrator.report_generator.output_dir = args.output_dir

    # Run validation
    print(f"Starting RAG validation for components: {components}")
    print("=" * 60)

    results = orchestrator.run_complete_validation(components)

    # Print final summary
    print("=" * 60)
    print("VALIDATION COMPLETED")
    print(f"Total Duration: {results['total_duration_seconds']:.2f} seconds")
    print(f"JSON Report: {results['report_paths']['json']}")
    print(f"CSV Report: {results['report_paths']['csv']}")
    print(f"Summary Report: {results['report_paths']['summary']}")

    return results


if __name__ == "__main__":
    # Load environment variables
    from dotenv import load_dotenv
    load_dotenv()

    # Run the main function
    main()