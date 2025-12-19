"""
Validation report generator for RAG validation system.
Handles creation and formatting of validation reports in various formats.
"""
import json
import csv
from datetime import datetime
from typing import Dict, List, Any, Optional
import os
from dataclasses import dataclass, asdict
from enum import Enum


class ValidationResult(Enum):
    """
    Enum for validation result states.
    """
    PASS = "PASS"
    FAIL = "FAIL"
    WARNING = "WARNING"
    SKIPPED = "SKIPPED"


@dataclass
class ValidationTest:
    """
    Data class representing a single validation test.
    """
    name: str
    description: str
    result: ValidationResult
    details: Dict[str, Any]
    timestamp: str
    duration_ms: float
    component: str


class ValidationReportGenerator:
    """
    Generator for validation reports in various formats (JSON, CSV, etc.).
    """

    def __init__(self, output_dir: str = "validation_reports"):
        """
        Initialize the validation report generator.

        Args:
            output_dir: Directory where reports will be saved
        """
        self.output_dir = output_dir
        os.makedirs(output_dir, exist_ok=True)
        self.tests: List[ValidationTest] = []

    def add_test_result(
        self,
        name: str,
        description: str,
        result: ValidationResult,
        details: Dict[str, Any],
        duration_ms: float,
        component: str
    ) -> None:
        """
        Add a test result to the report.

        Args:
            name: Name of the test
            description: Description of what the test validates
            result: Result of the validation (PASS, FAIL, WARNING, SKIPPED)
            details: Additional details about the test result
            duration_ms: Duration of the test in milliseconds
            component: Component being tested
        """
        test = ValidationTest(
            name=name,
            description=description,
            result=result,
            details=details,
            timestamp=datetime.utcnow().isoformat(),
            duration_ms=duration_ms,
            component=component
        )
        self.tests.append(test)

    def generate_json_report(self, filename: str = None) -> str:
        """
        Generate a validation report in JSON format.

        Args:
            filename: Optional filename for the report (default: validation_report_<timestamp>.json)

        Returns:
            Path to the generated report file
        """
        if filename is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"validation_report_{timestamp}.json"

        report_path = os.path.join(self.output_dir, filename)

        report_data = {
            "timestamp": datetime.utcnow().isoformat(),
            "total_tests": len(self.tests),
            "passed_tests": len([t for t in self.tests if t.result == ValidationResult.PASS]),
            "failed_tests": len([t for t in self.tests if t.result == ValidationResult.FAIL]),
            "warning_tests": len([t for t in self.tests if t.result == ValidationResult.WARNING]),
            "skipped_tests": len([t for t in self.tests if t.result == ValidationResult.SKIPPED]),
            "test_results": [asdict(test) for test in self.tests]
        }

        with open(report_path, 'w', encoding='utf-8') as f:
            json.dump(report_data, f, indent=2, ensure_ascii=False)

        return report_path

    def generate_csv_report(self, filename: str = None) -> str:
        """
        Generate a validation report in CSV format.

        Args:
            filename: Optional filename for the report (default: validation_report_<timestamp>.csv)

        Returns:
            Path to the generated report file
        """
        if filename is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"validation_report_{timestamp}.csv"

        report_path = os.path.join(self.output_dir, filename)

        with open(report_path, 'w', newline='', encoding='utf-8') as f:
            fieldnames = [
                'name', 'description', 'result', 'timestamp',
                'duration_ms', 'component', 'details'
            ]
            writer = csv.DictWriter(f, fieldnames=fieldnames)

            writer.writeheader()
            for test in self.tests:
                row = {
                    'name': test.name,
                    'description': test.description,
                    'result': test.result.value,
                    'timestamp': test.timestamp,
                    'duration_ms': test.duration_ms,
                    'component': test.component,
                    'details': json.dumps(test.details)
                }
                writer.writerow(row)

        return report_path

    def generate_summary_report(self, filename: str = None) -> str:
        """
        Generate a summary validation report with key metrics.

        Args:
            filename: Optional filename for the report (default: summary_report_<timestamp>.json)

        Returns:
            Path to the generated report file
        """
        if filename is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"summary_report_{timestamp}.json"

        report_path = os.path.join(self.output_dir, filename)

        # Calculate component-wise statistics
        component_stats = {}
        for test in self.tests:
            if test.component not in component_stats:
                component_stats[test.component] = {
                    'total': 0,
                    'pass': 0,
                    'fail': 0,
                    'warning': 0,
                    'skipped': 0,
                    'total_duration_ms': 0
                }

            stats = component_stats[test.component]
            stats['total'] += 1
            stats[test.result.value.lower()] += 1
            stats['total_duration_ms'] += test.duration_ms

        # Calculate overall statistics
        total_tests = len(self.tests)
        passed_tests = len([t for t in self.tests if t.result == ValidationResult.PASS])
        success_rate = (passed_tests / total_tests * 100) if total_tests > 0 else 0

        summary_data = {
            "timestamp": datetime.utcnow().isoformat(),
            "summary": {
                "total_tests": total_tests,
                "passed": passed_tests,
                "failed": len([t for t in self.tests if t.result == ValidationResult.FAIL]),
                "warnings": len([t for t in self.tests if t.result == ValidationResult.WARNING]),
                "skipped": len([t for t in self.tests if t.result == ValidationResult.SKIPPED]),
                "success_rate_percent": round(success_rate, 2)
            },
            "component_stats": component_stats,
            "total_duration_ms": sum(t.duration_ms for t in self.tests),
            "validation_passed": success_rate >= 95  # Assuming 95% as threshold for overall validation
        }

        with open(report_path, 'w', encoding='utf-8') as f:
            json.dump(summary_data, f, indent=2, ensure_ascii=False)

        return report_path

    def print_summary(self) -> None:
        """
        Print a summary of validation results to the console.
        """
        total_tests = len(self.tests)
        if total_tests == 0:
            print("No validation tests have been run yet.")
            return

        passed_tests = len([t for t in self.tests if t.result == ValidationResult.PASS])
        failed_tests = len([t for t in self.tests if t.result == ValidationResult.FAIL])
        warning_tests = len([t for t in self.tests if t.result == ValidationResult.WARNING])
        skipped_tests = len([t for t in self.tests if t.result == ValidationResult.SKIPPED])

        success_rate = (passed_tests / total_tests * 100) if total_tests > 0 else 0

        print(f"\\n{'='*60}")
        print("RAG VALIDATION SUMMARY")
        print(f"{'='*60}")
        print(f"Total Tests: {total_tests}")
        print(f"Passed: {passed_tests} ({success_rate:.1f}%)")
        print(f"Failed: {failed_tests}")
        print(f"Warnings: {warning_tests}")
        print(f"Skipped: {skipped_tests}")
        print(f"{'='*60}")

        if failed_tests > 0:
            print("\\nFailed Tests:")
            for test in self.tests:
                if test.result == ValidationResult.FAIL:
                    print(f"  - {test.name}: {test.description}")

        if warning_tests > 0:
            print("\\nWarning Tests:")
            for test in self.tests:
                if test.result == ValidationResult.WARNING:
                    print(f"  - {test.name}: {test.description}")

        print(f"\\nOverall Validation: {'PASSED' if success_rate >= 95 else 'FAILED'}")
        print(f"{'='*60}")


def create_validation_report_generator(output_dir: str = "validation_reports") -> ValidationReportGenerator:
    """
    Create and return a validation report generator instance.

    Args:
        output_dir: Directory where reports will be saved

    Returns:
        ValidationReportGenerator instance
    """
    return ValidationReportGenerator(output_dir)