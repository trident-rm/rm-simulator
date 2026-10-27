#!/usr/bin/env python3
# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""Run Python tooling tests with optional name filters and per-test wall times."""

import argparse
from pathlib import Path
import time
import unittest


class TimedResult(unittest.TextTestResult):
    """Measure setup, execution and teardown without changing unittest outcomes."""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.timings = []

    def startTest(self, test):
        self.started_at = time.perf_counter()
        super().startTest(test)

    def stopTest(self, test):
        self.timings.append((time.perf_counter() - self.started_at, test.id()))
        super().stopTest(test)


def select_tests(suite, excludes):
    """Keep test IDs unless an explicit excluded substring matches."""
    selected = unittest.TestSuite()
    for test in suite:
        if isinstance(test, unittest.TestSuite):
            selected.addTests(select_tests(test, excludes))
        elif not any(name in test.id() for name in excludes):
            selected.addTest(test)
    return selected


def main(argv=None):
    """Discover tests with the standard loader and fail on an empty selection."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--pattern', default='test_*.py', help='test filename glob')
    parser.add_argument('--name', help='test ID substring')
    parser.add_argument('--exclude', action='append', default=[], help='exclude a test ID substring')
    parser.add_argument('--durations', type=int, default=10, help='number of slowest tests to report')
    args = parser.parse_args(argv)
    loader = unittest.TestLoader()
    if args.name:
        loader.testNamePatterns = [f'*{args.name}*']
    suite = select_tests(loader.discover(str(Path(__file__).parent / 'tests'), pattern=args.pattern),
                         args.exclude)
    if not suite.countTestCases():
        parser.error('no tests selected')
    if args.exclude:
        print('Excluded test IDs containing: ' + ', '.join(args.exclude), flush=True)
    runner = unittest.TextTestRunner(resultclass=TimedResult)
    result = runner.run(suite)
    if args.durations > 0:
        print('\nSlowest tests (including setup and teardown):', file=runner.stream)
        for seconds, name in sorted(result.timings, reverse=True)[:args.durations]:
            print(f'{seconds:7.3f}s  {name}', file=runner.stream)
    return 0 if result.wasSuccessful() else 1


if __name__ == '__main__':
    raise SystemExit(main())
