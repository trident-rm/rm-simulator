# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""Conservative test selection, subprocess failures and build-profile propagation."""

import contextlib
import importlib.util
import io
import json
from pathlib import Path
import subprocess
import tempfile
import unittest
from unittest.mock import patch


def load(name):
    spec = importlib.util.spec_from_file_location(name, Path(__file__).parents[1] / f'{name}.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


rust = load('test-rust')
python = load('test-python')
runtime = load('check-runtime')


def metadata(root):
    packages = []
    for name, dependencies in [('core', []), ('physics', ['core']), ('app', ['physics']),
                               ('unrelated', [])]:
        packages.append({'id': name, 'name': name,
                         'manifest_path': str(root / 'crates' / name / 'Cargo.toml'),
                         'dependencies': [{'name': dependency, 'path': str(root / 'crates' / dependency),
                                           'kind': 'dev', 'optional': True}
                                          for dependency in dependencies]})
    return {'workspace_root': str(root), 'workspace_members': [p['id'] for p in packages],
            'packages': packages}


class AffectedTests(unittest.TestCase):
    def test_transitive_consumers_include_optional_dev_dependencies(self):
        data = metadata(Path('/repo'))
        self.assertEqual(rust.affected_packages(data, ['crates/core/src/lib.rs']),
                         ['app', 'core', 'physics'])
        self.assertEqual(rust.affected_packages(data, ['crates/physics/tests/regression.rs']),
                         ['app', 'physics'])

    def test_global_and_unknown_changes_select_everything(self):
        for path in ['Cargo.lock', 'Cargo.toml', '.cargo/config.toml', 'scripts/build.py',
                     'crates/new/src/lib.rs', '.github/workflows/ci.yaml']:
            with self.subTest(path=path):
                self.assertIsNone(rust.affected_packages(metadata(Path('/repo')), [path]))
        self.assertEqual(rust.affected_packages(metadata(Path('/repo')), ['docs/development.md']), [])

    def test_git_includes_deletions_staged_unstaged_and_untracked_paths(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            def git(*args):
                return subprocess.check_output(['git', *args], cwd=root)
            git('init', '-q')
            for name in ['deleted.rs', 'staged.rs', 'unstaged.rs']:
                (root / name).write_text('before')
            git('add', '.')
            git('-c', 'user.name=Test', '-c', 'user.email=test@example.test',
                '-c', 'core.hooksPath=/dev/null', 'commit', '-qm', 'initial')
            (root / 'deleted.rs').unlink()
            (root / 'staged.rs').write_text('after')
            git('add', 'staged.rs')
            (root / 'unstaged.rs').write_text('after')
            (root / 'new file.rs').write_text('new')
            self.assertEqual(rust.changed_paths(root, 'HEAD'),
                             ['deleted.rs', 'new file.rs', 'staged.rs', 'unstaged.rs'])

    def test_nextest_keeps_doctests_and_cargo_build_profile(self):
        with patch.object(rust.shutil, 'which', return_value=None), \
             patch.object(rust.subprocess, 'run') as run, contextlib.redirect_stdout(io.StringIO()):
            run.return_value.returncode = 0
            self.assertEqual(rust.main(['-p', 'core', '--runner', 'nextest',
                                        '--profile', 'release', '--filter', 'restore']), 0)
        commands = [call.args[0] for call in run.call_args_list]
        self.assertIn('--cargo-profile', commands[0])
        self.assertIn('release', commands[0])
        self.assertIn('--doc', commands[1])
        self.assertTrue(all(command[-2:] == ['--', 'restore'] for command in commands))

    def test_first_failure_stops_and_list_does_not_execute(self):
        with patch.object(rust.subprocess, 'run') as run, contextlib.redirect_stdout(io.StringIO()):
            run.return_value.returncode = 7
            self.assertEqual(rust.main(['-p', 'core', '--runner', 'nextest']), 7)
            self.assertEqual(run.call_count, 1)
            run.reset_mock()
            self.assertEqual(rust.main(['-p', 'core', '--list']), 0)
            run.assert_not_called()


class RuntimeProfileTests(unittest.TestCase):
    def test_every_runtime_command_uses_requested_profile(self):
        with patch.object(runtime.subprocess, 'check_output', return_value=json.dumps({'packages': []})), \
             patch.object(runtime.subprocess, 'run') as run:
            runtime.main(['--profile', 'release'])
        self.assertEqual(run.call_count, 8)
        for call in run.call_args_list:
            command = call.args[0]
            self.assertEqual(command[command.index('--profile') + 1], 'release')
            self.assertIn('--no-default-features', command)


class PythonSelectionTests(unittest.TestCase):
    def test_exclusion_keeps_other_tests_in_nested_suites(self):
        cases = [unittest.FunctionTestCase(lambda: None, description='fast'),
                 unittest.FunctionTestCase(lambda: None, description='process')]
        # FunctionTestCase IDs use the function name.
        cases[0]._testFunc.__name__ = 'fast'
        cases[1]._testFunc.__name__ = 'process'
        suite = unittest.TestSuite([unittest.TestSuite(cases)])
        selected = python.select_tests(suite, ['process'])
        self.assertEqual([test.id() for test in selected], ['fast'])

    def test_empty_selection_is_an_error(self):
        with contextlib.redirect_stderr(io.StringIO()), self.assertRaises(SystemExit) as error:
            python.main(['--pattern', 'no-such-tests-*.py'])
        self.assertEqual(error.exception.code, 2)
