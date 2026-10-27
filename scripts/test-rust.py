#!/usr/bin/env python3
# SPDX-License-Identifier: MIT OR Apache-2.0
# Copyright (c) 2026 hxyulin <hxyulin@proton.me>
"""Test selected Rust packages and their workspace consumers, or inspect the plan."""

import argparse
import json
from pathlib import Path
import shlex
import shutil
import subprocess


def affected_packages(metadata, paths):
    """Return a conservative reverse dependency closure; None means the workspace."""
    root = Path(metadata['workspace_root'])
    members = set(metadata['workspace_members'])
    packages = [p for p in metadata['packages'] if p['id'] in members]
    directories = {Path(p['manifest_path']).parent: p['name'] for p in packages}
    selected = set()
    for path in paths:
        candidate = root / path
        owners = [directory for directory in directories if candidate.is_relative_to(directory)]
        if owners:
            selected.add(directories[max(owners, key=lambda directory: len(directory.parts))])
        elif path.startswith('docs/') or Path(path).suffix == '.md':
            continue
        else:
            # Unknown tooling/configuration and workspace manifests can affect every crate.
            return None
    while True:
        previous = selected.copy()
        for package in packages:
            # Include normal, build and dev dependencies, even optional or target-specific.
            if any(dependency.get('path') and
                   directories.get(Path(dependency['path'])) in selected
                   for dependency in package['dependencies']):
                selected.add(package['name'])
        if selected == previous:
            return sorted(selected)


def changed_paths(root, base):
    """Include committed, staged, unstaged and untracked files relative to base."""
    paths = set()
    for command in [
        ['git', 'diff', '--no-renames', '--name-only', '-z', base, '--'],
        ['git', 'ls-files', '--others', '--exclude-standard', '-z'],
    ]:
        output = subprocess.check_output(command, cwd=root)
        paths.update(path.decode() for path in output.split(b'\0') if path)
    return sorted(paths)


def main(argv=None):
    """Print the exact commands before running them; never infer individual test coverage."""
    parser = argparse.ArgumentParser(description=__doc__)
    selection = parser.add_mutually_exclusive_group(required=True)
    selection.add_argument('-p', '--package', action='append', help='package, repeatable')
    selection.add_argument('--changed-since', metavar='REF', help='changed packages and all consumers')
    selection.add_argument('--workspace', action='store_true')
    parser.add_argument('--filter', default='', help='test-name substring')
    parser.add_argument('--profile', default='test', help='Cargo build profile')
    parser.add_argument('--runner', choices=['cargo', 'nextest'], default='cargo')
    parser.add_argument('--list', action='store_true', help='print the plan without compiling')
    args = parser.parse_args(argv)
    root = Path(__file__).resolve().parents[1]
    packages = args.package
    if args.changed_since is not None:
        metadata = json.loads(subprocess.check_output(
            ['cargo', 'metadata', '--locked', '--no-deps', '--format-version', '1'], cwd=root))
        packages = affected_packages(metadata, changed_paths(root, args.changed_since))
        if packages == []:
            print('No Rust packages affected. Python and repository checks run separately.')
            return 0
    options = ['--all-features', '--locked']
    if packages is None:
        options += ['--workspace']
    else:
        for package in packages:
            options += ['-p', package]
    rust = 'mbx' if shutil.which('mbx') else 'cargo'
    if args.runner == 'nextest':
        # nextest's --profile selects runner settings, not Cargo's build profile.
        commands = [[rust, 'nextest', 'run', '--cargo-profile', args.profile, *options],
                    [rust, 'test', '--doc', '--profile', args.profile, *options]]
    else:
        commands = [[rust, 'test', '--profile', args.profile, *options]]
    for command in commands:
        if args.filter:
            command += ['--', args.filter]
        print(shlex.join(command), flush=True)
        if not args.list:
            result = subprocess.run(command, cwd=root)
            if result.returncode:
                return result.returncode
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
