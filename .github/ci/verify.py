#!/usr/bin/env python3
"""Run one fixed robot check on a disposable GitHub-hosted VM."""

import hashlib
import json
import os
from pathlib import Path
import re
import shutil
import signal
import subprocess
import sys
import threading
import time
import xml.etree.ElementTree as ET
import zipfile

TASKS = {'compile': ':TeamCode:assembleDebug',
         'unit_tests': ':TeamCode:testDebugUnitTest', 'lint': ':TeamCode:lintDebug'}
SHA256 = '20f1b1176237254a6fc204d8434196fa11a4cfb387567519c61556e8710aed78'
MAX_LOG = 2 * 1024 * 1024
MAX_XML = 8 * 1024 * 1024


def xml_root(path):
    if path.is_symlink() or path.stat().st_size > MAX_XML:
        raise ValueError('Unexpected or oversized XML report')
    return ET.fromstring(path.read_bytes())


def main():
    kind = sys.argv[1]
    if kind not in TASKS:
        raise SystemExit('Unknown check')
    for module in ('TeamCode', 'FtcRobotController'):
        path = Path(module) / 'build'
        if path.is_symlink():
            raise SystemExit('Build output directory must not be a symlink')
        if path.exists():
            shutil.rmtree(path)
    destination = Path('ci-results') / kind
    destination.mkdir(parents=True, exist_ok=True)
    properties = Path('gradle/wrapper/gradle-wrapper.properties')
    text = properties.read_text()
    if r'distributionUrl=https\://services.gradle.org/distributions/gradle-8.13-bin.zip' not in text:
        raise SystemExit('This workflow requires the reviewed Gradle 8.13 toolchain')
    prior = re.search(r'^distributionSha256Sum=(.+)$', text, re.MULTILINE)
    if prior and prior.group(1).strip() != SHA256:
        raise SystemExit('Unexpected Gradle distribution checksum')
    if not prior:
        properties.write_text(text.rstrip() + '\ndistributionSha256Sum=' + SHA256 + '\n')
    Path('gradlew').chmod(0o755)
    command = ['./gradlew', '--no-daemon', '--console=plain', '--max-workers=2',
               '-Dorg.gradle.jvmargs=-Xmx1536m', TASKS[kind]]
    started = time.monotonic()
    log_path = destination / 'gradle.log'
    with log_path.open('wb') as log:
        process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                   start_new_session=True)
        def drain():
            kept = 0
            while True:
                chunk = process.stdout.read(65536)
                if not chunk:
                    break
                remaining = max(0, MAX_LOG - kept)
                log.write(chunk[:remaining])
                kept += min(len(chunk), remaining)
        reader = threading.Thread(target=drain, daemon=True)
        reader.start()
        try:
            code = process.wait(timeout=900)
            timed_out = False
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait()
            code, timed_out = 124, True
        reader.join(timeout=10)
        if reader.is_alive():
            raise SystemExit('Build log reader did not stop')
    result = {'schema_version': 1, 'check': kind,
              'status': 'passed' if code == 0 else 'failed',
              'exit_code': code, 'timed_out': timed_out,
              'seconds': round(time.monotonic() - started, 1),
              'head_sha': os.environ['PR_HEAD_SHA'], 'base_sha': os.environ['PR_BASE_SHA'],
              'merge_sha': os.environ['GITHUB_SHA'],
              'run_id': int(os.environ['GITHUB_RUN_ID']),
              'run_attempt': int(os.environ['GITHUB_RUN_ATTEMPT'])}
    counts = {'tests': 0, 'failures': 0, 'errors': 0, 'skipped': 0}
    warnings, errors = 0, 0
    if kind == 'unit_tests':
        for path in Path('TeamCode/build/test-results/testDebugUnitTest').glob('TEST-*.xml'):
            suite = xml_root(path)
            for key in counts:
                value = int(suite.get(key, '0'))
                if value < 0 or value > 100000:
                    raise ValueError('Invalid test count')
                counts[key] += value
            shutil.copyfile(path, destination / path.name)
        result['counts'] = counts
        if counts['failures'] or counts['errors']:
            result['status'] = 'failed'
        elif code == 0 and counts['tests'] == 0:
            result['status'] = 'no_tests_found'
        elif code == 0 and counts['skipped'] == counts['tests']:
            result['status'] = 'no_tests_executed'
    if kind == 'lint':
        path = Path('TeamCode/build/reports/lint-results-debug.xml')
        if path.exists():
            root = xml_root(path)
            for issue in root.findall('issue'):
                if issue.get('severity') in ('Error', 'Fatal'):
                    errors += 1
                elif issue.get('severity') == 'Warning':
                    warnings += 1
            shutil.copyfile(path, destination / path.name)
        elif code == 0:
            result['status'] = 'failed'
        result['lint_warnings'], result['lint_errors'] = warnings, errors
        if errors:
            result['status'] = 'failed'
    if kind == 'compile' and code == 0:
        apks = list(Path('TeamCode/build/outputs/apk/debug').glob('*.apk'))
        if not apks:
            result['status'] = 'failed'
        result['apks'] = []
        for apk in apks:
            with zipfile.ZipFile(apk) as package:
                if not {'AndroidManifest.xml', 'classes.dex'} <= set(package.namelist()):
                    result['status'] = 'failed'
            with apk.open('rb') as stream:
                digest = hashlib.file_digest(stream, 'sha256').hexdigest()
            result['apks'].append({'name': apk.name, 'bytes': apk.stat().st_size, 'sha256': digest})
    (destination / 'result.json').write_text(json.dumps(result, indent=2) + '\n')
    outputs = {**counts, 'status': result['status'], 'lint_warnings': warnings, 'lint_errors': errors}
    with open(os.environ['GITHUB_OUTPUT'], 'a') as output:
        for key, value in outputs.items():
            output.write(f'{key}={value}\n')
    with open(os.environ['GITHUB_STEP_SUMMARY'], 'a') as summary:
        summary.write(f'### FTC {kind.replace("_", " ")}\n\nResult: **{result["status"]}**.\n\n')
        summary.write(f'Tested PR head `{result["head_sha"]}` with base `{result["base_sha"]}`.\n\n')
        if kind == 'unit_tests':
            summary.write(f'{counts["tests"]} tests; {counts["failures"]} failures; '
                          f'{counts["errors"]} errors; {counts["skipped"]} skipped.\n\n')
            if result['status'].startswith('no_tests'):
                summary.write('No automated test coverage was established by this run.\n')
                print('::warning title=No unit test coverage::No tests ran; a successful Gradle task is not test coverage.')
        if kind == 'lint':
            summary.write(f'{warnings} warnings; {errors} errors.\n')
    print(json.dumps({key: value for key, value in result.items() if key != 'apks'}))
    if result['status'] == 'failed':
        for line in log_path.read_text(errors='replace').splitlines()[-30:]:
            safe = re.sub(r'[\x00-\x1f\x7f]', '?', line).replace('::', ': :')
            print('Build log: ' + safe[:500])
    return 0 if result['status'] in ('passed', 'no_tests_found', 'no_tests_executed') else 1


if __name__ == '__main__':
    raise SystemExit(main())
