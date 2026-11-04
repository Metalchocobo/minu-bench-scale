"""Compile the sketch's real DONE gesture/feedback helpers without hardware."""
from pathlib import Path
import argparse
import shutil
import subprocess
import tempfile
import re

ROOT = Path(__file__).resolve().parents[1]


def extract_function(source, name):
    match = re.search(r'^static\s+(?:bool|void)\s+' + re.escape(name) + r'\s*\(', source, re.M)
    if match is None:
        raise RuntimeError('Production function not found: ' + name)
    cursor = source.index('{', match.start()) + 1
    depth = 1
    while depth:
        depth += (source[cursor] == '{') - (source[cursor] == '}')
        cursor += 1
    return source[match.start():cursor] + '\n'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--vcvars', type=Path)
    args = parser.parse_args()
    compiler = shutil.which('g++') or shutil.which('clang++') or 'cl'
    source = (ROOT / 'firmware/esp32_hx711_serial/esp32_hx711_serial.ino').read_text(encoding='utf-8')
    production = '\n'.join(extract_function(source, name) for name in (
        'isDoneBehaviorReady', 'startDoneKey', 'updateDoneKey', 'completeRemoteDone'))
    with tempfile.TemporaryDirectory(prefix='minu-done-') as temporary:
        work = Path(temporary)
        (work / 'done_controls_production.inc').write_text(production, encoding='utf-8')
        executable = work / 'done-controls.exe'
        includes = [ROOT / 'tests/undo_stubs', ROOT, work]
        harness = ROOT / 'tests/firmware_done_controls_harness.cpp'
        if compiler == 'cl':
            command = ['cl', '/nologo', '/std:c++17', '/EHsc', '/W4']
            command += ['/I' + str(path) for path in includes]
            command += [str(harness), '/Fe:' + str(executable)]
            if args.vcvars:
                script = work / 'compile.cmd'
                script.write_text('@call "' + str(args.vcvars) + '" >nul\n@' +
                                  subprocess.list2cmdline(command) + '\n', encoding='utf-8')
                subprocess.run(['cmd.exe', '/d', '/c', str(script)], cwd=work, check=True)
            else:
                subprocess.run(command, cwd=work, check=True)
        else:
            command = [compiler, '-std=c++17', '-Wall', '-Wextra', '-Werror']
            command += ['-I' + str(path) for path in includes]
            subprocess.run(command + [str(harness), '-o', str(executable)], cwd=work, check=True)
        subprocess.run([str(executable)], cwd=work, check=True)


if __name__ == '__main__':
    main()
