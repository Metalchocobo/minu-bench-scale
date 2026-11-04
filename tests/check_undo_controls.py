"""Run the sketch's real UNDO functions against production stack/scale code.

The generated include is temporary: the harness never keeps a copied model of
the controller. The ESP32 build remains the final sketch syntax/type check.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import shutil
import subprocess
import tempfile


ROOT = Path(__file__).resolve().parents[1]
FIRMWARE = ROOT / "firmware" / "esp32_hx711_serial"


def extract_function(source: str, name: str) -> str:
    import re

    match = re.search(r"^static\s+(?:bool|void)\s+" + re.escape(name) + r"\s*\(", source, re.M)
    if match is None:
        raise RuntimeError(f"Production function not found: {name}")
    body = source.index("{", match.start())
    # These controller bodies contain no brace-bearing raw string literals.
    depth = 1
    cursor = body + 1
    while depth:
        depth += (source[cursor] == "{") - (source[cursor] == "}")
        cursor += 1
    return source[match.start():cursor] + "\n"


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--compiler", help="C++ compiler; defaults to g++/clang++ or MSVC")
    parser.add_argument("--vcvars", type=Path, help="vcvars64.bat when MSVC is not initialized")
    arguments = parser.parse_args()
    source = (FIRMWARE / "esp32_hx711_serial.ino").read_text(encoding="utf-8")
    names = ["restoreTopStackEntryAfterUndo", "completeRemoteUndo", "applyDetachedUndoNow",
             "isTareBehaviorLocked", "performUndoAction", "updateUndoKey"]
    production = "\n".join(extract_function(source, name) for name in names)
    compiler = arguments.compiler or shutil.which("g++") or shutil.which("clang++") or "cl"
    sources = [ROOT / "tests" / "firmware_undo_harness.cpp", FIRMWARE / "weigh_stack.cpp",
               FIRMWARE / "scale_state.cpp", FIRMWARE / "scale_filters.cpp"]
    includes = [ROOT / "tests" / "undo_stubs", ROOT / "tests" / "scale_stubs", ROOT]
    with tempfile.TemporaryDirectory(prefix="minu-undo-") as temporary:
        work = Path(temporary)
        (work / "undo_controls_production.inc").write_text(production, encoding="utf-8")
        includes.append(work)
        executable = work / ("undo-controls.exe" if Path(compiler).stem.lower() == "cl" else "undo-controls")
        if Path(compiler).stem.lower() == "cl":
            command = [compiler, "/nologo", "/std:c++17", "/EHsc", "/W4", "/D_CRT_SECURE_NO_WARNINGS"]
            command += ["/I" + str(path) for path in includes]
            command += [str(path) for path in sources] + ["/Fe:" + str(executable)]
            if arguments.vcvars:
                script = work / "compile.cmd"
                script.write_text('@call "' + str(arguments.vcvars) + '" >nul\n@' +
                                  subprocess.list2cmdline(command) + '\n', encoding="utf-8")
                subprocess.run(["cmd.exe", "/d", "/c", str(script)], cwd=work, check=True)
            else:
                subprocess.run(command, cwd=work, check=True)
        else:
            command = [compiler, "-std=c++17", "-Wall", "-Wextra", "-pedantic"]
            command += ["-I" + str(path) for path in includes]
            command += [str(path) for path in sources] + ["-o", str(executable)]
            subprocess.run(command, cwd=work, check=True)
        subprocess.run([str(executable)], cwd=work, check=True)


if __name__ == "__main__":
    main()
