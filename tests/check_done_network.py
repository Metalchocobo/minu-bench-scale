"""Compile the real DONE staging, outbox publish and terminal ACK code."""

from __future__ import annotations

import argparse
from pathlib import Path
import re
import shutil
import subprocess
import tempfile


ROOT = Path(__file__).resolve().parents[1]


def braced(source: str, start: int) -> str:
    body = source.index("{", start)
    depth = 1
    cursor = body + 1
    while depth:
        depth += (source[cursor] == "{") - (source[cursor] == "}")
        cursor += 1
    return source[start:cursor]


def function(source: str, name: str) -> str:
    match = re.search(r"^(?:static\s+)?(?:bool|void|uint8_t|uint32_t|const char\*)\s+"
                      + re.escape(name) + r"\s*\([^;]*?\)\s*\{", source, re.M)
    if match is None:
        raise RuntimeError("Production function not found: " + name)
    return braced(source, match.start())


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--compiler")
    parser.add_argument("--vcvars", type=Path)
    args = parser.parse_args()
    source = (ROOT / "firmware/esp32_hx711_serial/net_ota_cloud.cpp").read_text(encoding="utf-8")
    enum_start = source.index("enum MqttResponseKind")
    production = braced(source, enum_start) + ";\n"
    production += "\n".join(function(source, name) for name in [
        "mqttResetPendingResponse", "mqttGenerateResponseId", "mqttPublishPendingResponse",
        "isMqttDoneActionable", "getMqttDoneProductId", "getMqttDoneCommandId",
        "getMqttDoneConnectionId", "mqttDoneActiveProduction", "mqttPopDoneAck", "mqttPopDoneReject"])
    ack = braced(source, source.index("if (isAckTopic)"))
    production += "\nstatic void receiveAck(const TestDoc& doc, const char* type) {\n"
    production += "const bool isAckTopic = true;\n" + ack + "\n}\n"
    # The enum must precede the harness state declarations.
    enum_end = production.index("};") + 2
    compiler = args.compiler or shutil.which("g++") or shutil.which("clang++") or "cl"
    with tempfile.TemporaryDirectory(prefix="minu-done-network-") as temporary:
        work = Path(temporary)
        (work / "done_network_enum.inc").write_text(production[:enum_end], encoding="utf-8")
        (work / "done_network_production.inc").write_text(production[enum_end:], encoding="utf-8")
        executable = work / "done-network.exe"
        if Path(compiler).stem.lower() == "cl":
            command = [compiler, "/nologo", "/std:c++17", "/EHsc", "/W4", "/D_CRT_SECURE_NO_WARNINGS",
                       "/I" + str(work), str(ROOT / "tests/firmware_done_network_harness.cpp"),
                       "/Fe:" + str(executable)]
            if args.vcvars:
                script = work / "compile.cmd"
                script.write_text('@call "' + str(args.vcvars) + '" >nul\n@' +
                                  subprocess.list2cmdline(command) + '\n', encoding="utf-8")
                subprocess.run(["cmd.exe", "/d", "/c", str(script)], cwd=work, check=True)
            else:
                subprocess.run(command, cwd=work, check=True)
        else:
            subprocess.run([compiler, "-std=c++17", "-Wall", "-Wextra", "-pedantic", "-I" + str(work),
                            str(ROOT / "tests/firmware_done_network_harness.cpp"), "-o", str(executable)],
                           cwd=work, check=True)
        subprocess.run([str(executable)], cwd=work, check=True)


if __name__ == "__main__":
    main()
