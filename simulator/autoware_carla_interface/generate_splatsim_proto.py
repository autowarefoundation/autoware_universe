#!/usr/bin/env python3
"""Generate the SplatSim gRPC Python stubs from a proto file.

Invoked by CMakeLists.txt at build time. The proto file is fetched from the
pinned tier4/splatsim release, and the generated ``*_pb2*.py`` modules are
written into the package so ``ament_python_install_package`` can install them.
The generated modules are intentionally not committed (see .gitignore); they
are produced on every build to stay in sync with the installed protobuf/grpc
runtime.
"""

from __future__ import annotations

import argparse
from pathlib import Path
import subprocess
import sys


def parse_args() -> argparse.Namespace:
    """Parse the command-line arguments."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--proto", required=True, help="Path to the .proto file.")
    parser.add_argument("--out-dir", required=True, help="Directory for generated modules.")
    parser.add_argument(
        "--package",
        default="autoware_carla_interface.splatsim.proto",
        help="Python package the generated modules are installed under.",
    )
    return parser.parse_args()


def main() -> int:
    """Run protoc and rewrite the stub import to the installed package path."""
    args = parse_args()

    proto_file = Path(args.proto).resolve()
    proto_dir = proto_file.parent
    out_dir = Path(args.out_dir).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    command = [
        sys.executable,
        "-m",
        "grpc_tools.protoc",
        f"-I{proto_dir}",
        f"--python_out={out_dir}",
        f"--grpc_python_out={out_dir}",
        f"--pyi_out={out_dir}",
        str(proto_file),
    ]
    print("Running:", " ".join(command))
    subprocess.run(command, check=True)

    # protoc emits a flat ``import <stem>_pb2`` in the gRPC stub. Rewrite it to
    # the fully-qualified package path so the module resolves once installed.
    stem = proto_file.stem
    grpc_module = out_dir / f"{stem}_pb2_grpc.py"
    text = grpc_module.read_text()
    text = text.replace(
        f"import {stem}_pb2 as",
        f"from {args.package} import {stem}_pb2 as",
    )
    text = text.replace(
        f"from . import {stem}_pb2 as",
        f"from {args.package} import {stem}_pb2 as",
    )
    grpc_module.write_text(text)

    # Keep the output directory importable as a Python package.
    (out_dir / "__init__.py").touch()

    return 0


if __name__ == "__main__":
    sys.exit(main())
