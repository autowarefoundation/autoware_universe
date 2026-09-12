#!/usr/bin/env python3
# Copyright 2024 Tier IV, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

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

LICENSE_HEADER = """# Copyright 2024 Tier IV, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""


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

    # Note: --pyi_out is intentionally omitted. It needs the protoc-gen-pyi plugin,
    # which the distro python3-grpc-tools package does not ship, and the .pyi stub
    # is only a type hint that is not needed at runtime.
    command = [
        sys.executable,
        "-m",
        "grpc_tools.protoc",
        f"-I{proto_dir}",
        f"--python_out={out_dir}",
        f"--grpc_python_out={out_dir}",
        str(proto_file),
    ]
    print("Running:", " ".join(command))
    subprocess.run(command, check=True)

    stem = proto_file.stem
    pb2_module = out_dir / f"{stem}_pb2.py"
    grpc_module = out_dir / f"{stem}_pb2_grpc.py"

    # protoc emits a flat ``import <stem>_pb2`` in the gRPC stub. Rewrite it to
    # the fully-qualified package path so the module resolves once installed.
    grpc_text = grpc_module.read_text()
    grpc_text = grpc_text.replace(
        f"import {stem}_pb2 as",
        f"from {args.package} import {stem}_pb2 as",
    )
    grpc_text = grpc_text.replace(
        f"from . import {stem}_pb2 as",
        f"from {args.package} import {stem}_pb2 as",
    )
    grpc_module.write_text(grpc_text)

    # Prepend the license header the ament_copyright linter requires (the
    # generated files are scanned along with the hand-written sources).
    for module in (pb2_module, grpc_module):
        module.write_text(LICENSE_HEADER + module.read_text())

    # Keep the output directory importable as a Python package.
    (out_dir / "__init__.py").touch()

    return 0


if __name__ == "__main__":
    sys.exit(main())
