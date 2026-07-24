#!/usr/bin/env bash
# Refresh the vendored SplatSim proto from a tier4/splatsim release tag.
#
# The autoware build sandbox has no outbound internet, so rendering_service.proto
# is vendored into the package and the *_pb2*.py stubs are generated from it at
# build time. Run this from a networked machine when bumping the pinned version,
# then update SPLATSIM_VERSION in CMakeLists.txt to match.
#
# Usage: tools/update_splatsim_proto.sh [version]   # default: v1.2.0
set -euo pipefail

version="${1:-v1.2.0}"
script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
dest="${script_dir}/../src/autoware_carla_interface/splatsim/proto/rendering_service.proto"
url="https://raw.githubusercontent.com/tier4/splatsim/${version}/proto/splatsim/v1/rendering_service.proto"

echo "Fetching SplatSim proto ${version} from ${url}"
curl -fsSL "${url}" -o "${dest}"
echo "Updated ${dest}"
