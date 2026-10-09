# CAMP candidate selector

`autoware_camp_selector` selects an original candidate from an ordered pool.
It is independent of Diffusion Planner, trajectory ranker, CUDA and TensorRT.
Other ML planners can call the C++ scorer or publish `CampCandidatePool` with
their original candidates and the same deployment atoms and endpoint states.

```cpp
#include <autoware/camp_selector/camp_ranker.hpp>

const auto model = autoware::camp_selector::load_camp_fixed_weight_model(model_path);
const auto decision = autoware::camp_selector::rank_camp_candidates(model, status, raw_atoms);
const auto & selected = original_candidates.at(decision.selected_index);
```

Link `autoware::camp_selector` after `find_package(autoware_camp_selector REQUIRED)`.
The scorer itself has no ROS dependency. Its optional tensor/map materializer
depends on Eigen and Boost; the ROS transport is a separate executable.

## Scoring and input contract

The included version-1 Fixed export is unchanged: K=8, 16 atoms, 24 endpoint-status
heads, `clip(raw / scale, 0, 10)`, lowest cost and first original row on ties.
Its JSON is the original small scoring resource, rather than a generator checkpoint.
Observed atoms must be finite; unavailable and inapplicable endpoints remain inactive
and may contain NaN. Unsupported status patterns and mismatched pools are rejected.

`CampCandidatePool` is one atomic packet containing a header, monotonic `pool_id`,
ordered original candidates, 16 endpoint states and row-major K x 16 raw atoms.
The node publishes the original selected trajectory and turn indication, together
with a `CampSelection` decision. Candidate0 remains row0; selection does not modify
candidate coordinates, point fields or ordering. No teacher or actual future is an
online input.

`camp_atom_materializer.hpp` preserves the existing deployment formulas and layout:
80 steps at 0.1 s, ego plus the first 32 actors. The host provides map-coordinate
lane boundaries and the original `ego_to_map` transform. A nonnull empty boundary
vector means an available empty map; a null pointer uses the tensor geometry path.
The host supplies its previous accepted world plan and resets continuity at episode,
route/map and clock boundaries. Selection alone is not controller acknowledgement.

## Build and use

In a compatible ROS underlay:

```sh
colcon build --packages-select autoware_camp_selector
colcon test --packages-select autoware_camp_selector
ros2 launch autoware_camp_selector camp_selector.launch.xml
```

The default input is `/planning/camp/candidate_pool`; outputs are
`/planning/camp/trajectory`, `/planning/camp/turn_indicators` and
`/planning/camp/selection`. Launch arguments allow remapping and selecting the
fixed-weight resource.

For a standalone C++ build with nlohmann_json, GTest, Eigen and Boost:

```sh
cmake -S . -B build -DCAMP_STANDALONE=ON
cmake --build build
ctest --test-dir build --output-on-failure
cmake --install build --prefix install
```

Use `-DCAMP_BUILD_TENSOR_MATERIALIZER=OFF` for a scorer-only build. It explicitly
omits materializer implementation and tests; enable it before using that API.
`examples/installed` exercises the installed library export.

The tests cover fixed scoring, status handling, ties, malformed pools, raw-context
capture, publication-ledger resets and map-boundary materialization. A separate DDS
test checks selection transport, malformed-pool rejection and recovery. See the
[Diffusion Planner adapter](../autoware_camp_diffusion_adapter/README.md) for the
recorded integration demonstration and its scope.
