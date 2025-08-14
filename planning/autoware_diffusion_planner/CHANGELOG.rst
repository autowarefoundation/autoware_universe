^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_diffusion_planner
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.47.1 (2025-08-14)
-------------------

0.47.0 (2025-08-11)
-------------------
* refactor(diffusion_planner): remove unused code (`#11137 <https://github.com/autowarefoundation/autoware_universe/issues/11137>`_)
  Removed unused code
* feat: update diffusion planner inputs/outputs (`#11093 <https://github.com/autowarefoundation/autoware_universe/issues/11093>`_)
  * Updated
  * Fixed ego_shape
  * Removed unnecessary returns
  * Fixed for cpplint
  * Applied the formatter
  * Removed test for traffic light state
  * Fixed lane_segments_test
  * Moved AddTrafficLightOneHotEncodingToSegmentNoTrafficLight from `lane_segments_test.cpp` to `lanelet_integration_test.cpp`
  * Added `#include <map>`
  * Added EGO_AGENT_PAST_IDX_X
  * Fix
  * Fix
  * Fixed remap params
  * Fixed nits
  ---------
* style(pre-commit): update to clang-format-20 (`#11088 <https://github.com/autowarefoundation/autoware_universe/issues/11088>`_)
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* docs(autoware_diffusion_planner): remove obsolete information (`#11061 <https://github.com/autowarefoundation/autoware_universe/issues/11061>`_)
  * docs(autoware_diffusion_planner): remove obsolete information
  * style(pre-commit): autofix
  ---------
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
* fix(diffusion_planner): modify unread parameters (`#11025 <https://github.com/autowarefoundation/autoware_universe/issues/11025>`_)
  * fix(diffusion_planner): modify unread parameters
  * fix(diffusion_planner): remove unused artifact_dir argument
  ---------
* feat(autoware_diffusion_planner): add diffusion-based trajectory planner (`#10957 <https://github.com/autowarefoundation/autoware_universe/issues/10957>`_)
  * feat(autoware_diffusion_planner): add diffusion-based trajectory planner
  * fix: dead links in README.md
  * fix: fix by pre-commit
  * fix: modify spell for cspell
  * refactor: reorganize CMakeLists.txt for better structure and clarity
  * fix: modify for pre-commit ci
  * fix: update for cppcheck
  * fix: update for pre-commit
  * cosmetic change
  * rename test dir
  * fix: modify for pre-commit
  * change output topic name
  * add maintainer
  * remove unnecessary section in readme
  * fixed no install in cmake
  * fix wrong syntax in launch
  * refactor: consolidate geometry conversion functions into a template
  * fix: remove redundant return statement and improve string formatting in to_string methods
  * cosmetic change
  * fix: remove example configuration section from README
  * fix: remove outdated link to Autoware Universe from README
  * fix: remove unused parameters from launch files and restore default build_only value
  * fix: update input and output sections in README for clarity and consistency
  * fix: update diffusion planner parameters and remove unused launch file
  * fix: add JSON schema for diffusion planner parameters and update README
  * fix: update JSON schema path for diffusion planner parameters in README
  ---------
* Contributors: Mete Fatih Cırıt, SakodaShintaro, Shintaro Tomie, Yukihiro Saito
