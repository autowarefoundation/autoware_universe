^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_cuda_pointcloud_preprocessor
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.43.0 (2025-03-21)
-------------------
* fix: update tool version
* Merge remote-tracking branch 'origin/main' into chore/bump-version-0.43
* chore(autoware_cuda_pointcloud_preprocessor): add maintainer (`#10297 <https://github.com/autowarefoundation/autoware_universe/issues/10297>`_)
* feat(autoware_cuda_pointcloud_preprocessor): a cuda-accelerated pointcloud preprocessor (`#9454 <https://github.com/autowarefoundation/autoware_universe/issues/9454>`_)
  * feat: moved the cuda pointcloud preprocessor and organized from a personal repository
  * chore: fixed incorrect links
  * chore: fixed dead links pt2
  * chore: fixed spelling errors
  * chore: json schema fixes
  * chore: removed comments and filled the fields
  * fix: fixed the adapter for the case when the number of points in the pointcloud changes after the first iteration
  * feat: used the cuda host allocators for aster host to device copies
  * Update sensing/autoware_cuda_pointcloud_preprocessor/docs/cuda-pointcloud-preprocessor.md
  Co-authored-by: Max Schmeller <6088931+mojomex@users.noreply.github.com>
  * Update sensing/autoware_cuda_pointcloud_preprocessor/src/cuda_pointcloud_preprocessor/cuda_pointcloud_preprocessor.cu
  Co-authored-by: Manato Hirabayashi <3022416+manato@users.noreply.github.com>
  * Update sensing/autoware_cuda_pointcloud_preprocessor/src/cuda_pointcloud_preprocessor/cuda_pointcloud_preprocessor.cu
  Co-authored-by: Manato Hirabayashi <3022416+manato@users.noreply.github.com>
  * style(pre-commit): autofix
  * Update sensing/autoware_cuda_pointcloud_preprocessor/docs/cuda-pointcloud-preprocessor.md
  Co-authored-by: Max Schmeller <6088931+mojomex@users.noreply.github.com>
  * Update sensing/autoware_cuda_pointcloud_preprocessor/README.md
  Co-authored-by: Max Schmeller <6088931+mojomex@users.noreply.github.com>
  * Update sensing/autoware_cuda_pointcloud_preprocessor/README.md
  Co-authored-by: Max Schmeller <6088931+mojomex@users.noreply.github.com>
  * Update sensing/autoware_cuda_pointcloud_preprocessor/src/cuda_pointcloud_preprocessor/cuda_pointcloud_preprocessor.cu
  Co-authored-by: Max Schmeller <6088931+mojomex@users.noreply.github.com>
  * style(pre-commit): autofix
  * Update sensing/autoware_cuda_pointcloud_preprocessor/src/cuda_pointcloud_preprocessor/cuda_pointcloud_preprocessor.cu
  Co-authored-by: Manato Hirabayashi <3022416+manato@users.noreply.github.com>
  * style(pre-commit): autofix
  * Update sensing/autoware_cuda_pointcloud_preprocessor/src/cuda_pointcloud_preprocessor/cuda_pointcloud_preprocessor.cu
  Co-authored-by: Manato Hirabayashi <3022416+manato@users.noreply.github.com>
  * style(pre-commit): autofix
  * chore: fixed code compilation to reflect Hirabayashi-san's  memory pool proposal
  * feat: generalized the number of crop boxes. For two at least, the new approach is actually faster
  * chore: updated config, schema, and handled the null case in a specialized way
  * feat: moving the pointcloud organization into gpu
  * feat: reimplemented the organized pointcloud adapter in cuda. the only bottleneck is the H->D copy
  * chore: removed redundant ternay operator
  * chore: added a temporary memory check. the check will be unified in a later PR
  * chore: refactored the structure to avoid large files
  * chore: updated the copyright year
  * fix: fixed a bug in the undistortion kernel setup. validated it comparing it with the baseline
  * chore: removed unused packages
  * chore: removed mentions of the removed adapter
  * chore: fixed missing autoware prefix
  * fix: missing assignment in else branch
  * chore: added cuda/nvcc debug flags on debug builds
  * chore: refactored parameters for the undistortion settings
  * chore: removed unused headers
  * chore: changed default crop box to no filtering at all
  * feat: added missing restrict keyword
  * chore: spells
  * chore: removed default destructor
  * chore: ocd activated (spelling)
  * chore: fixed the schema
  * chore: improved readibility
  * chore: added dummy crop box
  * chore: added new repositories to ansible
  * chore: CI/CD
  * chore: more CI/CD
  * chore: mode CI/CD. some linters are conflicting
  * style(pre-commit): autofix
  * chore: ignoring uncrustify
  * chore: ignoring more uncrustify
  * chore: missed one more uncrustify exception
  * chore: added meta dep
  ---------
  Co-authored-by: Max Schmeller <6088931+mojomex@users.noreply.github.com>
  Co-authored-by: Manato Hirabayashi <3022416+manato@users.noreply.github.com>
  Co-authored-by: pre-commit-ci[bot] <66853113+pre-commit-ci[bot]@users.noreply.github.com>
  Co-authored-by: Amadeusz Szymko <amadeusz.szymko.2@tier4.jp>
* Contributors: Amadeusz Szymko, Hayato Mizushima, Kenzo Lobos Tsunekawa
