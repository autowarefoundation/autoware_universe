^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_trajectory_safety_filter
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.50.0 (2026-02-14)
-------------------
* Merge remote-tracking branch 'origin/main' into humble
* chore(trajectory_safety_filter): fix maintainer (`#12095 <https://github.com/autowarefoundation/autoware_universe/issues/12095>`_)
  put back saito-san
* chore(trajectory_safety_filter): add maintainer (`#12087 <https://github.com/autowarefoundation/autoware_universe/issues/12087>`_)
  * chore(trajectory_safety_filter): add maintainer
  * chore: rearrange maintainer order alphabetically
  * chore: remove Sakoda-san and Saito-san
  ---------
* feat(safety_filter): subscribe to acceleration (`#12030 <https://github.com/autowarefoundation/autoware_universe/issues/12030>`_)
  feat: subscribe to acceleration
* Contributors: Go Sakayori, Ryohsuke Mitsudome, Zulfaqar Azmi

0.49.0 (2025-12-30)
-------------------
* Merge remote-tracking branch 'origin/main' into prepare-0.49.0-changelog
* feat(autoware_lanelet2_utils): replace from/toBinMsg (Planning and Control Component) (`#11784 <https://github.com/autowarefoundation/autoware_universe/issues/11784>`_)
  * planning component toBinMsg replacement
  * control component fromBinMsg replacement
  * planning component fromBinMsg replacement
  ---------
* Contributors: Ryohsuke Mitsudome, Sarun MUKDAPITAK

0.48.0 (2025-11-18)
-------------------
* Merge remote-tracking branch 'origin/main' into humble
* feat: add safety gate for generator-selector framework (`#11404 <https://github.com/autowarefoundation/autoware_universe/issues/11404>`_)
  * copy packages from new planning framework
  * introduce plugin
  * add/remove plugins
  * fix precommit
  * fix readme
  * fix include guard
  * fix precommit
  * remove test section in CMakeList
  * change definition in README
  * add comment for ttc calculation
  * use lambda function for check_collison function
  * calculate obstacle position only once
  * use boundary departure checker
  * add future work section to README
  * small fix for if condition
  ---------
* Contributors: Go Sakayori, Ryohsuke Mitsudome
