^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_trajectory_traffic_rule_filter
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
* feat: add traffic rule validation for generator-selector framework (`#11426 <https://github.com/autowarefoundation/autoware_universe/issues/11426>`_)
  * bring packages from new planning framework
  * use filter as plugin
  * fix namespace in CMAKE
  * update README
  * small fix for README
  * fix publisher and subscriber
  * remove stop line filter
  * change subscriber/publisher deafult value
  * use boundary checker to get lanelets
  * initialize boundary checker
  * refactor feasible check
  * fix package.xml
  ---------
* Contributors: Go Sakayori, Ryohsuke Mitsudome
