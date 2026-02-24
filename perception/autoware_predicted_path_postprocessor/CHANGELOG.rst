^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package autoware_predicted_path_postprocessor
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.50.0 (2026-02-14)
-------------------
* Merge remote-tracking branch 'origin/main' into humble
* fix: add missing ament_index_cpp dependency (`#11875 <https://github.com/autowarefoundation/autoware_universe/issues/11875>`_)
* Contributors: Mete Fatih Cırıt, Ryohsuke Mitsudome

0.49.0 (2025-12-30)
-------------------
* Merge remote-tracking branch 'origin/main' into prepare-0.49.0-changelog
* feat(autoware_lanelet2_utils): replace from/toBinMsg (Sensing, Visualization and Perception Component) (`#11785 <https://github.com/autowarefoundation/autoware_universe/issues/11785>`_)
  * perception component toBinMsg replacement
  * visualization component fromBinMsg replacement
  * sensing component fromBinMsg replacement
  * perception component fromBinMsg replacement
  ---------
* Contributors: Ryohsuke Mitsudome, Sarun MUKDAPITAK

0.48.0 (2025-11-18)
-------------------
* Merge remote-tracking branch 'origin/main' into humble
* feat(predicted_path_postprocessor): add a new package to perform post-process for predicted paths (`#11421 <https://github.com/autowarefoundation/autoware_universe/issues/11421>`_)
  * feat: add baseline
  * feat: add support of publishing debug information
  * refactor: modify input to mutable reference and not to return object
  * feat: add new processor to refine objecs paths by their speed
  * chore: remove sample processor
  * chore: rename namespace
  * docs: add processor naming rules to README
  * fix: consider monotonically increasing
  * feat: update to publish processing time and cyclic time in ms
  * refactor: replace IntermediatePublisher into DebugPublisher
  * feat: include processing time in intermediate reports
  * feat: add lanelet data to the context
  * feat: add Report class and apply it to proccesor output
  * refactor: rename launch arguments
  * refactor: aggregate parameter files
  * refactor: store objects message as shared_ptr in context to enable reflecting their processed result
  * chore: update README and add JSON schema
  * chore: fix typo
  * chore: update maintainers and codeowners
  * test: resolve test failure
  * feat: update processor interface
  * feat: enable to specify interpolation method from config
  ---------
  Co-authored-by: Taekjin LEE <taekjin.lee@tier4.jp>
* Contributors: Kotaro Uetake, Ryohsuke Mitsudome
