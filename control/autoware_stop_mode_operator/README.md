# autoware_stop_mode_operator

This package publishes a stop command used when stop mode is selected.

## Auto parking

If `enable_auto_parking` is set to `true`, automatically shifts gear to parking when route is unset/arrived and the vehicle is stopped.
Since the stop mode needs to work independently of localization, velocity should be taken from vehicle status instead of kinematic state.
