# Diagnostics

## /adapi/node/localization: state

The level is OK when the localization state is INITIALIZED. Otherwise ERROR.

## /adapi/node/routing: state

The level is OK when the routing state is SET, REROUTING, or ARRIVED. Otherwise ERROR.

## /adapi/node/mrm_request: delegate

The level is OK when the delegate strategy is not requested. Otherwise ERROR.
The senders of the delegate strategy are set to keys.
