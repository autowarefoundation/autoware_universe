# autoware_pose_initialization_requester

This node calls localization initialize interface when the localization initialization state is uninitialized.
Since the API uses GNSS pose when no pose is specified, initialization using GNSS can be performed automatically.

| Interface    | Local Name | Global Name                            | Description                                |
| ------------ | ---------- | -------------------------------------- | ------------------------------------------ |
| Subscription | -          | /api/localization/initialization_state | The localization initialization state API. |
| Client       | -          | /api/localization/initialize           | The localization initialize API.           |
