# Parameters

The default parameters are set by [default_adapi.param.yaml](../config/default_adapi.param.yaml).

## /adapi/node/autoware_state

| Parameter   | Type   | Description                    |
| ----------- | ------ | ------------------------------ |
| update_rate | double | Update rate of autoware state. |

## /adapi/node/motion

| Parameter            | Type   | Description                                |
| -------------------- | ------ | ------------------------------------------ |
| require_accept_start | bool   | Require accept start API at start if true, |
| stop_check_duration  | double | Stop check duration of the vehicle.        |

## /adapi/node/routing

| Parameter           | Type   | Description                         |
| ------------------- | ------ | ----------------------------------- |
| stop_check_duration | double | Stop check duration of the vehicle. |

## /adapi/node/vehicle_door

| Parameter              | Type | Description                                               |
| ---------------------- | ---- | --------------------------------------------------------- |
| check_autoware_control | bool | Always allow door control during direct control if false. |

## /adapi/node/manual/local

| Parameter | Type   | Description                                                        |
| --------- | ------ | ------------------------------------------------------------------ |
| mode      | string | Input source of manual control. Specify "local" for local control. |

## /adapi/node/manual/remote

| Parameter | Type   | Description                                                          |
| --------- | ------ | -------------------------------------------------------------------- |
| mode      | string | Input source of manual control. Specify "remote" for remote control. |
