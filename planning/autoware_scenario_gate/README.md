## autoware_scenario_gate
# scenario_gate
Scenario Gate: multiplex trajectories according to the current scenario decided by selector.

## Switch Selector
- Edit `config/scenario_gate.with_selector.yaml` → set `selector_key: Default` or `selector_key: Extra`.
- The map in C++ expands the short key to the full plugin class name automatically.

## Data Flow
```text
+------------------+       +-----------------+       +----------------+
| Scenario Selector| --->  |  Scenario Gate  | --->  | Output Topics  |
| (Default/Extra)  |       |                 |       | trajectory &   |
| publishes /scenario ---->| subscribes      |       | current_scenario |
+------------------+       +-----------------+       +----------------+