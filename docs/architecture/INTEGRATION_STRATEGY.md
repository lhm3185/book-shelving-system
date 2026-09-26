# Navigation, Manipulation, and Isaac Standalone Integration Strategy

## Status and source branches

- Integration base: `refactor/project-layout` (`8d24cc7`)
- Manipulation reference: `origin/work/rnd_0925` (`dafcb0f`)
- The current Nav2 and task-manager flow are authoritative.
- The R&D branch is a source for proven manipulation behavior, not a merge base.

## Architectural decisions

1. `task_manager` owns the system workflow.
2. `shelving_navigation` owns Nav2 communication and mobile-base navigation.
3. `shelving_manipulation` owns all communication with perception for empty-slot and tray-book detection.
4. Isaac Standalone owns physical scene state, tray motion, arm execution, and Stop/Play reset behavior.
5. The main workflow does not restore `DETECT_TARGET_SLOT` to `task_manager`.
6. R&D `nav_manager.py`, `navigation_executor.py`, old maps, and old standalone runner are not imported.
7. Carter and M0609 code and assets are excluded from the target runtime.

## Target runtime flow

```text
TrayJob
  -> task_manager
  -> Nav2: return_station
  -> LoadTray action: Isaac Standalone
  -> Nav2: target shelf
  -> PlaceBook action: manipulation_node
       -> perception: empty-slot scan
       -> perception: tray-book detection
       -> Isaac manipulation command
  -> next book or return_home
```

The `PlaceBook` request carries job, book, shelf, book dimensions, and insertion speed. The target slot and grasp observation remain empty so that manipulation owns perception.

## Target Stop/Play flow

```text
STOP
  -> cancel navigation, tray loading, and manipulation
  -> stop velocity commands
  -> mark the scenario reset-pending

PLAY
  -> restore robot, arm, tray, and books
  -> re-enable required Action Graphs
  -> reset navigation localization and costmaps
  -> reset manipulation and task-manager state
  -> publish READY with a new run identifier
  -> publish a new TrayJob
```

Simulation time should remain monotonic. Reset scene objects and component state without rewinding `/clock` to zero.

## Target file boundaries

```text
isaac_sim/
├── run_simulation.py
├── config/
│   ├── scenes.yaml
│   ├── arm.yaml
│   └── simulation.yaml
├── assets/
│   ├── worlds/
│   ├── robots/
│   ├── props/
│   └── books/
└── lib/
    ├── scenario_runtime.py
    ├── tray_runtime.py
    ├── manipulation_runtime.py
    └── manipulation/
        ├── arm_geometry.py
        ├── arm_planning.py
        └── arm_primitives.py
```

`run_simulation.py` is the only Isaac Standalone entry point. Libraries contain implementation, not additional runners.

## Implementation phases

### Phase 1: interfaces

- Add `shelf_id` to the `PlaceBook` goal.
- Add `LoadTray.action`.
- Add one typed scenario-state interface for Stop/Play coordination.
- Update `shelving_interfaces/CMakeLists.txt` and package dependencies.

### Phase 2: standalone scenario core

- Keep the current composed-stage and Nav2 Action Graph setup.
- Add initial-state capture for robot, arm, tray, and books.
- Add Timeline Stop/Play observation.
- Add idempotent scene reset.
- Add scenario state publication.

### Phase 3: tray loading

- Adapt the R&D group-motion, interpolation, rebase, and follow algorithms.
- Trigger delivery only from `LoadTray`, not at application startup.
- Read the source pose from the active return-machine tray Prim.
- Read the destination from a Ridgeback `tray_mount` Prim or configured relative pose.
- Return success only after the tray is attached to the robot-follow relationship.

### Phase 4: task-manager tray gate

- Preserve `RECEIVE_TRAY`.
- Replace the immediate first-book transition with a `LoadTray` action request.
- Enter `SELECT_BOOK` only after tray loading succeeds.

### Phase 5: ROS manipulation migration

Port from `origin/work/rnd_0925`:

- automatic empty-slot scanning;
- tray-book detection;
- perception request/observation topics;
- cancellation and safe feedback handling;
- error codes and recovery metadata;
- multi-book slot bookkeeping and stale-gap invalidation.

Do not port the R&D task-manager perception state. The current direct `NAV_TO_SHELF -> PLACE_BOOK` transition remains.

### Phase 6: Isaac manipulation migration

Port the verified Franka planning, grasp, insertion, retreat, and verification behavior. Adapt it to the current scene configuration and Prim paths.

Remove during adaptation:

- Carter and M0609 branches;
- old absolute paths and level coordinates;
- R&D navigation execution;
- recording and one-off diagnostic execution paths;
- process-restart assumptions.

### Phase 7: PlaceBook integration

`task_manager` sends:

- `job_id`, `book_id`, and `shelf_id`;
- book dimensions and insertion speed;
- empty `target_slot.header.frame_id`;
- `has_grasp = false`.

`manipulation_node` then owns empty-slot detection, tray-book detection, and Isaac execution.

### Phase 8: reset coordination

- Cancel active actions on Stop.
- Reset physical and ROS component state on the next Play.
- Reinitialize AMCL pose and clear Nav2 costmaps after teleporting the robot.
- Clear manipulation trackers, used tray slots, cached observations, and pending commands.
- Start a new job only after the scenario publishes READY.

### Phase 9: launch and scripts

- Keep `navigation-test` with `mock_manipulation_server`.
- Add `full` mode with real manipulation and perception nodes.
- Keep Isaac Standalone outside ROS launch.
- Extend the root runner to start Standalone, wait for READY, launch ROS, and clean up all processes on exit.

### Phase 10: cleanup after acceptance

Delete only after the integrated runtime passes:

- old Isaac navigation executor and related custom navigation code;
- Carter and M0609 profiles and assets;
- duplicate standalone runners;
- old integrated USD files;
- unused experiment scripts and tools;
- legacy `DetectTargetSlot` compatibility path if no remaining caller exists.

## Acceptance session

One Isaac session is sufficient:

1. Complete `Nav2 -> LoadTray -> PlaceBook -> return_home`.
2. Press Stop and then Play without restarting Isaac Sim.
3. Complete the same workflow again from the initial state.
4. Confirm that no stale action, localization, tray-slot, or book-placement state remains.
