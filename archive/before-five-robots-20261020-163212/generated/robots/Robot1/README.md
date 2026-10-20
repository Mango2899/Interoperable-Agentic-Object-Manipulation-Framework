# Generated Robot: Robot1

This folder was created by the phase-one WoMAT robot generator.

## Source Input

- URDF: /home/maxchrs/Desktop/MA/WoMAT/external/RobWoT/Assets/URDFs/ur_description/urdf/ur10_robot.urdf

## Generated Runtime Shape

- Lower-level robot Thing port: 8110
- AgentThing TD port: 8111
- Agent connector websocket port: 8211
- AgentSpeak file: src/agt/generated/robot1_agent.asl
- AgentThing file: src/WoT/generated/Robot1_agent_thing.ts
- Agent TD: src/td/generated/Robot1.agent.td.json
- JaCaMo file: generated/robots/Robot1/Robot1.jcm
- Config fragment: generated/robots/Robot1/config.fragment.json

## Phase-Two Pipeline

Check the generated pipeline paths without CoppeliaSim:

```bash
npm run robot:pipeline -- --manifest generated/robots/Robot1/manifest.json --dry-run
```

With CoppeliaSim running and the websocket remote API available on localhost:23050, generate the CoppeliaSim model:

```bash
npm run robot:pipeline -- --manifest generated/robots/Robot1/manifest.json --generate-model
```

Then generate workspace data and the lower-level robot TD:

```bash
npm run robot:pipeline -- --manifest generated/robots/Robot1/manifest.json --generate-td
```

Or run both CoppeliaSim-dependent steps together:

```bash
npm run robot:pipeline -- --manifest generated/robots/Robot1/manifest.json --full-pipeline
```

If the command says it cannot connect to `localhost:23050`, start CoppeliaSim and make sure its websocket remote API is available. If it says `simURDF.import` is missing, enable/install the CoppeliaSim URDF plugin and restart CoppeliaSim.

Expected generated artifacts:

- generated/robots/Robot1/model/Robot1.ttm
- generated/robots/Robot1/scene/Robot1_workspace.ttt
- generated/robots/Robot1/info/Robot1_info.json
- generated/robots/Robot1/info/Robot1_shape.stl
- generated/robots/Robot1/info/Robot1_data_point.csv
- generated/robots/Robot1/td/Robot1.lower.td.json

## Lower Thing Runtime

After phase two artifacts exist, expose the generated lower-level robot Thing:

```bash
npm run robot:serve -- --manifest generated/robots/Robot1/manifest.json
```

The workspace scene includes visuals for the reachable workspace (white point cloud plus a transparent convex hull) and collision regions (red point clouds). To spare rendering resources when many robots are spawned, set `runtime.workspaceVisuals` to `false` in the manifest and restart the server; the visuals are then removed from the scene at load time. No pipeline re-run is needed to toggle this.

In another terminal, expose the generated robot AgentThing bridge. This is what the JaCaMo `wotCommunication` artifact connects to over websocket:

```bash
npm run robot:agent -- --manifest generated/robots/Robot1/manifest.json
```

In another terminal, check the TD:

```bash
curl http://localhost:8110/coppeliasim_virtualrobot_Robot1
```

Phase 5 motion test:

```bash
curl -X POST http://localhost:8110/coppeliasim_virtualrobot_robot1/actions/moveTocartesianPosition \
  -H 'Content-Type: application/json' \
  -d '{"x":0.1,"y":0.1,"z":0.4}'
```

Phase 5 joint motion test:

```bash
curl -X POST http://localhost:8110/coppeliasim_virtualrobot_robot1/actions/moveTojointPosition \
  -H 'Content-Type: application/json' \
  -d '{"joint1":10,"joint2":-20,"joint3":15,"joint4":0,"joint5":0,"joint6":0}'
```

The runtime applies smooth interpolation and workspace prechecks. Motion settings are configurable in `manifest.json` via `runtime.jointSteps`, `runtime.cartesianSteps`, `runtime.stepDelayMs`, `runtime.cartesianMaxVelocity`, and `runtime.jointMaxVelocityDeg`.

Low-level collision checking is configurable in `manifest.json` via `runtime.enableCollisionCheck`. Joint-space preflight is configurable via `runtime.enableJointCollisionPreflight`. By default generated robots check the configured wall obstacle `80cmHighWall50cm`.

Phase 6 wall collision test:

```bash
curl -X POST http://localhost:8110/coppeliasim_virtualrobot_robot1/actions/moveTocartesianPosition \
  -H 'Content-Type: application/json' \
  -d '{"x":-0.25,"y":-0.2,"z":0.3}'
```

With `runtime.enableCollisionCheck` set to `true`, a wall-crossing path should return `path_collision_detected` before movement starts.

## Agent Check

The JaCaMo side is scaffolded with a generated horizontal demo plan:

- `!horizontalDemo` moves to `horizontalDemoPosition(X,Y,Z)`.
- It then calls `!goHome`.
- If `homeJoints(array(...))` is present in the generated `.asl`, `!goHome` uses joint-space home motion for an upright pose.
- Otherwise `!goHome` uses the generated cartesian `homePosition(X,Y,Z)`.

Run it after both `robot:serve` and `robot:agent` are running:

```bash
./gradlew -Pjcm=generated/robots/Robot1/Robot1.jcm
```
