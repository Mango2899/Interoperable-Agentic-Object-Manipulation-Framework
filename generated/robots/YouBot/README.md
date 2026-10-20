# Generated Robot: YouBot

This folder was created by the phase-one WoMAT robot generator.

## Source Input

- URDF: /home/maxchrs/Desktop/MA/WoMAT/external/RobWoT/Assets/URDFs/youbot_description/urdf/youbot.urdf

## Mobile Platform

The URDF was classified as a mobile platform (`omnidirectional`):

- Wheel joints: caster_joint_fl, wheel_joint_fl, caster_joint_fr, wheel_joint_fr, caster_joint_bl, wheel_joint_bl, caster_joint_br, wheel_joint_br
- Arm joints: arm_joint_1, arm_joint_2, arm_joint_3, arm_joint_4, arm_joint_5
- Gripper joints: gripper_finger_joint_l, gripper_finger_joint_r
- Base link: base_footprint, arm root: arm_link_0, tip: arm_link_5

The classification is stored in `manifest.json` under `mobility` and can be
overridden with `--wheel-joints`, `--arm-joints`, `--gripper-joints`, and
`--platform`.

Planned lower-level Thing affordances for mobile platforms (later phases):
`moveTo {x, y, theta(deg)}` for the base, `moveArmTo {x, y, z}` in the base
frame, `pick`/`drop` composites, and a `getBasePose` property. The agent
scaffold (TD, .asl, AgentThing) already targets these names. Note that
`robot:pipeline` and `robot:serve` do not handle mobile platforms yet; the
CoppeliaSim steps below will treat the robot as a fixed manipulator until the
bridge is extended.

## Generated Runtime Shape

- Lower-level robot Thing port: 8094
- AgentThing TD port: 8095
- Agent connector websocket port: 8195
- AgentSpeak file: src/agt/generated/youBot_agent.asl
- AgentThing file: src/WoT/generated/YouBot_agent_thing.ts
- Agent TD: src/td/generated/YouBot.agent.td.json
- JaCaMo file: generated/robots/YouBot/YouBot.jcm
- Config fragment: generated/robots/YouBot/config.fragment.json

## Phase-Two Pipeline

Check the generated pipeline paths without CoppeliaSim:

```bash
npm run robot:pipeline -- --manifest generated/robots/YouBot/manifest.json --dry-run
```

With CoppeliaSim running and the websocket remote API available on localhost:23050, generate the CoppeliaSim model:

```bash
npm run robot:pipeline -- --manifest generated/robots/YouBot/manifest.json --generate-model
```

Then generate workspace data and the lower-level robot TD:

```bash
npm run robot:pipeline -- --manifest generated/robots/YouBot/manifest.json --generate-td
```

Or run both CoppeliaSim-dependent steps together:

```bash
npm run robot:pipeline -- --manifest generated/robots/YouBot/manifest.json --full-pipeline
```

If the command says it cannot connect to `localhost:23050`, start CoppeliaSim and make sure its websocket remote API is available. If it says `simURDF.import` is missing, enable/install the CoppeliaSim URDF plugin and restart CoppeliaSim.

Expected generated artifacts:

- generated/robots/YouBot/model/YouBot.ttm
- generated/robots/YouBot/scene/YouBot_workspace.ttt
- generated/robots/YouBot/info/YouBot_info.json
- generated/robots/YouBot/info/YouBot_shape.stl
- generated/robots/YouBot/info/YouBot_data_point.csv
- generated/robots/YouBot/td/YouBot.lower.td.json

## Lower Thing Runtime

After phase two artifacts exist, expose the generated lower-level robot Thing:

```bash
npm run robot:serve -- --manifest generated/robots/YouBot/manifest.json
```

The workspace scene includes visuals for the reachable workspace (white point cloud plus a transparent convex hull) and collision regions (red point clouds). To spare rendering resources when many robots are spawned, set `runtime.workspaceVisuals` to `false` in the manifest and restart the server; the visuals are then removed from the scene at load time. No pipeline re-run is needed to toggle this.

In another terminal, expose the generated robot AgentThing bridge. This is what the JaCaMo `wotCommunication` artifact connects to over websocket:

```bash
npm run robot:agent -- --manifest generated/robots/YouBot/manifest.json
```

In another terminal, check the TD:

```bash
curl http://localhost:8094/coppeliasim_virtualrobot_YouBot
```

Phase 5 motion test:

```bash
curl -X POST http://localhost:8094/coppeliasim_virtualrobot_youbot/actions/moveTocartesianPosition \
  -H 'Content-Type: application/json' \
  -d '{"x":0.1,"y":0.1,"z":0.4}'
```

Phase 5 joint motion test:

```bash
curl -X POST http://localhost:8094/coppeliasim_virtualrobot_youbot/actions/moveTojointPosition \
  -H 'Content-Type: application/json' \
  -d '{"joint1":10,"joint2":-20,"joint3":15,"joint4":0,"joint5":0,"joint6":0}'
```

The runtime applies smooth interpolation and workspace prechecks. Motion settings are configurable in `manifest.json` via `runtime.jointSteps`, `runtime.cartesianSteps`, `runtime.stepDelayMs`, `runtime.cartesianMaxVelocity`, and `runtime.jointMaxVelocityDeg`.

Low-level collision checking is configurable in `manifest.json` via `runtime.enableCollisionCheck`. Joint-space preflight is configurable via `runtime.enableJointCollisionPreflight`. By default generated robots check the configured wall obstacle `80cmHighWall50cm`.

Phase 6 wall collision test:

```bash
curl -X POST http://localhost:8094/coppeliasim_virtualrobot_youbot/actions/moveTocartesianPosition \
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
./gradlew -Pjcm=generated/robots/YouBot/YouBot.jcm
```
