# WoMAT RobWoT URDF Integration Report

Date: 2026-06-02

This report summarizes the work done to extend WoMAT so that a robot can be generated from a URDF input, connected to RobWoT/CoppeliaSim artifacts, exposed as WoT Things, and controlled through the existing agent-oriented WoMAT stack.

The work was implemented in six phases:

1. Phase 1: WoMAT robot scaffolding from URDF input.
2. Phase 2: RobWoT bridge for CoppeliaSim model and workspace generation.
3. Phase 3: Runtime lower-level Thing.
4. Phase 4: AgentThing and Gradle/JaCaMo integration.
5. Phase 5: Smooth motion and workspace safety.
6. Phase 6: Wall-aware collision detection and prevention.

The current generated test robot is `myRobot`, based on:

```text
/home/maxchrs/Desktop/MA/RobWoT/Assets/URDFs/ur_description/urdf/ur3_robot.urdf
```

The generated robot manifest is:

```text
generated/robots/myRobot/manifest.json
```

## Current Commands

Generate WoMAT robot scaffolding from a URDF:

```bash
npm run generate:robot -- --urdf /path/to/robot.urdf --name myRobot --thing-port 8110 --agent-port 8111 --websocket-port 8211
```

Run the RobWoT/CoppeliaSim bridge:

```bash
npm run robot:pipeline -- --manifest generated/robots/myRobot/manifest.json --dry-run
npm run robot:pipeline -- --manifest generated/robots/myRobot/manifest.json --generate-model
npm run robot:pipeline -- --manifest generated/robots/myRobot/manifest.json --generate-td
npm run robot:pipeline -- --manifest generated/robots/myRobot/manifest.json --full-pipeline
```

Serve the lower-level generated robot Thing:

```bash
npm run robot:serve -- --manifest generated/robots/myRobot/manifest.json
```

Run the generated AgentThing:

```bash
npx ts-node --transpile-only src/WoT/generated/myRobot_agent_thing.ts
```

Run the generated JaCaMo agent configuration:

```bash
./gradlew -Pjcm=generated/robots/myRobot/myRobot.jcm
```

Delete one generated robot and its `config.json` entry:

```bash
npm run delete:robot -- --name myRobot
```

Preview deletion first:

```bash
npm run delete:robot -- --name myRobot --dry-run
```

Clone with submodules:

```bash
git clone --recurse-submodules <repo-url>
```

If the repository was already cloned:

```bash
git submodule update --init --recursive
```

Install RobWoT's Node dependencies after initializing submodules:

```bash
cd external/RobWoT
npm install
```

The WoMAT generator now uses `external/RobWoT` by default when that submodule exists. A custom RobWoT location can still be passed with:

```bash
npm run generate:robot -- --urdf <file.urdf> --name <robotName> --robwot-root /path/to/RobWoT
```

## Phase 1: WoMAT Scaffolding

### Goal

The first phase created a generator that accepts a URDF file path and a robot name, then produces the WoMAT-side scaffolding needed for the robot.

### Implementation

The new generator is:

```text
src/generator/generateRobot.js
```

The npm script is:

```json
"generate:robot": "node ./src/generator/generateRobot.js"
```

For a generated robot, the generator creates:

```text
generated/robots/<name>/manifest.json
generated/robots/<name>/README.md
generated/robots/<name>/config.fragment.json
generated/robots/<name>/<name>.jcm
src/agt/generated/<name>_agent.asl
src/WoT/generated/<name>_agent_thing.ts
src/WoT/generated/<name>_lower_thing.ts
src/td/generated/<name>.agent.td.json
```

It also updates the root `config.json` unless `--skip-config-update` is passed.

The generated lower-layer URL uses the node-wot slug form:

```text
http://localhost:<thingPort>/coppeliasim_virtualrobot_<lowercaseName>
```

For `myRobot`, this is:

```text
http://localhost:8110/coppeliasim_virtualrobot_myrobot
```

### Important Fixes

The generated TypeScript helper files were adjusted with `export {};` so that repeated generated files do not collide through global `main()` declarations.

The generated Agent TD schema was fixed so that action input arrays use `oneOf` item schemas instead of invalid mixed JSON schema type arrays.

### Phase 1 Test

```bash
npm run generate:robot -- --urdf /home/maxchrs/Desktop/MA/RobWoT/Assets/URDFs/ur_description/urdf/ur3_robot.urdf --name myRobot --thing-port 8110 --agent-port 8111 --websocket-port 8211
```

Expected result:

- Robot manifest exists.
- Generated AgentSpeak exists.
- Generated AgentThing TypeScript exists.
- Generated lower Thing placeholder exists.
- Generated `.jcm` exists.
- Root config contains the generated robot entry.

## Phase 2: RobWoT/CoppeliaSim Bridge

### Goal

The second phase connected WoMAT generation to useful RobWoT functionality:

- Import URDF into CoppeliaSim.
- Save a generated CoppeliaSim model.
- Generate workspace data.
- Generate lower-level TD and robot information artifacts.
- Keep the result described in the generated manifest.

### Implementation

The new bridge is:

```text
src/generator/robwotBridge.js
```

The npm script is:

```json
"robot:pipeline": "node ./src/generator/robwotBridge.js"
```

The bridge uses the RobWoT project at:

```text
/home/maxchrs/Desktop/MA/RobWoT
```

It uses RobWoT/CoppeliaSim inputs such as:

```text
/home/maxchrs/Desktop/MA/RobWoT/Coppeliasim scene/robot_urdf_load_scene.ttt
/home/maxchrs/Desktop/MA/RobWoT/Coppeliasim scene/robot_virtual_workspace.ttt
/home/maxchrs/Desktop/MA/RobWoT/Library/GenerateRobotDescription/robot_driver_workspace.lua
/home/maxchrs/Desktop/MA/RobWoT/Library/GenerateRobotDescription/robot_template.json
```

Generated artifacts include:

```text
generated/robots/myRobot/model/myRobot.ttm
generated/robots/myRobot/scene/myRobot_workspace.ttt
generated/robots/myRobot/info/myRobot_info.json
generated/robots/myRobot/info/myRobot_shape.stl
generated/robots/myRobot/info/myRobot_data_point.csv
generated/robots/myRobot/td/myRobot.lower.td.json
```

### URDF Import

The bridge imports the URDF through CoppeliaSim's `simURDF` plugin. Direct websocket access did not expose `simURDF.import`, so the solution was to execute the import inside a CoppeliaSim script sandbox with `sim.executeScriptString`.

The bridge also passes a package replacement path so that URDF references using `package://` can resolve meshes correctly.

### Workspace and Point Clouds

The workspace generator samples random robot configurations and records cartesian positions. The CSV uses:

```text
Collision_state=1
```

for reachable, non-colliding samples, and:

```text
Collision_state=0
```

for samples that collide with configured obstacles.

In CoppeliaSim:

- `pointCloud[0]` contains reachable, non-colliding workspace samples.
- `pointCloud[1]` contains colliding workspace samples.

The point cloud density is controlled by:

```json
"workspaceRandomSteps": 80000
```

inside:

```text
generated/robots/myRobot/manifest.json
```

### Obstacle Support

The wall obstacle is configured as:

```json
"collisionObstacles": [
  "80cmHighWall50cm"
]
```

The bridge checks sampled tool points against the local bounding box of this CoppeliaSim object. This is implemented as a point-vs-obstacle AABB test in CoppeliaSim Lua, using each obstacle's object matrix and bounding box parameters.

### Phase 2 Test

With CoppeliaSim running and websocket remote API listening on `localhost:23050`:

```bash
npm run robot:pipeline -- --manifest generated/robots/myRobot/manifest.json --full-pipeline
```

To regenerate only the TD/workspace artifacts after changing `workspaceRandomSteps`:

```bash
npm run robot:pipeline -- --manifest generated/robots/myRobot/manifest.json --generate-td
```

## Phase 3: Lower-Level Runtime Thing

### Goal

The third phase made the generated lower-level TD usable at runtime. Instead of only generating static files, WoMAT now has a runtime server that loads the generated CoppeliaSim scene and exposes the robot as a node-wot Thing.

### Implementation

The runtime server is:

```text
src/generator/lowerThingServer.js
```

The npm script is:

```json
"robot:serve": "node ./src/generator/lowerThingServer.js"
```

The runtime server:

- Connects to CoppeliaSim websocket remote API on `localhost:23050`.
- Loads the generated workspace scene.
- Injects/adapts the RobWoT robot runtime driver as a customization script.
- Exposes the generated lower-level Thing through node-wot.
- Reads and writes robot state through CoppeliaSim script calls.

The lower-level Thing for `myRobot` is exposed at:

```text
http://localhost:8110/coppeliasim_virtualrobot_myrobot
```

### Runtime Endpoints

Read cartesian position:

```bash
curl http://localhost:8110/coppeliasim_virtualrobot_myrobot/properties/getCartesianposition
```

Read joint position:

```bash
curl http://localhost:8110/coppeliasim_virtualrobot_myrobot/properties/getJointposition
```

Move cartesian:

```bash
curl -X POST http://localhost:8110/coppeliasim_virtualrobot_myrobot/actions/moveTocartesianPosition \
  -H 'Content-Type: application/json' \
  -d '{"x":0.1,"y":0.1,"z":0.4}'
```

Move joints:

```bash
curl -X POST http://localhost:8110/coppeliasim_virtualrobot_myrobot/actions/moveTojointPosition \
  -H 'Content-Type: application/json' \
  -d '{"joint1":10,"joint2":-20,"joint3":15,"joint4":0,"joint5":0,"joint6":0}'
```

### Phase 3 Test

1. Start CoppeliaSim and confirm websocket remote API is listening:

```bash
lsof -iTCP:23050 -sTCP:LISTEN -n -P
```

2. Start the lower Thing:

```bash
npm run robot:serve -- --manifest generated/robots/myRobot/manifest.json
```

3. Query the TD:

```bash
curl http://localhost:8110/coppeliasim_virtualrobot_myrobot
```

4. Query properties and execute actions with `curl`.

## Phase 4: AgentThing and Gradle/JaCaMo Integration

### Goal

The fourth phase connected the generated robot to the agent layer. The goal was that the generated JaCaMo configuration and generated AgentThing should work with the normal Gradle workflow.

### Implementation

The generated AgentThing is:

```text
src/WoT/generated/myRobot_agent_thing.ts
```

It exposes:

```text
http://localhost:8111/myrobot_agent
```

The generated AgentSpeak file is:

```text
src/agt/generated/myRobot_agent.asl
```

The generated JaCaMo configuration is:

```text
generated/robots/myRobot/myRobot.jcm
```

The generated `.jcm` config creates a robot agent and focuses it on the existing WoMAT `wotCommunication.wotCommunicator` artifact:

```jcm
mas myRobotMas {
    agent myRobotAgent: generated/myRobot_agent.asl {
        focus: myRobot.wotComm
    }

    workspace myRobot {
        artifact wotComm: wotCommunication.wotCommunicator("myRobot")
    }
}
```

The generated AgentThing connects to the lower-level Thing through:

```text
http://localhost:8110/coppeliasim_virtualrobot_myrobot
```

When the agent receives `moveTo` or `goHome`, the AgentThing forwards the command to the lower-level Thing action `moveTocartesianPosition`.

### Phase 4 Test

Start the lower Thing:

```bash
npm run robot:serve -- --manifest generated/robots/myRobot/manifest.json
```

Start the generated AgentThing:

```bash
npx ts-node --transpile-only src/WoT/generated/myRobot_agent_thing.ts
```

Run the generated JaCaMo application:

```bash
./gradlew -Pjcm=generated/robots/myRobot/myRobot.jcm
```

Expected result:

- Gradle recognizes the generated `.jcm`.
- The generated agent starts.
- The generated AgentThing can connect to the lower-level Thing.
- Agent-level movement requests can be forwarded to the lower-level robot TD.

## Phase 5: Smooth Motion and Workspace Safety

### Goal

The fifth phase improved runtime motion and added workspace safety behavior.

Earlier, cartesian movement was a rapid drop/teleport toward the target. We changed this into interpolated motion and then added workspace and obstacle checks.

### Smooth Motion

The runtime now interpolates movement:

```json
"runtime": {
  "jointSteps": 60,
  "cartesianSteps": 60,
  "stepDelayMs": 25,
  "cartesianMaxVelocity": 0.45,
  "jointMaxVelocityDeg": 180
}
```

Cartesian motion interpolates from the current end-effector position to the target over `cartesianSteps`.

Joint motion interpolates from current joint angles to target joint angles over `jointSteps`.

After the phase 5 refinement, `cartesianSteps` and `jointSteps` are minimum step counts. The runtime also computes distance-aware step counts from `cartesianMaxVelocity`, `jointMaxVelocityDeg`, and `stepDelayMs`. This keeps short movements smooth and prevents longer movements from becoming large jumps.

Successful movement responses include the actual number of interpolation steps used:

```json
{
  "success": true,
  "state": "cartesian_motion_complete",
  "motion": {
    "steps": 60
  }
}
```

### Workspace Safety

The runtime reads:

```text
generated/robots/myRobot/info/myRobot_data_point.csv
```

and checks that cartesian targets are:

- Inside the sampled workspace bounds.
- Near reachable workspace samples.

The tolerance is configured as:

```json
"workspaceTolerance": 0.08
```

## Phase 6: Collision Detection and Prevention

### Goal

The sixth phase added obstacle-aware collision detection and, more importantly, collision prevention.

The concrete test obstacle is:

```text
80cmHighWall50cm
```

The goal was not only to detect that a wall collision would happen, but to reject unsafe cartesian movement before the robot starts moving.

### Workspace Collision Classification

During workspace generation, samples that intersect the wall are written with:

```text
Collision_state=0
```

Those samples appear in `pointCloud[1]`.

Reachable non-colliding samples use:

```text
Collision_state=1
```

Those samples appear in `pointCloud[0]`.

### Runtime Collision Prevention

The final implementation prevents cartesian moves into the wall by preflighting the whole cartesian path before movement begins.

The important distinction is:

- Reactive checking only detects collision during movement.
- Preflight checking samples the planned path first and rejects unsafe paths before sending movement commands.

The cartesian path preflight returns:

```json
{
  "success": false,
  "reason": "path_collision_detected",
  "obstacleMode": "tip_vs_obstacle_aabb"
}
```

This prevents the robot from moving into the wall.

### Runtime Joint-Space Collision Prevention

The collision layer now also protects separate joint movements. This closes the earlier gap where a direct `moveTojointPosition` action could rotate the arm body into the wall even when cartesian tool-point moves were protected.

The runtime injects a CoppeliaSim Lua function:

```lua
function jointConfigCollisionCheck(targetConf)
    local currentConf={}
    for i=1,#jointHandles,1 do
        currentConf[i]=sim.getJointPosition(jointHandles[i])
        sim.setJointPosition(jointHandles[i], targetConf[i])
    end

    local colliding=womatRobotObstacleCollision()

    for i=1,#jointHandles,1 do
        sim.setJointPosition(jointHandles[i], currentConf[i])
    end

    return colliding
end
```

The important detail is that the candidate joint configuration is applied only inside the collision probe, then the previous joint state is restored before the function returns.

The robot-vs-obstacle check uses CoppeliaSim collections:

```lua
local robotCollection=sim.createCollection(0)
sim.addItemToCollection(robotCollection, sim.handle_tree, simBase, 0)
local colliding=sim.checkCollision(robotCollection, obstacleCollection)~=0
```

On the JavaScript side, `smoothJointMove(...)` now calls `preflightJointPath(...)` before executing the actual joint interpolation. If any planned joint step collides with the configured obstacle collection, the runtime returns:

```json
{
  "success": false,
  "reason": "joint_path_collision_detected",
  "obstacleMode": "robot_vs_obstacle_collection"
}
```

and no real joint movement command is sent.

### Phase 6 Collision Test

Start the lower Thing:

```bash
npm run robot:serve -- --manifest generated/robots/myRobot/manifest.json
```

Read the cartesian position:

```bash
curl http://localhost:8110/coppeliasim_virtualrobot_myrobot/properties/getCartesianposition
```

Send a wall-colliding target:

```bash
curl -X POST http://localhost:8110/coppeliasim_virtualrobot_myrobot/actions/moveTocartesianPosition \
  -H 'Content-Type: application/json' \
  -d '{"x":-0.25,"y":-0.2,"z":0.3}'
```

Expected result:

```json
{
  "success": false,
  "reason": "path_collision_detected",
  "obstacleMode": "tip_vs_obstacle_aabb"
}
```

Read the cartesian position again. It should be unchanged.

Latest verification:

```json
{
  "success": false,
  "reason": "path_collision_detected",
  "obstacleMode": "tip_vs_obstacle_aabb",
  "blockedAtStep": 51
}
```

The cartesian position before and after the blocked wall command was identical:

```json
{"x":0.10312851024545,"y":0.10178568841193,"z":0.39946089515415}
```

Send a reachable target:

```bash
curl -X POST http://localhost:8110/coppeliasim_virtualrobot_myrobot/actions/moveTocartesianPosition \
  -H 'Content-Type: application/json' \
  -d '{"x":0.1,"y":0.1,"z":0.4}'
```

Expected result:

```json
{
  "success": true,
  "state": "cartesian_motion_complete"
}
```

### Phase 6 Joint Collision Test

For R2D2, this joint target was found to collide with the wall:

```bash
curl -X POST http://localhost:8120/coppeliasim_virtualrobot_r2d2/actions/moveTojointPosition \
  -H 'Content-Type: application/json' \
  -d '{"joint1":-48.03,"joint2":12.82,"joint3":117.06,"joint4":35.48,"joint5":-10.43,"joint6":76.3}'
```

Expected result:

```json
{
  "success": false,
  "reason": "joint_path_collision_detected",
  "obstacleMode": "robot_vs_obstacle_collection"
}
```

Latest verification:

```json
{
  "success": false,
  "reason": "joint_path_collision_detected",
  "blockedAtStep": 12
}
```

The joint readback after rejection stayed at all zeros:

```json
{"joint1":0,"joint2":0,"joint3":0,"joint4":0,"joint5":0,"joint6":0}
```

## RobWoT Comparison

This section compares the original RobWoT behavior with the WoMAT integration built in this work. The main design decision is that RobWoT remains the source for CoppeliaSim/URDF/workspace concepts, while WoMAT adds reproducible generation, runtime Thing exposure, and explicit safety preflight before movement.

### Motion Control

RobWoT executes motion inside CoppeliaSim with a script state machine and coroutines. In:

```text
external/RobWoT/Library/Robot_WoT_Server/robot_driver.lua
```

cartesian and joint movement are selected by `rState`:

```lua
rState = 0 -- initial
rState = 1 -- cartesian move
rState = 2 -- joint move
rState = 3 -- stopped
```

`sysCall_actuation()` resumes the corresponding coroutine:

```lua
elseif rState==1 then
    coroutine.resume(corout_move, finalPos)
elseif rState==2 then
    coroutine.resume(corout_moveJoint, var1, var2, var3, var4, var5)
end
```

For cartesian motion, RobWoT uses CoppeliaSim's built-in pose motion:

```lua
sim.moveToPose(-1,currentPose,maxVelocity,maxAcceleration,maxJerk,targetPose,moveToPoseCallback,auxData,nil)
```

For joint motion, RobWoT uses:

```lua
sim.moveToConfig(-1,currentConf,nil,nil,maxVel,maxAccel,maxJerk,targetConf,nil,movCallback,handles)
```

WoMAT changes the control model. In:

```text
src/generator/lowerThingServer.js
```

the RobWoT asynchronous movement functions are renamed, and synchronous helpers are injected:

```js
.replace("function sysCall_actuation()", "function sysCall_actuation_disabled()")
.replace("function moveTojoint(targetConf)", "function moveTojoint_async(targetConf)")
.replace("function moveToPosition(targetPosition)", "function moveToPosition_async(targetPosition)")
```

WoMAT then drives motion from the node-wot HTTP action handler:

```js
thing.setActionHandler("moveTojointPosition", async (params) => {
  const payload = await params.value();
  const radians = jointPayloadToRadians(payload, td);
  return smoothJointMove(sim, scriptHandle, td, radians, options);
});
```

The important change is that movement is not just started and left to CoppeliaSim. WoMAT computes the planned interpolation path first:

```js
const steps = jointStepCount(start, targetRadians, options);
const pathSafety = await preflightJointPath(sim, scriptHandle, start, targetRadians, options, steps);
if (!pathSafety.ok) {
  return { success: false, ...pathSafety, motion: { steps } };
}
```

Then it applies one step at a time:

```js
for (let step = 1; step <= steps; step += 1) {
  const next = interpolate(start, targetRadians, steps, step);
  await executeInScript(sim, scriptHandle, `return moveTojoint(${luaArray(next)})`);
  await delay(options.stepDelayMs);
}
```

What was taken from RobWoT:

- IK target/tip structure.
- Separate cartesian and joint movement concepts.
- Default velocity scale, such as `0.45 m/s` cartesian and `180 deg/s` joint.

What WoMAT adds:

- HTTP action lifecycle around every movement.
- Structured success/failure responses.
- Workspace checks before cartesian motion.
- Collision preflight before cartesian and joint motion.
- `motion.steps` reporting in the action response.

### Workspace Point Sphere / Point Cloud Generation

RobWoT's workspace generation is implemented in:

```text
external/RobWoT/Library/GenerateRobotDescription/robot_driver_workspace.lua
```

The original driver samples random joint configurations, records the tip position, and stores the result in two Lua arrays:

```lua
local points={}
local collisionpoints={}
```

For non-colliding points:

```lua
points[#points+1]=matrix[4]
points[#points+1]=matrix[8]
points[#points+1]=matrix[12]
```

For visualization, RobWoT creates point clouds:

```lua
local ptcld=sim.createPointCloud(0.05,1,0,2)
sim.insertPointsIntoPointCloud(ptcld,0,points)

local collPtcld=sim.createPointCloud(0.05,1,0,2)
sim.insertPointsIntoPointCloud(collPtcld,0,collisionpoints,{255,0,0})
```

It also writes a CSV:

```text
X,Y,Z,Collision_state
```

WoMAT keeps that output format because it is useful for TD bounds, runtime workspace checks, and later inspection:

```text
generated/robots/<name>/info/<name>_data_point.csv
```

WoMAT changes how the RobWoT workspace script is prepared. In:

```text
src/generator/robwotBridge.js:prepareWorkspaceDriverScript(ctx)
```

the sample count is controlled by the generated manifest:

```js
const randomSteps = Number(ctx.manifest.pipeline?.workspaceRandomSteps || 80000);
```

and injected into the RobWoT Lua script:

```js
.replace("randomSteps=80000", `randomSteps=${randomSteps}`)
```

WoMAT also patches the collision classification line. RobWoT originally uses broad scene collision:

```lua
if sim.checkCollision(robotCollection,sim.handle_all)~=0 then
    colliding=true
end
```

WoMAT replaces that with configured obstacle classification:

```js
.split("if sim.checkCollision(robotCollection,sim.handle_all)~=0 then")
.join("if womatPointObstacleCollision(matrix[4], matrix[8], matrix[12]) then")
```

This makes `Collision_state=0` mean "the sampled tool point intersects a configured obstacle such as `80cmHighWall50cm`", instead of "the robot collided with anything in the whole scene."

The resulting meaning is:

```text
Collision_state=1 -> reachable non-colliding workspace sample
Collision_state=0 -> configured-obstacle workspace sample
```

So the point clouds/sphere are still RobWoT-style workspace visualization, but the classification is now controlled by WoMAT's manifest and obstacle list.

### Collision Detection

RobWoT has a generic collision check in its runtime driver:

```lua
function collisionCheck()
    local robotCollection=sim.createCollection(0)
    local base=sim.getObject('.')
    sim.addItemToCollection(robotCollection,sim.handle_tree,base,0)
    return sim.checkCollision(robotCollection,sim.handle_all)~=0
end
```

This answers "is the robot colliding with anything?" but it is reactive and broad. It does not by itself prevent an unsafe HTTP action before the robot starts moving.

WoMAT adds explicit obstacle-aware checks. For workspace generation and cartesian runtime preflight, it injects:

```lua
function womatPointObstacleCollision(x, y, z)
    local obstacle=sim.getObject('/80cmHighWall50cm', {noError=true})
    local m=sim.getObjectMatrix(obstacle, sim.handle_world)
    local inv=sim.getMatrixInverse(m)
    local p=sim.multiplyVector(inv, {x, y, z})
    return p[1] >= minX and p[1] <= maxX
       and p[2] >= minY and p[2] <= maxY
       and p[3] >= minZ and p[3] <= maxZ
end
```

The key idea is that a world-space point is transformed into the obstacle's local coordinate frame, then checked against the obstacle's local bounding box.

For cartesian actions, WoMAT checks the full planned tip path before motion:

```js
const pathSafety = await preflightCartesianPath(sim, scriptHandle, start, point, options, steps);
if (!pathSafety.ok) {
  return { success: false, ...pathSafety, motion: { steps } };
}
```

For joint actions, WoMAT now checks planned joint configurations against robot-vs-obstacle collections:

```lua
function jointConfigCollisionCheck(targetConf)
    local currentConf={}
    for i=1,#jointHandles,1 do
        currentConf[i]=sim.getJointPosition(jointHandles[i])
        sim.setJointPosition(jointHandles[i], targetConf[i])
    end

    local colliding=womatRobotObstacleCollision()

    for i=1,#jointHandles,1 do
        sim.setJointPosition(jointHandles[i], currentConf[i])
    end

    return colliding
end
```

The JavaScript joint preflight calls this before real movement:

```js
async function preflightJointPath(sim, scriptHandle, start, targetRadians, options, steps) {
  for (let step = 1; step <= steps; step += 1) {
    const next = interpolate(start, targetRadians, steps, step);
    if (await checkJointConfigCollisionIfEnabled(sim, scriptHandle, options, next)) {
      return {
        ok: false,
        reason: "joint_path_collision_detected",
        obstacleMode: "robot_vs_obstacle_collection",
        blockedAtStep: step
      };
    }
  }
  return { ok: true };
}
```

Comparison summary:

| Area | RobWoT | WoMAT integration |
| --- | --- | --- |
| Motion execution | CoppeliaSim coroutine state machine with `sim.moveToPose` and `sim.moveToConfig` | HTTP action-driven interpolation with preflight and structured responses |
| Workspace points | Random joint samples, point arrays, optional point clouds | Same base sampling, manifest-controlled sample count, stable CSV/TD integration |
| Workspace collision label | Broad `robotCollection` vs `sim.handle_all` | Tool point vs configured obstacle bounding boxes |
| Cartesian safety | Runtime motion command can be issued directly | Planned cartesian path is checked before motion |
| Joint safety | Generic reactive collision check | Planned joint path is checked before motion with robot-vs-obstacle collections |
| Runtime output | Mostly string/state responses | JSON result with `success`, `reason`, `blockedAtStep`, `motion.steps`, safety metadata |

## Technical Implementation Details

This section records where the important behavior is implemented and how the relevant functions work.

### Phase 1 Function Details

The phase 1 entry point is:

```text
src/generator/generateRobot.js:main()
```

It parses command-line arguments, validates the URDF path and robot name, creates output directories, writes generated files, and optionally updates the root `config.json`.

The important helper functions are:

```text
parseArgs(argv)
validateRobotName(name)
createAgentTd(name)
createAgentAsl(name)
createAgentThingTs(name, configKey, tdPath, lowerLayerUrl)
createLowerThingTs(name, thingPort)
createJcm(name, configKey)
createReadme(name, urdfPath, thingPort, agentPort, websocketPort)
```

The TD schema fix is in:

```text
src/generator/generateRobot.js:createAgentTd(name)
```

The generated `tell` action accepts arrays of strings, numbers, or booleans by using `oneOf` for each array item:

```js
values: {
  type: "array",
  items: {
    oneOf: [
      { type: "string" },
      { type: "number" },
      { type: "boolean" }
    ]
  }
}
```

This fixed the invalid schema shape that previously tried to use a mixed `type` array.

The generated TypeScript global-name fix is in:

```text
src/generator/generateRobot.js:createAgentThingTs(...)
```

The generated file ends with:

```ts
export {};
```

This turns the generated TypeScript file into a module and prevents duplicate global `main()` function conflicts when several generated files exist.

The lower-layer URL is normalized by:

```text
lowerThingTitle(name)
lowerThingSlug(name)
```

For `myRobot`, the slug becomes:

```text
coppeliasim_virtualrobot_myrobot
```

That is why the runtime TD is reachable at:

```text
http://localhost:8110/coppeliasim_virtualrobot_myrobot
```

### Phase 2 Function Details

The phase 2 bridge entry point is:

```text
src/generator/robwotBridge.js:main()
```

It reads the manifest, checks selected mode flags, and delegates to:

```text
generateModel(ctx)
generateWorkspaceData(ctx)
generateLowerTd(ctx)
updateManifest(ctx, updates)
```

The context is built by:

```text
src/generator/robwotBridge.js:createContext(manifestFile)
```

`createContext` resolves all important paths from the manifest, including:

- URDF file.
- RobWoT root.
- URDF import scene.
- Workspace scene template.
- Workspace driver Lua file.
- Generated model path.
- Generated workspace scene path.
- Generated CSV, STL, info JSON, and TD paths.

#### URDF Import Fix

The URDF import fix is implemented in:

```text
src/generator/robwotBridge.js:importUrdfThroughSandbox(sim, ctx)
```

The original problem was that the CoppeliaSim websocket remote API did not expose `simURDF.import` directly. The fix is to ask CoppeliaSim to execute Lua code inside its own sandbox script:

```js
async function importUrdfThroughSandbox(sim, ctx) {
  const sandboxHandle = await getSandboxScriptHandle(sim);
  const replacement = ctx.manifest.pipeline?.packageReplacement || inferPackageReplacement(ctx.paths.urdf);
  const script = [
    "local simURDF = require('simURDF')",
    `return simURDF.import(${luaString(ctx.paths.urdf)}, 516, ${luaString(replacement)})`
  ].join("\n");
  const result = await sim.executeScriptString(script, sandboxHandle);
  const executionResult = Array.isArray(result) ? result[1] : result;
  console.log(`URDF import returned: ${executionResult}`);
  return executionResult;
}
```

The important idea is:

```text
Node.js websocket client -> sim.executeScriptString(...) -> CoppeliaSim sandbox Lua -> simURDF.import(...)
```

This works because the plugin is available inside CoppeliaSim even though it is not exposed as a direct websocket object.

The surrounding model generation is handled by:

```text
src/generator/robwotBridge.js:generateModel(ctx)
```

Its flow is:

1. Connect to CoppeliaSim.
2. Stop simulation if it is running.
3. Load the RobWoT URDF import scene.
4. Call `importUrdfThroughSandbox`.
5. Configure joints and shape dynamics with `configureImportedRobot`.
6. Save the imported robot as a `.ttm` model.

The joint and shape configuration is in:

```text
src/generator/robwotBridge.js:configureImportedRobot(sim, robotName)
```

It finds imported objects with joints, sets joint mode/control parameters, sets aliases, configures respondable/static shape flags, and marks the root as a model.

#### Mesh Path Fix

The package path fix is also part of `importUrdfThroughSandbox`.

This line decides the replacement path:

```js
const replacement = ctx.manifest.pipeline?.packageReplacement || inferPackageReplacement(ctx.paths.urdf);
```

That replacement is passed to:

```js
simURDF.import(urdfPath, 516, replacement)
```

This is what helps resolve URDF mesh references such as `package://...`.

#### Workspace and Point Cloud Generation

The point cloud generation starts in:

```text
src/generator/robwotBridge.js:generateWorkspaceData(ctx)
```

Its flow is:

1. Check that the generated `.ttm` model exists.
2. Copy the RobWoT workspace scene template.
3. Load that scene into CoppeliaSim.
4. Load the generated robot model into the scene.
5. Attach a modified workspace driver script with `loadWorkspaceDriver`.
6. Call RobWoT's workspace calculation functions inside CoppeliaSim.
7. Save the updated scene.
8. Normalize generated files into stable WoMAT names.

The relevant execution call is:

```js
const calculationResult = await executeInScript(
  sim,
  robotScriptHandle,
  [
    `generateWorkingspace(true, ${divisions}, ${luaString(ctx.infoDir)}, ${luaArray([1, 1, 1])}, ${luaArray([0, 0, 0])})`,
    "local ok, err = pcall(doCalculation)",
    "if not ok then return false, tostring(err) end",
    "return true, robotInfo()"
  ].join("\n")
);
```

The point clouds themselves are produced by the RobWoT workspace Lua code during `doCalculation()`. WoMAT controls the sampling density and collision classification by patching the RobWoT workspace driver before attaching it.

That patching happens in:

```text
src/generator/robwotBridge.js:prepareWorkspaceDriverScript(ctx)
```

The number of sampled workspace points is taken from:

```js
const randomSteps = Number(ctx.manifest.pipeline?.workspaceRandomSteps || 80000);
```

and injected into the RobWoT Lua driver by replacing:

```js
.replace("randomSteps=80000", `randomSteps=${randomSteps}`)
```

For `myRobot`, the manifest currently uses:

```json
"workspaceRandomSteps": 80000
```

This is why increasing this manifest value increases the density of both CoppeliaSim point clouds.

The collision classification for workspace samples is also patched in `prepareWorkspaceDriverScript(ctx)`.

The original broad collision check:

```lua
sim.checkCollision(robotCollection, sim.handle_all)
```

is replaced with:

```lua
womatPointObstacleCollision(matrix[4], matrix[8], matrix[12])
```

Here, `matrix[4]`, `matrix[8]`, and `matrix[12]` are the sampled tool/world position from the workspace calculation.

The injected Lua function checks the sample against configured obstacle bounding boxes:

```lua
function womatPointObstacleCollision(x, y, z)
    local obstacleNames={...}
    for i=1,#obstacleNames,1 do
        local obstacle=sim.getObject('/'..obstacleNames[i], {noError=true})
        if obstacle ~= -1 then
            local minX=sim.getObjectFloatParam(obstacle, sim.objfloatparam_objbbox_min_x)
            local maxX=sim.getObjectFloatParam(obstacle, sim.objfloatparam_objbbox_max_x)
            local minY=sim.getObjectFloatParam(obstacle, sim.objfloatparam_objbbox_min_y)
            local maxY=sim.getObjectFloatParam(obstacle, sim.objfloatparam_objbbox_max_y)
            local minZ=sim.getObjectFloatParam(obstacle, sim.objfloatparam_objbbox_min_z)
            local maxZ=sim.getObjectFloatParam(obstacle, sim.objfloatparam_objbbox_max_z)
            local m=sim.getObjectMatrix(obstacle, sim.handle_world)
            local inv=sim.getMatrixInverse(m)
            local p=sim.multiplyVector(inv, {x, y, z})
            if p[1] >= minX and p[1] <= maxX and p[2] >= minY and p[2] <= maxY and p[3] >= minZ and p[3] <= maxZ then
                return true
            end
        end
    end
    return false
end
```

The important detail is that the world-space sample point is transformed into the obstacle's local coordinate frame before checking the local bounding box. This makes the test work even if the obstacle is moved or rotated.

The generated CSV then separates samples by `Collision_state`:

```text
Collision_state=1 -> reachable pointCloud[0]
Collision_state=0 -> colliding pointCloud[1]
```

The output file normalization is handled by:

```text
src/generator/robwotBridge.js:normalizeWorkspaceFiles(ctx, rawRobotName)
```

This copies RobWoT's generated filenames into stable WoMAT paths like:

```text
generated/robots/myRobot/info/myRobot_data_point.csv
```

### Phase 3 Function Details

The phase 3 runtime entry point is:

```text
src/generator/lowerThingServer.js:main()
```

It reads the manifest, validates generated inputs, connects to CoppeliaSim, loads the generated scene, injects the runtime driver, and starts the node-wot Thing.

The runtime context is built by:

```text
src/generator/lowerThingServer.js:createContext(manifestFile)
```

The CoppeliaSim connection is made by:

```text
src/generator/lowerThingServer.js:connect(ctx)
```

This uses RobWoT's installed `coppelia-ws-api` package and connects to:

```text
localhost:23050
```

The scene and runtime driver are loaded by:

```text
src/generator/lowerThingServer.js:loadRuntimeScene(ctx, sim)
```

Its flow is:

1. Stop the simulation.
2. Load the generated workspace scene.
3. Find the generated robot object by manifest name.
4. Remove an old customization script if one exists.
5. Add a new customization script.
6. Set its text to `prepareRuntimeDriverScript(ctx)`.
7. Associate the script with the robot object.
8. Initialize the script.

The runtime driver patching happens in:

```text
src/generator/lowerThingServer.js:prepareRuntimeDriverScript(ctx)
```

It reads the RobWoT runtime driver and disables/replaces asynchronous movement functions so WoMAT can explicitly drive movement from node-wot action calls:

```js
.replace("function sysCall_actuation()", "function sysCall_actuation_disabled()")
.replace("function moveTojoint(targetConf)", "function moveTojoint_async(targetConf)")
.replace("function moveToPosition(targetPosition)", "function moveToPosition_async(targetPosition)")
```

Then it appends synchronous helper functions:

```lua
function moveTojoint(targetConf)
    for i=1,#jointHandles,1 do
        if sim.isDynamicallyEnabled(jointHandles[i]) then
            sim.setJointTargetPosition(jointHandles[i], targetConf[i])
        else
            sim.setJointPosition(jointHandles[i], targetConf[i])
        end
    end
    rState=3
    return "success"
end

function moveToPosition(targetPosition)
    sim.setObjectPosition(simTarget, sim.handle_world, {targetPosition[1], targetPosition[2], targetPosition[3]})
    simIK.applyIkEnvironmentToScene(ikEnv, ikGroup)
    rState=3
    return "success"
end
```

JavaScript calls Lua functions through:

```text
src/generator/lowerThingServer.js:executeInScript(sim, scriptHandle, code)
```

Example:

```js
return executeInScript(sim, scriptHandle, "return getCartesianposition()");
```

The actual node-wot Thing is created in:

```text
src/generator/lowerThingServer.js:startThing(ctx, sim, scriptHandle)
```

It sets property and action handlers:

```js
thing.setPropertyReadHandler("getJointposition", async () => {
  return executeInScript(sim, scriptHandle, "return getJointposition()");
});

thing.setPropertyReadHandler("getCartesianposition", async () => {
  return executeInScript(sim, scriptHandle, "return getCartesianposition()");
});

thing.setActionHandler("moveTojointPosition", async (params) => {
  const payload = await params.value();
  const radians = jointPayloadToRadians(payload, td);
  return smoothJointMove(sim, scriptHandle, td, radians, options);
});

thing.setActionHandler("moveTocartesianPosition", async (params) => {
  const payload = await params.value();
  return smoothCartesianMove(sim, scriptHandle, { x: Number(payload.x), y: Number(payload.y), z: Number(payload.z) }, safety, options);
});
```

This is the runtime bridge from WoT HTTP requests into CoppeliaSim Lua script calls.

### Phase 5 Function Details

The runtime safety settings are read by:

```text
src/generator/lowerThingServer.js:createRuntimeOptions(ctx)
```

It maps manifest values into runtime options:

```js
return {
  jointSteps: Number(runtime.jointSteps || 60),
  cartesianSteps: Number(runtime.cartesianSteps || 60),
  stepDelayMs: Number(runtime.stepDelayMs || 25),
  cartesianMaxVelocity: Number(runtime.cartesianMaxVelocity || 0.45),
  jointMaxVelocityDeg: Number(runtime.jointMaxVelocityDeg || 180),
  workspaceTolerance: Number(runtime.workspaceTolerance || 0.08),
  enableCollisionCheck: runtime.enableCollisionCheck === true
};
```

The workspace CSV is read by:

```text
src/generator/lowerThingServer.js:readWorkspaceSafety(csvFile)
```

It parses rows, keeps reachable rows where `Collision_state === 1`, and computes workspace bounds:

```js
const reachable = rows.filter((row) => row[3] === 1);
const bounds = {
  x: [Math.min(...source.map((row) => row[0])), Math.max(...source.map((row) => row[0]))],
  y: [Math.min(...source.map((row) => row[1])), Math.max(...source.map((row) => row[1]))],
  z: [Math.min(...source.map((row) => row[2])), Math.max(...source.map((row) => row[2]))]
};
```

The workspace target check is:

```text
src/generator/lowerThingServer.js:checkWorkspace(point, safety, options)
```

It first checks whether the target is inside sampled bounds:

```js
if (!withinBounds(point, safety.bounds)) {
  return { ok: false, reason: "target_outside_workspace_bounds", bounds: safety.bounds };
}
```

Then it checks the nearest reachable sample:

```js
const nearest = nearestReachableDistance(point, safety);
if (nearest > options.workspaceTolerance) {
  return {
    ok: false,
    reason: "target_not_near_workspace_samples",
    nearestReachableDistance: nearest,
    tolerance: options.workspaceTolerance
  };
}
```

This prevents requests that are far outside the generated workspace cloud.

#### Smooth Cartesian Movement

Smooth cartesian motion is implemented in:

```text
src/generator/lowerThingServer.js:smoothCartesianMove(sim, scriptHandle, point, safety, options)
```

The high-level flow is:

1. Check target against sampled workspace.
2. Read current cartesian position.
3. Compute the distance-aware interpolation step count.
4. Delegate collision path preflight to the phase 6 collision layer.
5. If the path is safe, interpolate and apply each step.
6. Keep reactive collision checks as a fallback guard.

The number of cartesian steps is computed by:

```text
src/generator/lowerThingServer.js:cartesianStepCount(start, target, options)
```

It calculates the euclidean target distance, converts `cartesianMaxVelocity` and `stepDelayMs` into a maximum allowed step length, and returns the larger of the configured minimum and the velocity-limited step count:

```js
function cartesianStepCount(start, target, options) {
  const distance = cartesianDistance(start, target);
  const maxStep = options.cartesianMaxVelocity * (options.stepDelayMs / 1000);
  const velocitySteps = maxStep > 0 ? Math.ceil(distance / maxStep) : 1;
  return Math.max(1, options.cartesianSteps, velocitySteps);
}
```

The interpolation is:

```js
const alpha = step / steps;
const next = [
  start.x + (point.x - start.x) * alpha,
  start.y + (point.y - start.y) * alpha,
  start.z + (point.z - start.z) * alpha,
  0,
  0.707,
  0,
  0.707
];
```

The movement is applied with:

```js
await executeInScript(sim, scriptHandle, `return moveToPosition(${luaArray(next)})`);
```

### Phase 6 Function Details

#### Preflight Collision Detection

Preflight collision detection is implemented in:

```text
src/generator/lowerThingServer.js:preflightCartesianPath(sim, scriptHandle, start, target, options, steps)
```

It samples the planned straight-line cartesian path before any motion is executed:

```js
async function preflightCartesianPath(sim, scriptHandle, start, target, options, steps) {
  for (let step = 1; step <= steps; step += 1) {
    const alpha = step / steps;
    const point = {
      x: start.x + (target.x - start.x) * alpha,
      y: start.y + (target.y - start.y) * alpha,
      z: start.z + (target.z - start.z) * alpha
    };
    if (await checkPointCollisionIfEnabled(sim, scriptHandle, options, point)) {
      return {
        ok: false,
        reason: "path_collision_detected",
        obstacleMode: "tip_vs_obstacle_aabb",
        blockedAtStep: step,
        blockedPoint: point
      };
    }
  }

  return { ok: true };
}
```

The collision query is delegated to:

```text
src/generator/lowerThingServer.js:checkPointCollisionIfEnabled(sim, scriptHandle, options, point)
```

which calls the CoppeliaSim Lua helper:

```js
return executeInScript(sim, scriptHandle, `return womatPointObstacleCollision(${point.x}, ${point.y}, ${point.z})`);
```

The helper `womatPointObstacleCollision` is appended inside `prepareRuntimeDriverScript(ctx)`. It is the runtime version of the same point-vs-obstacle AABB idea used during workspace generation.

The preflight result is used in `smoothCartesianMove` before the movement loop:

```js
const start = await getCartesian(sim, scriptHandle);
const steps = cartesianStepCount(start, point, options);
const pathSafety = await preflightCartesianPath(sim, scriptHandle, start, point, options, steps);
if (!pathSafety.ok) {
  return { success: false, ...pathSafety };
}
```

This is the exact change that prevents the robot from moving into the wall after the system has detected the collision.

#### Smooth Joint Movement

Smooth joint motion is implemented in:

```text
src/generator/lowerThingServer.js:smoothJointMove(sim, scriptHandle, td, targetRadians, options)
```

It reads current joint angles, computes a planned joint path, preflights that path against the configured obstacles, and only then applies each step. It still keeps the reactive runtime collision guard as a fallback.

The number of joint steps is computed by:

```text
src/generator/lowerThingServer.js:jointStepCount(start, target, options)
```

It takes the largest joint delta, converts `jointMaxVelocityDeg` and `stepDelayMs` into a maximum joint step in radians, and returns the larger of the configured minimum and velocity-limited step count.

```js
const start = await getJointRadians(sim, scriptHandle, td);
const steps = jointStepCount(start, targetRadians, options);
const pathSafety = await preflightJointPath(sim, scriptHandle, start, targetRadians, options, steps);
if (!pathSafety.ok) {
  return { success: false, ...pathSafety, motion: { steps } };
}

for (let step = 1; step <= steps; step += 1) {
  const next = interpolate(start, targetRadians, steps, step);
  await executeInScript(sim, scriptHandle, `return moveTojoint(${luaArray(next)})`);
  if (await checkCollisionIfEnabled(sim, scriptHandle, options)) {
    await executeInScript(sim, scriptHandle, `return moveTojoint(${luaArray(start)})`);
    return { success: false, reason: "collision_detected" };
  }
  await delay(options.stepDelayMs);
}
```

The joint preflight calls `jointConfigCollisionCheck(...)` inside the CoppeliaSim runtime script. That function temporarily applies each candidate joint configuration, checks robot-vs-obstacle collision collections, restores the previous joint state, and reports whether the candidate is unsafe.

#### How RobWoT Executes Motion

RobWoT's original runtime driver is:

```text
/home/maxchrs/Desktop/MA/RobWoT/Library/Robot_WoT_Server/robot_driver.lua
```

It uses a CoppeliaSim script state machine:

```lua
rState = 0 -- initial
rState = 1 -- cartesian move
rState = 2 -- joint move
rState = 3 -- stopped
```

`sysCall_actuation()` resumes movement coroutines while the simulation is stepping.

For cartesian motion, RobWoT uses:

```text
moveToPosition(targetPosition)
moveToPos(targetPos)
moveToPose_viaIK(...)
sim.moveToPose(...)
moveToPoseCallback(...)
simIK.applyIkEnvironmentToScene(...)
```

The cartesian path is executed by `sim.moveToPose` with IK and RobWoT velocity/acceleration/jerk settings:

```lua
local maxIkVel={0.45,0.45,0.45,4.5}
local maxIkAccel={0.13,0.13,0.13,1.24}
local maxIkJerk={0.1,0.1,0.1,0.2}
```

For joint motion, RobWoT uses:

```text
moveTojoint(targetConf)
moveToConfig(...)
sim.moveToConfig(...)
movCallback(...)
```

with default joint limits:

```lua
local vel=180
local accel=40
local jerk=80
```

What WoMAT takes from this:

- The idea of separating cartesian and joint motion.
- The same IK target/tip structure.
- The same default velocity scale: `0.45 m/s` cartesian and `180 deg/s` joint.
- The CoppeliaSim IK application mechanism.

What WoMAT currently does differently:

- WoMAT disables RobWoT's coroutine-driven `sysCall_actuation` movement and replaces it with explicit synchronous helper functions.
- Node-wot actions remain in control of each step, so workspace and collision checks can happen before and during execution.
- The action response includes structured safety and motion information, such as `motion.steps`.

What can be improved later:

- Reintroduce RobWoT-style acceleration and jerk profiles in the WoMAT-controlled interpolation.
- Add CoppeliaSim-side planned trajectory preview functions, so path safety can be checked without giving up the smoother `sim.moveToPose`/`sim.moveToConfig` behavior.
- Add full link-level collision checks instead of the current tool-point wall check.

## Problems Encountered and Solutions

### Problem 1: Placeholder URDF Path

The first test command used:

```text
/path/to/robot.urdf
```

This failed because the path did not exist.

Solution:

Use the real URDF path:

```text
/home/maxchrs/Desktop/MA/RobWoT/Assets/URDFs/ur_description/urdf/ur3_robot.urdf
```

### Problem 2: Manifest Missing

The pipeline failed when the manifest did not exist:

```text
generated/robots/myRobot/manifest.json
```

This happened because the generator had not successfully created the robot scaffolding yet.

Solution:

Run `generate:robot` successfully first, then run `robot:pipeline`.

Code location:

```text
src/generator/generateRobot.js:main()
src/generator/robwotBridge.js:createContext(manifestFile)
```

`generateRobot.js:main()` writes the manifest. `robwotBridge.js:createContext()` is the function that later tries to read it and resolve all generated paths.

### Problem 3: `simURDF.import` Not Exposed Over Websocket

CoppeliaSim websocket remote API did not expose:

```text
simURDF.import
```

Solution:

Use `sim.executeScriptString` to run CoppeliaSim-side Lua code that loads `simURDF` and calls `simURDF.import` inside the simulator context.

Code location:

```text
src/generator/robwotBridge.js:importUrdfThroughSandbox(sim, ctx)
src/generator/robwotBridge.js:generateModel(ctx)
```

The fix is that `generateModel(ctx)` no longer depends on direct websocket exposure of `simURDF.import`. Instead it calls `importUrdfThroughSandbox(sim, ctx)`, which executes:

```lua
local simURDF = require('simURDF')
return simURDF.import(...)
```

inside the CoppeliaSim sandbox script.

### Problem 4: Manual URDF Import Showed Boxes Instead of Full Visual Meshes

Manual URDF import initially showed simplified cuboid-like joint objects.

Likely cause:

Mesh references in the URDF were not being resolved correctly, especially `package://` paths.

Solution:

The bridge passes a replacement package path during `simURDF.import` so the URDF importer can resolve RobWoT/UR meshes.

Code location:

```text
src/generator/robwotBridge.js:importUrdfThroughSandbox(sim, ctx)
```

The important line is:

```js
const replacement = ctx.manifest.pipeline?.packageReplacement || inferPackageReplacement(ctx.paths.urdf);
```

That replacement is passed as the third argument to `simURDF.import`.

### Problem 5: Workspace Script Was Not Initialized

The workspace generation failed with messages such as:

```text
script is not initialized
```

and later:

```text
sim.executeScriptString: operation failed
```

Solution:

The workspace generation was moved to a properly initialized customization script attached to the robot/object in CoppeliaSim, then executed through `sim.executeScriptString`.

Code location:

```text
src/generator/robwotBridge.js:loadWorkspaceDriver(...)
src/generator/robwotBridge.js:waitForScriptInitialization(sim, scriptHandle)
src/generator/robwotBridge.js:executeInScript(sim, scriptHandle, code)
src/generator/robwotBridge.js:generateWorkspaceData(ctx)
```

`generateWorkspaceData(ctx)` loads the model into the workspace scene, attaches the patched workspace driver, waits for initialization, and only then calls `generateWorkingspace(...)` and `doCalculation()`.

### Problem 6: Ports Were Occupied

Ports such as `8084`, `8085`, `8086`, and later `8110` were occupied by stale Node/CoppeliaSim-related processes.

Solution:

Use `lsof` to identify listeners and stop only the relevant stale process.

Example:

```bash
lsof -iTCP:8110 -sTCP:LISTEN -n -P
```

### Problem 7: CoppeliaSim Websocket Not Running

`robot:serve` failed with:

```text
Could not connect to CoppeliaSim websocket remote API at localhost:23050
```

Solution:

Start CoppeliaSim first and make sure the websocket remote API is listening on port `23050`.

Check with:

```bash
lsof -iTCP:23050 -sTCP:LISTEN -n -P
```

Code location:

```text
src/generator/lowerThingServer.js:connect(ctx)
src/generator/robwotBridge.js:connect(ctx)
```

Both runtime serving and pipeline generation depend on the same CoppeliaSim websocket API endpoint.

### Problem 8: Collision Was Detected But Robot Still Moved

The first collision implementation checked during interpolation. That meant the robot could visibly move toward or into the wall before returning an error.

Solution:

Add cartesian path preflight in `lowerThingServer.js`. The runtime now samples the full planned cartesian path against the wall AABB before movement starts. If any point collides, the whole action is rejected and the robot does not move.

Code location:

```text
src/generator/lowerThingServer.js:preflightCartesianPath(sim, scriptHandle, start, target, options)
src/generator/lowerThingServer.js:checkPointCollisionIfEnabled(sim, scriptHandle, options, point)
src/generator/lowerThingServer.js:smoothCartesianMove(sim, scriptHandle, point, safety, options)
```

`smoothCartesianMove(...)` calls `preflightCartesianPath(...)` before the interpolation loop. If preflight returns `path_collision_detected`, no `moveToPosition(...)` command is sent to CoppeliaSim.

### Problem 9: Point Clouds Were Sparse

The workspace point clouds were generated with:

```json
"workspaceRandomSteps": 10000
```

Solution:

Increase the manifest value to:

```json
"workspaceRandomSteps": 80000
```

Then regenerate:

```bash
npm run robot:pipeline -- --manifest generated/robots/myRobot/manifest.json --generate-td
```

Code location:

```text
src/generator/robwotBridge.js:prepareWorkspaceDriverScript(ctx)
src/generator/robwotBridge.js:generateWorkspaceData(ctx)
```

`prepareWorkspaceDriverScript(ctx)` reads `pipeline.workspaceRandomSteps` from the manifest and injects it into RobWoT's workspace Lua driver. `generateWorkspaceData(ctx)` then runs the patched driver in CoppeliaSim, producing the denser CSV and point clouds.

### Problem 10: Fresh Robots Moved Through the Wall

After cleaning generated outputs and testing a fresh URDF as `testRobot`, the wall target:

```json
{"x":-0.25,"y":-0.2,"z":0.3}
```

initially returned:

```json
{"success":true,"state":"cartesian_motion_complete"}
```

Cause:

The fresh `testRobot` manifest was created by the older generator path and only contained phase 2 pipeline information. It did not contain the phase 5/6 `runtime` section, so `lowerThingServer.js:createRuntimeOptions(ctx)` used:

```js
enableCollisionCheck: runtime.enableCollisionCheck === true
```

and resolved collision checking to `false`.

Solution:

The generator and pipeline manifest updater now write runtime safety defaults:

```json
"runtime": {
  "jointSteps": 60,
  "cartesianSteps": 60,
  "stepDelayMs": 25,
  "cartesianMaxVelocity": 0.45,
  "jointMaxVelocityDeg": 180,
  "workspaceTolerance": 0.08,
  "enableCollisionCheck": true,
  "collisionObstacles": ["80cmHighWall50cm"]
}
```

Code location:

```text
src/generator/generateRobot.js
src/generator/robwotBridge.js:updateManifest(ctx, updates)
```

The current `testRobot` manifest was also patched with these settings. After restarting `npm run robot:serve`, the same wall command returned:

```json
{
  "success": false,
  "reason": "path_collision_detected",
  "obstacleMode": "tip_vs_obstacle_aabb",
  "blockedAtStep": 31
}
```

Important testing note:

After changing `manifest.json`, restart the lower Thing server. The runtime reads manifest settings only on startup.

### Maintenance: Delete a Generated Robot

Generated robot outputs can be removed with:

```bash
npm run delete:robot -- --name <robotName>
```

The script is implemented in:

```text
src/generator/deleteRobot.js
```

It removes:

```text
generated/robots/<robotName>
src/WoT/generated/<robotName>_agent_thing.ts
src/WoT/generated/<robotName>_lower_thing.ts
src/agt/generated/<robotName>_agent.asl
src/td/generated/<robotName>.agent.td.json
```

It also reads the robot manifest, finds the generated `configKey`, and removes that entry from:

```text
config.json
```

The script supports a safe preview mode:

```bash
npm run delete:robot -- --name <robotName> --dry-run
```

## Current State

The current implementation can:

- Generate WoMAT robot scaffolding from a URDF input.
- Use RobWoT/CoppeliaSim to import the URDF and create a `.ttm` model.
- Generate workspace data, CSV files, point clouds, and lower TDs.
- Expose a generated lower-level robot Thing.
- Expose a generated AgentThing.
- Run JaCaMo with the generated `.jcm`.
- Perform smooth cartesian and joint interpolation.
- Reject cartesian targets outside the sampled workspace.
- Reject cartesian paths that would collide with the configured wall before motion begins.
- Reject joint-space paths that would move the robot body into the configured wall before motion begins.
- Generate denser workspace point clouds using `workspaceRandomSteps`.

## Remaining Work

The current collision layer includes cartesian tool-point path preflight and full robot-vs-obstacle collection checks for joint path preflight.

Future improvements should include:

- More accurate mesh-based collision geometry.
- More advanced motion planning instead of straight-line cartesian interpolation.
- Collision-aware replanning around obstacles.
- Per-robot tuning of runtime safety settings.
- Automatic cleanup or warning for occupied ports.
