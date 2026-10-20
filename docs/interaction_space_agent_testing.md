# Agent-owned interaction space

Robot 1 is the only interaction-space owner. Its AgentThing reads both raw
reachable-workspace CSV files, estimates a voxel size with deterministic
nearest-neighbour sampling, voxelises both point clouds once against a common
origin, and stores the intersection privately. It then publishes summary
beliefs to the Robot 1 BDI agent and selects handover points locally.

The shared pipeline no longer calculates the intersection. The lower runtime
knows only the handover session ID and source/target robot names; it continues
to enforce motion serialization, collision checks, and physical cube ownership.

## Automated tests

From the repository root:

```bash
npm test
```

This runs estimator, common-origin, transform, ownership, AgentThing mapping,
and lower handover-session tests.

To calculate the current Robot1/Robot2 interaction space without CoppeliaSim:

```bash
npm run interaction:estimate
```

The command should report `success: true`, an estimated voxel size, a shared
origin, and a voxel count. It also refreshes:

- `generated/robots/Robot1_Robot2.shared-space.csv`
- `generated/robots/Robot1_Robot2.shared-space.json`

## End-to-end simulation test

1. Start CoppeliaSim with the websocket remote API on port `23050`.
2. Start the shared lower runtime:

   ```bash
   npm run robot:serve -- --manifests generated/robots/Robot1/manifest.json,generated/robots/Robot2/manifest.json
   ```

3. In two more terminals, start both AgentThings:

   ```bash
   npm run robot:agent -- --manifest generated/robots/Robot1/manifest.json
   npm run robot:agent -- --manifest generated/robots/Robot2/manifest.json
   ```

4. Start JaCaMo:

   ```bash
   ./gradlew -Pjcm=generated/robots/Robot1_Robot2.jcm
   ```

   Robot 1 should log interaction-space construction and receive
   `interactionRobot`, `interactionSpace`, and `interactionSpaceReady` beliefs.
   Robot 2 should not calculate a grid.

5. Trigger the handover through Robot 1 only:

   ```bash
   curl -X POST http://localhost:8111/robot1_agent/actions/handoverCube \
     -H 'Content-Type: application/json' \
     -d '{"targetAgent":"robot2_agent","targetRobot":"robot2"}'
   ```

6. Verify that Robot 1 logs a locally selected `handoverPlan`, Robot 2 receives
   the selected receiver coordinates, and the lower runtime logs only session
   registration/transfer operations—never voxel loading or selection.

After an owner-generated CSV exists, refresh the optional green visualization:

```bash
npm run robot:pipeline -- \
  --manifests generated/robots/Robot1/manifest.json,generated/robots/Robot2/manifest.json \
  --update-shared-space-visual
```

To verify ownership rejection directly, inspect the Robot 2 Agent TD. It must
not contain the `handoverCube` action:

```bash
node -e "const td=require('./src/td/generated/Robot2.agent.td.json'); console.log(td.actions.handoverCube === undefined)"
```
