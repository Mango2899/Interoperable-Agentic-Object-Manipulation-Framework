# Robot4_CR3

A copy of the generated robot **Robot4** as it stood on 2026-09-21,
taken before regenerating it from a different URDF.

## What it was built from

- URDF: `/home/maxchrs/Desktop/MA/WoMAT/external/RobWoT/Assets/URDFs/cr_description/urdf/cr3_robot.urdf`
- Model adopted from: `(none -- imported from the URDF)`
- Platform: fixed, base link `dummy_link`
- Arm joints: joint1, joint2, joint3, joint4, joint5, joint6
- Gripper joints: -
- Ports: lower 8140, agent 8141, websocket 8241
- Cell: `generated/robots/Robot1_Robot2_Robot3_Robot4_Robot5.shared.json`

## Layout

- `files/` mirrors the repository tree, so every path here is where it came from.
- `SHA256SUMS` covers `files/`; verify with `cd files && sha256sum -c ../SHA256SUMS`.
- `config.entry.json` holds only this robot's `config.json` entry (`Robot4`).
  The rest of that file belongs to the other robots and is deliberately not copied.

## Restoring it

```bash
# from the repository root
rsync -a archive/Robot4_CR3/files/ ./
# then put the config.json entry back by hand from config.entry.json
```

The cell files copied alongside the robot describe the arrangement it was part
of. Restoring the robot without them leaves a descriptor that names an arm the
scene no longer contains.
