#!/usr/bin/env bash
set -euo pipefail
cd '/home/maxchrs/Desktop/MA/WoMAT'
npm run robot:pipeline -- --manifests generated/experiments/e11/cell4/Robot1/manifest.json,generated/experiments/e11/cell4/Robot2/manifest.json,generated/experiments/e11/cell4/Robot3/manifest.json,generated/experiments/e11/cell4/Robot4/manifest.json --generate-shared-cell --offsets '0,0,0;1.39,0.19,0;2.19,1.14,0;2.12,-0.24,0' --orientations '0,0,-15;0,0,50;-90,-155,90;-90,35,90' --wall-offsets '0,0,0;0,0,0;-0.42876,-0.25726,0;-0.2,-0.2,0' --sites 'Robot1_grip_support@0.55,0.55,0.08,0,Robot1_grip_cube;Robot4_grip_support_b@2.42,-0.14,0.08' --resample ''
