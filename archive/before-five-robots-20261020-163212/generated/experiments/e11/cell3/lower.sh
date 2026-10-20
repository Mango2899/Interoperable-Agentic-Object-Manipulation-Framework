#!/usr/bin/env bash
set -euo pipefail
cd '/home/maxchrs/Desktop/MA/WoMAT'
npm run robot:serve -- --manifests generated/experiments/e11/cell3/Robot1/manifest.json,generated/experiments/e11/cell3/Robot2/manifest.json,generated/experiments/e11/cell3/Robot3/manifest.json
