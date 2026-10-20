#!/usr/bin/env bash
set -euo pipefail
cd '/home/maxchrs/Desktop/MA/WoMAT'
npm run robot:serve -- --manifests generated/experiments/e11/cell4/Robot1/manifest.json,generated/experiments/e11/cell4/Robot2/manifest.json,generated/experiments/e11/cell4/Robot3/manifest.json,generated/experiments/e11/cell4/Robot4/manifest.json
