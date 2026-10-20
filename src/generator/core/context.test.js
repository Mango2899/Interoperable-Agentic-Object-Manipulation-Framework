"use strict";

const assert = require("node:assert/strict");
const fs = require("node:fs");
const path = require("node:path");
const test = require("node:test");

const { REPO_ROOT, assertRuntimeInputs, createContext } = require("./context");

test("the repository root is the repository, not whichever directory this file sits in", () => {
  // Moving createContext into core/ silently shifted this one level deeper, and
  // every generated path in every manifest resolved under src/ instead of the
  // repository root. Nothing failed until a robot was actually served, because
  // no test had ever asked whether the paths a context produces exist.
  assert.ok(fs.existsSync(path.join(REPO_ROOT, "package.json")));
  assert.ok(fs.existsSync(path.join(REPO_ROOT, "src", "generator", "core", "context.js")));
  assert.equal(path.basename(REPO_ROOT), "WoMAT");
});

test("a generated robot's context points at files that are actually there", () => {
  for (const name of ["Robot1", "Robot2", "Robot3"]) {
    const manifest = `generated/robots/${name}/manifest.json`;
    if (!fs.existsSync(path.join(REPO_ROOT, manifest))) continue;
    const ctx = createContext(manifest);
    assert.equal(ctx.repoRoot, REPO_ROOT);
    for (const [label, file] of Object.entries(ctx.paths)) {
      if (!file) continue;
      assert.ok(fs.existsSync(file), `${name} ${label} does not exist: ${file}`);
    }
    // The same check the runtime makes before it connects to anything.
    assertRuntimeInputs(ctx);
  }
});
