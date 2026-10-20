"use strict";

// The runtime as one searchable surface.
//
// A number of tests assert on how the runtime is written rather than on what it
// returns: that a sweep runs inside a single script call, that the watchdog
// tests contact on every step, that a planning call cannot run long enough to
// raise CoppeliaSim's abort dialog. Those properties are not observable from a
// return value, and they were previously checked by grepping one large file.
//
// The runtime is now a directory of modules plus the Lua chunks they assemble,
// so this joins them back into the single surface those assertions read. It is
// deliberately not part of any runtime path.

const fs = require("fs");
const path = require("path");

const { driverTemplate } = require("./luaDriver");

const CORE_DIRECTORY = __dirname;

function coreFiles() {
  return fs.readdirSync(CORE_DIRECTORY)
    // Tests are not runtime. Including them made a "the runtime never does X"
    // assertion pass or fail on whether a test happened to mention X.
    .filter((name) => name.endsWith(".js") && !name.endsWith(".test.js") && name !== "runtimeSource.js")
    .sort();
}

function runtimeSource() {
  const modules = coreFiles().map((name) => fs.readFileSync(path.join(CORE_DIRECTORY, name), "utf8"));
  return [
    fs.readFileSync(path.resolve(CORE_DIRECTORY, "..", "lowerThingServer.js"), "utf8"),
    ...modules,
    driverTemplate()
  ].join("\n");
}

module.exports = { CORE_DIRECTORY, coreFiles, runtimeSource };
