"use strict";

const assert = require("node:assert/strict");
const { execFileSync } = require("node:child_process");
const fs = require("node:fs");
const os = require("node:os");
const path = require("node:path");
const test = require("node:test");

// Every plan file, parsed by the parser the MAS actually uses.
//
// A plan file that does not parse fails SILENTLY. Jason drops the whole file and
// every agent that included it carries on without a single one of its plans, so
// the first symptom is "Found a goal for which there is no relevant plan" for a
// goal that is plainly written down -- with nothing naming the file, the line or
// the character responsible. One Prolog-ism ("=<" where Jason wants "<=") took
// every plan in routePlans.asl out of all four agents at once, and the suite was
// green throughout: the plan-text tests read these files as strings, which a
// broken file still is.

const REPO_ROOT = path.resolve(__dirname, "..", "..");
const CHECKER = path.join(REPO_ROOT, "tools", "AslSyntaxCheck.java");

function jarsUnder(directory) {
  if (!fs.existsSync(directory)) return [];
  const found = [];
  const walk = (current) => {
    for (const entry of fs.readdirSync(current, { withFileTypes: true })) {
      const full = path.join(current, entry.name);
      if (entry.isDirectory()) walk(full);
      else if (entry.name.endsWith(".jar")) found.push(full);
    }
  };
  walk(directory);
  return found;
}

// The interpreter and the JSON library its parser constructor touches. Both are
// gradle dependencies of this project, so they are wherever gradle put them.
function jasonClasspath() {
  const caches = [
    path.join(os.homedir(), ".gradle", "caches", "modules-2", "files-2.1"),
    path.join(REPO_ROOT, ".gradle", "caches", "modules-2", "files-2.1")
  ];
  const jars = [];
  for (const cache of caches) {
    jars.push(...jarsUnder(path.join(cache, "io.github.jason-lang")));
    jars.push(...jarsUnder(path.join(cache, "javax.json")));
  }
  return [...new Set(jars)];
}

function planFiles() {
  const directories = [path.join(REPO_ROOT, "src", "agt"), path.join(REPO_ROOT, "src", "agt", "generated")];
  return directories.flatMap((directory) => fs.readdirSync(directory)
    .filter((name) => name.endsWith(".asl"))
    .map((name) => path.join(directory, name)));
}

test("every plan file parses with Jason's own parser", () => {
  const classpath = jasonClasspath();
  assert.ok(
    classpath.some((jar) => jar.includes("jason-interpreter")),
    "no Jason interpreter jar in the gradle caches; run the MAS build once so this check has a parser"
  );
  const files = planFiles();
  assert.ok(files.length >= 5, `expected the cell's plan files, found ${files.length}`);

  let output;
  try {
    output = execFileSync(
      "java",
      ["-cp", classpath.join(path.delimiter), CHECKER, ...files],
      { encoding: "utf8", stdio: ["ignore", "pipe", "ignore"] }
    );
  } catch (error) {
    // The checker exits non-zero on the first unparseable file and prints the
    // parser's own message -- line, column, and what it was expecting.
    assert.fail(`Jason could not parse a plan file:\n${error.stdout || error.message}`);
  }
  for (const file of files) {
    assert.match(output, new RegExp(`^ok\\s+${file.replace(/[.*+?^${}()|[\]\\]/g, "\\$&")}$`, "m"));
  }
});

test("a broken plan file is caught, with the line that broke it", () => {
  // The check is only worth having if it fails when it should. "=<" is the exact
  // Prolog-ism that took the cell down: valid in Prolog, not in Jason.
  const classpath = jasonClasspath();
  if (!classpath.some((jar) => jar.includes("jason-interpreter"))) return;
  const directory = fs.mkdtempSync(path.join(os.tmpdir(), "womat-asl-"));
  const broken = path.join(directory, "broken.asl");
  fs.writeFileSync(broken, "+!count(N): true <-\n    if (N =< 2) {\n        .print(N)\n    }.\n");
  try {
    execFileSync("java", ["-cp", classpath.join(path.delimiter), CHECKER, broken], {
      encoding: "utf8", stdio: ["ignore", "pipe", "ignore"]
    });
    assert.fail("the checker accepted a file Jason cannot parse");
  } catch (error) {
    assert.match(error.stdout || "", /FAIL/);
    assert.match(error.stdout || "", /line 2/, "the parser's own position must reach the reader");
  } finally {
    fs.rmSync(directory, { recursive: true, force: true });
  }
});
