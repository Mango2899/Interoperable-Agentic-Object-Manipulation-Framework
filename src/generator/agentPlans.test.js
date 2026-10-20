const assert = require("node:assert/strict");
const fs = require("node:fs");
const os = require("node:os");
const path = require("node:path");
const test = require("node:test");
const { execFileSync, spawnSync } = require("node:child_process");

const root = path.resolve(__dirname, "../..");

// A syntax error in a plan file is only discovered when the MAS starts, which
// costs a full gradle-plus-simulator cycle. The interpreter that will read
// these files is already on disk as a gradle dependency, so ask it here.
function jasonClasspath() {
  const caches = path.join(os.homedir(), ".gradle", "caches", "modules-2");
  if (!fs.existsSync(caches)) return null;
  const jars = [];
  const walk = (dir, depth) => {
    if (depth > 8 || jars.length > 400) return;
    let entries;
    try {
      entries = fs.readdirSync(dir, { withFileTypes: true });
    } catch (error) {
      return;
    }
    for (const entry of entries) {
      const full = path.join(dir, entry.name);
      if (entry.isDirectory()) walk(full, depth + 1);
      else if (entry.name.endsWith(".jar")) jars.push(full);
    }
  };
  walk(caches, 0);
  return jars.some((jar) => path.basename(jar).startsWith("jason-interpreter"))
    ? jars.join(path.delimiter)
    : null;
}

const classpath = jasonClasspath();
const toolsPresent = ["javac", "java"].every(
  (tool) => spawnSync(tool, ["-version"], { stdio: "ignore" }).status === 0
);
const skip = !classpath
  ? "Jason interpreter jar not in the gradle cache"
  : !toolsPresent && "javac/java not installed";

test("every shipped plan file parses with the interpreter that will run it", { skip }, () => {
  const dir = fs.mkdtempSync(path.join(os.tmpdir(), "womat-asl-"));
  fs.writeFileSync(path.join(dir, "ParseAsl.java"), `
import jason.asSyntax.parser.as2j;
import jason.asSemantics.Agent;
import java.io.FileInputStream;
public class ParseAsl {
  public static void main(String[] args) throws Exception {
    int bad = 0;
    for (String file : args) {
      try {
        Agent ag = new Agent();
        ag.initAg();
        new as2j(new FileInputStream(file)).agent(ag);
        System.out.println("OK " + file + " " + ag.getPL().getPlans().size());
      } catch (Throwable error) {
        bad++;
        System.out.println("FAIL " + file + " :: " + error);
      }
    }
    System.exit(bad == 0 ? 0 : 1);
  }
}
`);
  execFileSync("javac", ["-cp", classpath, "-d", dir, path.join(dir, "ParseAsl.java")], { stdio: "pipe" });

  const files = [
    "src/agt/handoverPlans.asl",
    "src/agt/wotPlans.asl",
    ...fs.readdirSync(path.join(root, "src/agt/generated"))
      .filter((name) => name.endsWith(".asl"))
      .map((name) => `src/agt/generated/${name}`)
  ];

  let output;
  try {
    output = execFileSync("java", ["-cp", `${classpath}${path.delimiter}${dir}`, "ParseAsl", ...files], {
      cwd: root, encoding: "utf8", stdio: ["ignore", "pipe", "pipe"]
    });
  } catch (error) {
    assert.fail(`plan files failed to parse:\n${error.stdout || error.message}`);
  }

  const parsed = output.split("\n").filter((line) => line.startsWith("OK "));
  assert.equal(parsed.length, files.length, `expected every file to parse:\n${output}`);
  // A file that parses to nothing is a file the MAS will silently ignore.
  for (const line of parsed) {
    const [, file, count] = line.split(/\s+/);
    assert.ok(Number(count) > 0, `${file} parsed to zero plans`);
  }
});
