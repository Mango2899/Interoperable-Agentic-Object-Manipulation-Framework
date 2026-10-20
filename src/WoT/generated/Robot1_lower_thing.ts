// Generated lower-level bridge entry for Robot1.
// Phase two: delegates URDF/model/TD generation to src/generator/robwotBridge.js.

async function main() {
    const bridge = require("../../generator/robwotBridge");
    const manifest = "./generated/robots/Robot1/manifest.json";
    const ctx = bridge.createContext(manifest);

    if (!bridge.validateContext(ctx)) {
        process.exit(1);
    }

    console.log("Robot1 lower-level bridge scaffold");
    console.log("Expected TD port: 8110");
    console.log("Manifest: " + manifest);
    console.log("Run model generation with:");
    console.log("  npm run robot:pipeline -- --manifest " + manifest + " --generate-model");
    console.log("Run TD generation after the model exists with:");
    console.log("  npm run robot:pipeline -- --manifest " + manifest + " --generate-td");
}

main();
