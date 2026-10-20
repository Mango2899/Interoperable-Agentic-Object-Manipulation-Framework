// Generated lower-level bridge entry for YouBot.
// Phase two: delegates URDF/model/TD generation to src/generator/robwotBridge.js.

async function main() {
    const bridge = require("../../generator/robwotBridge");
    const manifest = "./generated/robots/YouBot/manifest.json";
    const ctx = bridge.createContext(manifest);

    if (!bridge.validateContext(ctx)) {
        process.exit(1);
    }

    console.log("YouBot lower-level bridge scaffold");
    console.log("Expected TD port: 8094");
    console.log("Manifest: " + manifest);
    console.log("Run model generation with:");
    console.log("  npm run robot:pipeline -- --manifest " + manifest + " --generate-model");
    console.log("Run TD generation after the model exists with:");
    console.log("  npm run robot:pipeline -- --manifest " + manifest + " --generate-td");
}

main();
