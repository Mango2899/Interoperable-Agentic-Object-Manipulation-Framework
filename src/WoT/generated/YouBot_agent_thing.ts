import { AgentThing } from "../agentThing";
import { AgentMessage } from "../agentConnector";

class GeneratedMobileRobotAgentThing extends AgentThing {
    private lowerThing: any = null;
    private lowerTD: any = null;
    private lowerLayerAddress: string;
    public baseHomePose = { x: 0, y: 0, theta: 0 };
    public armHomePosition = { x: 0.4, y: 0.02, z: 0.53 };
    public homeJoints: any = null;

    constructor(config: any, path2TD: string, initialKnownAgents: { [key: string]: string }, lowerLayerAddress: string, additionalOntology: any = {}) {
        super(config, path2TD, initialKnownAgents, additionalOntology);
        this.lowerLayerAddress = lowerLayerAddress;
    }

    protected async customInit(): Promise<boolean> {
        try {
            [this.lowerTD, this.lowerThing] = await this.fetchAgentTD(this.lowerLayerAddress);
        } catch (error) {
            console.error("Failed to connect to generated lower-level Thing. Start it before this AgentThing.", error);
            return false;
        }

        await this.invokeArmHomeMotion();
        return true;
    }

    protected async handleCustomAgentRequests(data: AgentMessage): Promise<boolean> {
        const targetAgentId = data.targetAgentId;
        const content = data.content as any;
        const keyword = content.keyword;
        const value = content?.value ?? [];

        if (targetAgentId !== "ownLayer1") {
            return false;
        }

        if (keyword === "goTo") {
            const payload = { x: value[0], y: value[1], theta: value[2] ?? 0 };
            const result = await this.invokeBaseMove(payload);
            if (!result.success) {
                await this.addBelief2Agent("motionFailed", [result.reason || "navigation_failed"], false, "ownLayer1");
                return true;
            }
            await this.addBelief2Agent("arrivedAt", value, false, "ownLayer1");
            return true;
        }

        if (keyword === "moveArmTo" || keyword === "moveTo" || keyword === "moveAt") {
            const payload = { x: value[0], y: value[1], z: value[2] };
            const result = await this.invokeArmMove(payload);
            if (!result.success) {
                await this.addBelief2Agent("motionFailed", [result.reason || "arm_move_failed"], false, "ownLayer1");
                return true;
            }
            await this.addBelief2Agent("movedTo", value, false, "ownLayer1");
            return true;
        }

        if (keyword === "goHome" || keyword === "homeing" || keyword === "goHomeJoints") {
            const requestedHomeJoints = keyword === "goHomeJoints" ? value : null;
            const result = await this.invokeArmHomeMotion(requestedHomeJoints);
            if (!result.success) {
                await this.addBelief2Agent("motionFailed", [result.reason || "home_failed"], false, "ownLayer1");
                return true;
            }
            await this.addBelief2Agent("movedTo", ["home"], false, "ownLayer1");
            return true;
        }

        if (keyword === "goHomeBase") {
            const result = await this.invokeBaseMove(this.baseHomePose);
            if (!result.success) {
                await this.addBelief2Agent("motionFailed", [result.reason || "navigation_failed"], false, "ownLayer1");
                return true;
            }
            await this.addBelief2Agent("arrivedAt", [this.baseHomePose.x, this.baseHomePose.y, this.baseHomePose.theta], false, "ownLayer1");
            return true;
        }

        return false;
    }

    private async invokeBaseMove(payload: { x: number; y: number; theta: number }) {
        if (!this.lowerThing) {
            throw new Error("Lower-level Thing is not connected.");
        }

        const output = await this.lowerThing.invokeAction("moveTo", payload);
        return this.unwrapOutput(output);
    }

    private async invokeArmMove(payload: { x: number; y: number; z: number }) {
        if (!this.lowerThing) {
            throw new Error("Lower-level Thing is not connected.");
        }

        const output = await this.lowerThing.invokeAction("moveArmTo", payload);
        return this.unwrapOutput(output);
    }

    private async invokeArmHomeMotion(homeJoints: any = null) {
        if (!this.lowerThing) {
            throw new Error("Lower-level Thing is not connected.");
        }

        const configuredHomeJoints = homeJoints !== null ? homeJoints : this.homeJoints;
        if (configuredHomeJoints !== null) {
            const joints = Array.isArray(configuredHomeJoints) ? configuredHomeJoints : configuredHomeJoints?.value ?? configuredHomeJoints;
            const payload: any = {};
            const jointNames = Object.keys(this.lowerTD?.actions?.moveTojointPosition?.input?.properties || {});
            jointNames.forEach((jointName, index) => {
                payload[jointName] = Number(joints[index] || 0);
            });
            const output = await this.lowerThing.invokeAction("moveTojointPosition", payload);
            return this.unwrapOutput(output);
        }

        return this.invokeArmMove(this.armHomePosition);
    }

    private async unwrapOutput(output: any) {
        if (output && typeof output.value === "function") {
            const value = await output.value();
            return value || { success: true };
        }
        return output || { success: true };
    }
}

async function main() {
    const config = require("./../../../config.json");
    const ownTD = "./../../src/td/generated/YouBot.agent.td.json";
    const lowerLayerAddress = "http://localhost:8094/coppeliasim_virtualrobot_youbot";
    const initialKnownAgents = {};
    const additionalOntology = {};

    const robot = new GeneratedMobileRobotAgentThing(config["YouBot"], ownTD, initialKnownAgents, lowerLayerAddress, additionalOntology);
    await robot.init();
}

main();

export {};
