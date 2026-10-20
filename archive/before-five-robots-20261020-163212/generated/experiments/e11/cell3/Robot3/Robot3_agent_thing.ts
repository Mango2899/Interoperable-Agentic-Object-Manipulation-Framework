import { AgentThing } from "../agentThing";
import { AgentMessage } from "../agentConnector";

class GeneratedRobotAgentThing extends AgentThing {
    private lowerThing: any = null;
    private lowerTD: any = null;
    private lowerLayerAddress: string;
    public homePosition = { x: 0, y: 0, z: 0.3 };
    public homeJoints: any = [0,0,0,0,0,0];

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

        await this.invokeHomeMotion();
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

        if (keyword === "moveTo" || keyword === "moveAt") {
            const payload = { x: value[0], y: value[1], z: value[2] };
            const result = await this.invokeLowerMove(payload);
            if (!result.success) {
                await this.addBelief2Agent("motionFailed", [result.reason || "move_failed"], false, "ownLayer1");
                return true;
            }
            await this.addBelief2Agent("movedTo", value, false, "ownLayer1");
            return true;
        }

        if (keyword === "goHome" || keyword === "homeing" || keyword === "goHomeJoints") {
            const requestedHomeJoints = keyword === "goHomeJoints" ? value : null;
            const result = await this.invokeHomeMotion(requestedHomeJoints);
            if (!result.success) {
                await this.addBelief2Agent("motionFailed", [result.reason || "home_failed"], false, "ownLayer1");
                return true;
            }
            await this.addBelief2Agent("movedTo", ["home"], false, "ownLayer1");
            return true;
        }

        if (keyword === "horizontalDemo") {
            await this.addAchieveGoal2Agent("horizontalDemo", null, false, "unknown");
            return true;
        }

        return false;
    }

    private async invokeLowerMove(payload: { x: number; y: number; z: number }) {
        if (!this.lowerThing) {
            throw new Error("Lower-level Thing is not connected.");
        }

        const output = await this.lowerThing.invokeAction("moveTocartesianPosition", payload);
        return this.unwrapOutput(output);
    }

    private async invokeHomeMotion(homeJoints: any = null) {
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

        return this.invokeLowerMove(this.homePosition);
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
    const ownTD = "./../../src/td/generated/Robot3.agent.td.json";
    const lowerLayerAddress = "http://localhost:8130/coppeliasim_virtualrobot_robot3";
    const initialKnownAgents = {};
    const additionalOntology = {};

    const robot = new GeneratedRobotAgentThing(config["Robot3"], ownTD, initialKnownAgents, lowerLayerAddress, additionalOntology);
    await robot.init();
}

main();

export {};
