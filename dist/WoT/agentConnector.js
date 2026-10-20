"use strict";
// agentConnector.ts
var __createBinding = (this && this.__createBinding) || (Object.create ? (function(o, m, k, k2) {
    if (k2 === undefined) k2 = k;
    var desc = Object.getOwnPropertyDescriptor(m, k);
    if (!desc || ("get" in desc ? !m.__esModule : desc.writable || desc.configurable)) {
      desc = { enumerable: true, get: function() { return m[k]; } };
    }
    Object.defineProperty(o, k2, desc);
}) : (function(o, m, k, k2) {
    if (k2 === undefined) k2 = k;
    o[k2] = m[k];
}));
var __setModuleDefault = (this && this.__setModuleDefault) || (Object.create ? (function(o, v) {
    Object.defineProperty(o, "default", { enumerable: true, value: v });
}) : function(o, v) {
    o["default"] = v;
});
var __importStar = (this && this.__importStar) || (function () {
    var ownKeys = function(o) {
        ownKeys = Object.getOwnPropertyNames || function (o) {
            var ar = [];
            for (var k in o) if (Object.prototype.hasOwnProperty.call(o, k)) ar[ar.length] = k;
            return ar;
        };
        return ownKeys(o);
    };
    return function (mod) {
        if (mod && mod.__esModule) return mod;
        var result = {};
        if (mod != null) for (var k = ownKeys(mod), i = 0; i < k.length; i++) if (k[i] !== "default") __createBinding(result, mod, k[i]);
        __setModuleDefault(result, mod);
        return result;
    };
})();
var __awaiter = (this && this.__awaiter) || function (thisArg, _arguments, P, generator) {
    function adopt(value) { return value instanceof P ? value : new P(function (resolve) { resolve(value); }); }
    return new (P || (P = Promise))(function (resolve, reject) {
        function fulfilled(value) { try { step(generator.next(value)); } catch (e) { reject(e); } }
        function rejected(value) { try { step(generator["throw"](value)); } catch (e) { reject(e); } }
        function step(result) { result.done ? resolve(result.value) : adopt(result.value).then(fulfilled, rejected); }
        step((generator = generator.apply(thisArg, _arguments || [])).next());
    });
};
var __generator = (this && this.__generator) || function (thisArg, body) {
    var _ = { label: 0, sent: function() { if (t[0] & 1) throw t[1]; return t[1]; }, trys: [], ops: [] }, f, y, t, g = Object.create((typeof Iterator === "function" ? Iterator : Object).prototype);
    return g.next = verb(0), g["throw"] = verb(1), g["return"] = verb(2), typeof Symbol === "function" && (g[Symbol.iterator] = function() { return this; }), g;
    function verb(n) { return function (v) { return step([n, v]); }; }
    function step(op) {
        if (f) throw new TypeError("Generator is already executing.");
        while (g && (g = 0, op[0] && (_ = 0)), _) try {
            if (f = 1, y && (t = op[0] & 2 ? y["return"] : op[0] ? y["throw"] || ((t = y["return"]) && t.call(y), 0) : y.next) && !(t = t.call(y, op[1])).done) return t;
            if (y = 0, t) op = [op[0] & 2, t.value];
            switch (op[0]) {
                case 0: case 1: t = op; break;
                case 4: _.label++; return { value: op[1], done: false };
                case 5: _.label++; y = op[1]; op = [0]; continue;
                case 7: op = _.ops.pop(); _.trys.pop(); continue;
                default:
                    if (!(t = _.trys, t = t.length > 0 && t[t.length - 1]) && (op[0] === 6 || op[0] === 2)) { _ = 0; continue; }
                    if (op[0] === 3 && (!t || (op[1] > t[0] && op[1] < t[3]))) { _.label = op[1]; break; }
                    if (op[0] === 6 && _.label < t[1]) { _.label = t[1]; t = op; break; }
                    if (t && _.label < t[2]) { _.label = t[2]; _.ops.push(op); break; }
                    if (t[2]) _.ops.pop();
                    _.trys.pop(); continue;
            }
            op = body.call(thisArg, _);
        } catch (e) { op = [6, e]; y = 0; } finally { f = t = 0; }
        if (op[0] & 5) throw op[1]; return { value: op[0] ? op[1] : void 0, done: true };
    }
};
Object.defineProperty(exports, "__esModule", { value: true });
exports.AgentConnector = exports.AgentMessage = void 0;
/*  This file is the agent connector. It is responsible for the communication between the agent TD and the agents JaCoMo artifact.
*/
var WebSocket = __importStar(require("ws"));
var AgentMessage = /** @class */ (function () {
    function AgentMessage(performative, sourceAgentId, targetAgentId, content, messageId, isResponse) {
        if (messageId === void 0) { messageId = -1; }
        if (isResponse === void 0) { isResponse = false; }
        this.messageId = -1;
        this.isResponse = false;
        this.performative = performative;
        this.sourceAgentId = sourceAgentId;
        this.targetAgentId = targetAgentId;
        this.content = content;
        this.messageId = messageId;
        this.isResponse = isResponse;
    }
    AgentMessage.fromString = function (message) {
        var data = JSON.parse(message);
        return new AgentMessage(data.performative, data.sourceAgentId, data.targetAgentId, data.content, data.messageId, data.isResponse);
    };
    AgentMessage.prototype.toString = function () {
        return JSON.stringify({
            performative: this.performative,
            sourceAgentId: this.sourceAgentId,
            targetAgentId: this.targetAgentId,
            content: this.content,
            messageId: this.messageId,
            isResponse: this.isResponse
        });
    };
    return AgentMessage;
}());
exports.AgentMessage = AgentMessage;
var AgentConnector = /** @class */ (function () {
    function AgentConnector(wsPort) {
        this.loggingLevel = 0; // 0: log erros, 1: log erros and IA on me / when I invoke IA on others, 2: log IAs with longer input/output, 3: log everything (also messages to wotCommunicator)
        // responseHandlers is a map of requestIds and their corresponding response handlers
        this.responseHandlers = new Map();
        this.requestIdCounter = 0;
        // the agent according agent artifact should be the only client. Therefore I can send to all agents.
        this.clients = [];
        this.wsPort = wsPort;
    }
    AgentConnector.prototype.connect = function (onMessageReceivedCallback) {
        var _this = this;
        this.wss = new WebSocket.Server({ port: this.wsPort });
        this.wss.on('connection', function (ws) {
            console.log('New Local Agent connection');
            _this.clients.push(ws);
            ws.send('Local Agent Connection established');
            ws.onmessage = function (event) {
                var agentMessage = AgentMessage.fromString(event.data.toString());
                if (_this.loggingLevel > 2) {
                    console.log('Received message from agent:', JSON.stringify(agentMessage));
                }
                if (agentMessage.isResponse) {
                    var responseHandler = _this.responseHandlers.get(agentMessage.messageId);
                    if (responseHandler) {
                        responseHandler(agentMessage);
                        _this.responseHandlers.delete(agentMessage.messageId);
                    }
                    else {
                        console.log('No response handler for message:', agentMessage);
                    }
                }
                else {
                    onMessageReceivedCallback(agentMessage);
                }
            };
            ws.on('close', function () {
                console.log('Local Agent Connection closed');
                _this.clients = _this.clients.filter(function (client) { return client !== ws; });
            });
        });
        this.wss.on('error', function (error) {
            console.error('WebSocket server error:', error);
        });
    };
    AgentConnector.prototype.send2Agent = function (message) {
        this.forward2Agent(message);
    };
    AgentConnector.prototype.forward2Agent = function (message) {
        var _this = this;
        this.clients.forEach(function (client) {
            if (client.readyState === WebSocket.OPEN) {
                if (_this.loggingLevel > 2) {
                    console.log('Sending message to agent:', JSON.stringify(message));
                }
                client.send(JSON.stringify(message));
            }
            else {
                console.error('WebSocket is not open.');
            }
        });
    };
    AgentConnector.prototype.send2AgentAndWaitForResponse = function (message) {
        return __awaiter(this, void 0, void 0, function () {
            var _this = this;
            return __generator(this, function (_a) {
                return [2 /*return*/, new Promise(function (resolve, reject) {
                        _this.responseHandlers.set(_this.requestIdCounter, resolve);
                        message.messageId = _this.requestIdCounter;
                        _this.forward2Agent(message);
                        _this.requestIdCounter++;
                    })];
            });
        });
    };
    return AgentConnector;
}());
exports.AgentConnector = AgentConnector;
//# sourceMappingURL=agentConnector.js.map