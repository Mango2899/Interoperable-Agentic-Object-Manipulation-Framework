"use strict";
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
exports.AgentThing = void 0;
var agentConnector_1 = require("./agentConnector");
var core_1 = require("@node-wot/core");
var binding_http_1 = require("@node-wot/binding-http");
var binding_http_2 = require("@node-wot/binding-http");
var plan_translator_1 = require("./plan_translator");
var AgentThing = /** @class */ (function () {
    function AgentThing(config, path2TD, initialKnownAgents, additionalOntology) {
        if (additionalOntology === void 0) { additionalOntology = {}; }
        this.logginLevel = 1; // 0: log erros, 1: log erros and IA on me / when I invoke IA on others, 2: log IAs with longer input/output, 3: log everything (also messages to wotCommunicator)
        this.thingConfig = config;
        this.thingDescrpiption = require(path2TD);
        this.servient = new core_1.Servient();
        this.agentConnector = new agentConnector_1.AgentConnector(this.thingConfig.websocketPort);
        this.woT = null;
        this.wotHelper = null;
        this.knownAgents = initialKnownAgents;
        this.subscriptions = {};
        this.TDurl = "";
        this.parsedTD = { "actions": {}, "properties": {}, "events": {} };
        this.additionalOntology = additionalOntology;
    }
    // Custom functions to be overwritten by subclasses.
    AgentThing.prototype.customInit = function () {
        if (this.logginLevel > 0) {
            console.log("No custom initialization implemented.");
        }
    };
    AgentThing.prototype.handleCustomAgentRequests = function (data) {
        return __awaiter(this, void 0, void 0, function () {
            return __generator(this, function (_a) {
                /* Handles requests from the local agent to the agentThing.
                This can be used to customly set how an agent request should be translated into the WoT world. */
                // return true if the request was handled, else false
                if (this.logginLevel > 0) {
                    console.log("No custom handling of agent requests implemented.");
                }
                return [2 /*return*/, false];
            });
        });
    };
    AgentThing.prototype.setCustomInteractionAffordanceHandlers = function (producer) {
        return __awaiter(this, void 0, void 0, function () {
            return __generator(this, function (_a) {
                /* Sets custom interaction affordance handlers for the Thing.
                Here should be defined how InteractionAffordances shall be signaled to the local agent, besides the ways set in this.produce */
                if (this.logginLevel > 0) {
                    console.log("No custom Interaction Affordance Handlers set.");
                }
                return [2 /*return*/];
            });
        });
    };
    AgentThing.prototype.init = function () {
        return __awaiter(this, void 0, void 0, function () {
            var _this = this;
            return __generator(this, function (_a) {
                // init to the producer part
                this.servient.addServer(new binding_http_1.HttpServer({ port: this.thingConfig.TDPort }));
                core_1.Helpers.setStaticAddress(this.thingConfig.staticAddress);
                // init the consumer part
                this.servient.addClientFactory(new binding_http_2.HttpClientFactory(null));
                this.wotHelper = new core_1.Helpers(this.servient);
                this.servient.start().then(function (WoT) { return __awaiter(_this, void 0, void 0, function () {
                    var _a;
                    return __generator(this, function (_b) {
                        switch (_b.label) {
                            case 0:
                                this.woT = WoT;
                                // start agent connector server to be able to communicate locally with the agent
                                this.agentConnector.connect(this.handleAgentRequests.bind(this));
                                // start the producer part
                                _a = this;
                                return [4 /*yield*/, this.produce()];
                            case 1:
                                // start the producer part
                                _a.producer = _b.sent();
                                return [2 /*return*/];
                        }
                    });
                }); });
                // // Start handling keyboard input                 
                // process.stdin.on('data', (data) => {
                //     const key = data.toString();
                //     console.log("Key pressed: ", key)
                //     if (key == 'q') { process.exit(); } 
                //     else {
                //         console.log('Invalid key pressed. Press q to quit.');
                //         return;
                //     }                     
                // });
                this.customInit();
                return [2 /*return*/];
            });
        });
    };
    AgentThing.prototype.isArrayOfStrings = function (array) {
        return Array.isArray(array) && array.every(function (item) { return typeof item === 'string'; });
    };
    AgentThing.prototype.autoSetIAHandlers = function (producer, TD) {
        return __awaiter(this, void 0, void 0, function () {
            var TDparsed, actions, properties, events;
            var _this = this;
            return __generator(this, function (_a) {
                TDparsed = JSON.parse(TD);
                this.parsedTD = TDparsed;
                actions = TDparsed.actions;
                properties = TDparsed.properties;
                events = TDparsed.events;
                // Set property read handlers
                // No observe handler is necessary. emitPropertyChange is called always when property changes
                Object.entries(properties).forEach(function (_a) {
                    var propertyName = _a[0], propertyDetails = _a[1];
                    producer.setPropertyReadHandler(propertyName, function () { return __awaiter(_this, void 0, void 0, function () {
                        var response;
                        return __generator(this, function (_a) {
                            switch (_a.label) {
                                case 0:
                                    if (this.logginLevel > 0) {
                                        console.log("Reading property: ", propertyName);
                                    }
                                    return [4 /*yield*/, this.addTestGoal2Agent(propertyName, [], true, "unknown")];
                                case 1:
                                    response = _a.sent();
                                    if (this.logginLevel > 1) {
                                        console.log("Answer for readProperty: ", propertyName, " :", response);
                                    }
                                    else if (this.logginLevel > 0) {
                                        console.log("Answer for readProperty: ", propertyName);
                                    }
                                    if (response == null) {
                                        return [2 /*return*/];
                                    }
                                    else {
                                        return [2 /*return*/, response];
                                    }
                                    return [2 /*return*/];
                            }
                        });
                    }); });
                });
                Object.entries(actions).forEach(function (_a) {
                    var _b, _c;
                    var actionName = _a[0], actionDetails = _a[1];
                    var hasInput = (_b = actionDetails === null || actionDetails === void 0 ? void 0 : actionDetails.input) !== null && _b !== void 0 ? _b : false;
                    var hasOutput = (_c = actionDetails === null || actionDetails === void 0 ? void 0 : actionDetails.output) !== null && _c !== void 0 ? _c : false;
                    producer.setActionHandler(actionName, function (params) { return __awaiter(_this, void 0, void 0, function () {
                        var inputValue, valueType, content, response;
                        return __generator(this, function (_a) {
                            switch (_a.label) {
                                case 0:
                                    inputValue = null;
                                    valueType = "none";
                                    if (!(hasInput == false)) return [3 /*break*/, 1];
                                    inputValue = null;
                                    return [3 /*break*/, 3];
                                case 1: return [4 /*yield*/, params.value()];
                                case 2:
                                    inputValue = _a.sent();
                                    _a.label = 3;
                                case 3:
                                    if (this.logginLevel > 1) {
                                        console.log("Action: ", actionName, " with input: ", inputValue);
                                    }
                                    else if (this.logginLevel > 0) {
                                        console.log("Action was invoked: ", actionName);
                                    }
                                    content = { "keyword": actionName, "value": inputValue };
                                    return [4 /*yield*/, this.addAchieveGoal2Agent(actionName, inputValue, hasOutput, "unknown")];
                                case 4:
                                    response = _a.sent();
                                    if (this.logginLevel > 1) {
                                        console.log("Answer for action: ", actionName, " :", response);
                                    }
                                    else if (this.logginLevel > 0) {
                                        console.log("Answer for action: ", actionName);
                                    }
                                    if (response == null) {
                                        return [2 /*return*/];
                                    }
                                    else {
                                        return [2 /*return*/, response];
                                    }
                                    return [2 /*return*/];
                            }
                        });
                    }); });
                });
                // overwrite the auto set handlers
                producer.setActionHandler("askHow", function (params) { return __awaiter(_this, void 0, void 0, function () {
                    var input, planLabel, targetTDAdress, content, agentMessage, data, _a, target_td, otherAgent, response, _i, _b, planString, reduced, planStringOntology, targetPlanObject;
                    return __generator(this, function (_c) {
                        switch (_c.label) {
                            case 0: return [4 /*yield*/, params.value()];
                            case 1:
                                input = _c.sent();
                                planLabel = input["planLabel"];
                                planLabel = (0, plan_translator_1.ontologyLabel2planLabel)(planLabel, this.additionalOntology);
                                targetTDAdress = input["targetTDAdress"];
                                if (this.logginLevel > 1) {
                                    console.log("Action askHow for planLabel: ", planLabel, " and target TD: ", targetTDAdress);
                                }
                                else if (this.logginLevel > 0) {
                                    console.log("Action askHow was invoked for planLabel: ", planLabel);
                                }
                                content = { "keyword": "{ +" + planLabel + "} " };
                                agentMessage = new agentConnector_1.AgentMessage("askHow", "", "", content);
                                return [4 /*yield*/, this.agentConnector.send2AgentAndWaitForResponse(agentMessage)];
                            case 2:
                                data = _c.sent();
                                if (!!this.isArrayOfStrings(data.content)) return [3 /*break*/, 3];
                                if (this.logginLevel > 0) {
                                    console.log("No plans found for planLabel: ", planLabel, " and targetTD: ", targetTDAdress);
                                }
                                return [2 /*return*/, []];
                            case 3: return [4 /*yield*/, this.fetchAgentTD(targetTDAdress)];
                            case 4:
                                _a = _c.sent(), target_td = _a[0], otherAgent = _a[1];
                                response = [];
                                for (_i = 0, _b = data.content; _i < _b.length; _i++) {
                                    planString = _b[_i];
                                    if (planString.includes("wotExchangable")) {
                                        reduced = this.stripMetadataFromPlanString(planString);
                                        planStringOntology = (0, plan_translator_1.planString2OntologyPlanObject)(reduced, TDparsed, this.additionalOntology);
                                        targetPlanObject = (0, plan_translator_1.OntologyPlanObject2TargetPlanObject)(planStringOntology, target_td);
                                        response.push(targetPlanObject);
                                    }
                                }
                                if (this.logginLevel == 1) {
                                    console.log("Plans found for planLabel: ", planLabel, " and targetTD: ", targetTDAdress);
                                }
                                else if (this.logginLevel >= 2) {
                                    console.log("Plans found for planLabel: ", planLabel, " and targetTD: ", targetTDAdress, " : ", response);
                                }
                                return [2 /*return*/, response];
                        }
                    });
                }); });
                producer.setActionHandler("tell", function (params) { return __awaiter(_this, void 0, void 0, function () {
                    var message, keyword, values, error_1;
                    var _a;
                    return __generator(this, function (_b) {
                        switch (_b.label) {
                            case 0:
                                _b.trys.push([0, 3, , 4]);
                                return [4 /*yield*/, params.value()];
                            case 1:
                                message = _b.sent();
                                keyword = message["belief"];
                                values = (_a = message["values"]) !== null && _a !== void 0 ? _a : null;
                                if (this.logginLevel > 1) {
                                    console.log("Action tell was invoked with params: ", message);
                                }
                                else if (this.logginLevel > 0) {
                                    console.log("Action tell was invoked");
                                }
                                return [4 /*yield*/, this.addBelief2Agent(keyword, values, false, "unknown")];
                            case 2:
                                _b.sent();
                                return [3 /*break*/, 4];
                            case 3:
                                error_1 = _b.sent();
                                console.error("Error in action handler for 'tell':", error_1);
                                // Re-throw the error or handle it appropriately
                                throw error_1; // This will ensure that the error is communicated back to the consumer.
                            case 4: return [2 /*return*/];
                        }
                    });
                }); });
                producer.setActionHandler("send", function (params) { return __awaiter(_this, void 0, void 0, function () {
                    var message, error_2;
                    return __generator(this, function (_a) {
                        switch (_a.label) {
                            case 0:
                                _a.trys.push([0, 2, , 3]);
                                return [4 /*yield*/, params.value()];
                            case 1:
                                message = _a.sent();
                                if (this.logginLevel > 0) {
                                    console.log("Action send was invoked");
                                }
                                else if (this.logginLevel > 1) {
                                    console.log("Action send was invoked with params: ", message);
                                }
                                this.agentConnector.forward2Agent(message);
                                return [3 /*break*/, 3];
                            case 2:
                                error_2 = _a.sent();
                                console.error("Error in action handler for 'send':", error_2);
                                // Re-throw the error or handle it appropriately
                                throw error_2; // This will ensure that the error is communicated back to the consumer.
                            case 3: return [2 /*return*/];
                        }
                    });
                }); });
                return [2 /*return*/];
            });
        });
    };
    AgentThing.prototype.stripMetadataFromPlanString = function (planString) {
        // Regular expression targets the pattern starting with '@p' followed by any characters
        // until it reaches the closing ']' bracket.
        // makes this { @p__11[source(self),url("file:src/agt/robot_0_agent.asl"),wotExchangable] +!process(blue) <- !moveTo(0.05,-0.05,0.4); !dropObject }
        // to this: { +!process(blue) <- !moveTo(0.05,-0.05,0.4); !dropObject }
        // needed for plan exchange
        var regex = /@\w+\[[^\]]+\]\s*/g; // The 'g' flag is used to replace all occurrences
        // Replace the identified pattern with an empty string
        var cleanedString = planString.replace(regex, '');
        return cleanedString;
    };
    AgentThing.prototype.produce = function () {
        return __awaiter(this, void 0, void 0, function () {
            var producer, title;
            return __generator(this, function (_a) {
                switch (_a.label) {
                    case 0: return [4 /*yield*/, this.woT.produce(this.thingDescrpiption)];
                    case 1:
                        producer = _a.sent();
                        this.autoSetIAHandlers(producer, JSON.stringify(this.thingDescrpiption));
                        // Set custom interaction affordance handler after automatic handlers to overwrite them if necceassary
                        return [4 /*yield*/, this.setCustomInteractionAffordanceHandlers(producer)];
                    case 2:
                        // Set custom interaction affordance handler after automatic handlers to overwrite them if necceassary
                        _a.sent();
                        return [4 /*yield*/, producer.expose()];
                    case 3:
                        _a.sent();
                        title = producer.getThingDescription().title.toLowerCase();
                        this.TDurl = "http://" + this.thingConfig.staticAddress + ":" + this.thingConfig.TDPort + "/" + title;
                        console.info("TD is exposed at " + this.TDurl);
                        return [2 /*return*/, producer];
                }
            });
        });
    };
    AgentThing.prototype.fetchAgentTD = function (TD2Consume) {
        return __awaiter(this, void 0, void 0, function () {
            var td, otherAgent;
            return __generator(this, function (_a) {
                switch (_a.label) {
                    case 0: return [4 /*yield*/, this.wotHelper.fetch(TD2Consume)];
                    case 1:
                        td = _a.sent();
                        return [4 /*yield*/, this.woT.consume(td)];
                    case 2:
                        otherAgent = _a.sent();
                        return [2 /*return*/, [td, otherAgent]];
                }
            });
        });
    };
    AgentThing.prototype.consumeTargetAgent = function (targetAgentId) {
        return __awaiter(this, void 0, void 0, function () {
            var targetAgent, _a, td, otherAgent;
            return __generator(this, function (_b) {
                switch (_b.label) {
                    case 0:
                        targetAgent = this.knownAgents[targetAgentId];
                        if (targetAgent == undefined) {
                            console.log("Unknown target agent: ", targetAgentId);
                            return [2 /*return*/, [null, null]];
                        }
                        return [4 /*yield*/, this.fetchAgentTD(targetAgent)];
                    case 1:
                        _a = _b.sent(), td = _a[0], otherAgent = _a[1];
                        return [2 /*return*/, [td, otherAgent]];
                }
            });
        });
    };
    AgentThing.prototype.handleAgentRequests = function (agentMessage) {
        return __awaiter(this, void 0, void 0, function () {
            var customHandled, keyword_1, performative, path_1, targetAgentName_1, _a, td, otherAgent, AgentBeliefName_1, observe, _b, ia2invoke, property2read_1, answer, value, subscription, subscription, subscription, _c, ia2invoke, property2read, answer, value, targetAgentName, _d, td, otherAgent, _e, ia2invoke, action2invoke, value, data, output, error_3, targetAgentName, _f, td, otherAgent, keyword_2, value, _g, ia2invoke, property2write, content, targetAgentName_2, _h, td, otherAgent, planTrigger_1, content, data, PlanObjects, targetAgent, _j, td, otherAgent;
            var _this = this;
            var _k, _l, _m, _o, _p, _q, _r, _s, _t, _u, _v, _w, _x, _y, _z, _0;
            return __generator(this, function (_1) {
                switch (_1.label) {
                    case 0:
                        // handles the requests from the agent artifact behind the local websocket
                        // forwards the agent requests to other TDs
                        if (this.logginLevel > 2) {
                            console.log("Received message from local agent: ", agentMessage);
                        }
                        return [4 /*yield*/, this.handleCustomAgentRequests(agentMessage)];
                    case 1:
                        customHandled = _1.sent();
                        if (!(customHandled == false)) return [3 /*break*/, 48];
                        keyword_1 = (_l = (_k = agentMessage.content) === null || _k === void 0 ? void 0 : _k.keyword) !== null && _l !== void 0 ? _l : false;
                        performative = agentMessage.performative;
                        if (!((performative == "askOne" || "askAll" || "achieve" || "tell" || "askHow") && keyword_1 != false)) return [3 /*break*/, 45];
                        path_1 = (_o = (_m = agentMessage.content) === null || _m === void 0 ? void 0 : _m.path) !== null && _o !== void 0 ? _o : false;
                        if (!(performative == "askOne")) return [3 /*break*/, 20];
                        targetAgentName_1 = agentMessage.targetAgentId;
                        return [4 /*yield*/, this.consumeTargetAgent(targetAgentName_1)];
                    case 2:
                        _a = _1.sent(), td = _a[0], otherAgent = _a[1];
                        if (otherAgent == null) {
                            console.log("Error: Unknown target agent: ", targetAgentName_1);
                            return [2 /*return*/];
                        }
                        AgentBeliefName_1 = (_q = (_p = agentMessage.content) === null || _p === void 0 ? void 0 : _p.AgentBeliefName) !== null && _q !== void 0 ? _q : keyword_1;
                        observe = (_s = (_r = agentMessage.content) === null || _r === void 0 ? void 0 : _r.observe) !== null && _s !== void 0 ? _s : null;
                        if (!(observe == true)) return [3 /*break*/, 11];
                        return [4 /*yield*/, this.findIA2InvokeInTD(td, keyword_1, path_1)];
                    case 3:
                        _b = _1.sent(), ia2invoke = _b[0], property2read_1 = _b[1];
                        if (this.logginLevel > 0) {
                            console.log("Observing: ", ia2invoke, ":", property2read_1, " for my name: ", keyword_1);
                        }
                        if (!(ia2invoke == "properties")) return [3 /*break*/, 8];
                        return [4 /*yield*/, otherAgent.readProperty(property2read_1)];
                    case 4:
                        answer = _1.sent();
                        return [4 /*yield*/, answer.value()];
                    case 5:
                        value = _1.sent();
                        return [4 /*yield*/, this.addBelief2Agent(AgentBeliefName_1, value, false, targetAgentName_1)];
                    case 6:
                        _1.sent();
                        return [4 /*yield*/, otherAgent.observeProperty(property2read_1, function (data) { return __awaiter(_this, void 0, void 0, function () {
                                var value;
                                return __generator(this, function (_a) {
                                    switch (_a.label) {
                                        case 0: return [4 /*yield*/, data.value()];
                                        case 1:
                                            value = _a.sent();
                                            if (this.logginLevel > 0) {
                                                console.log("Observed ", property2read_1, " for my name: ", keyword_1, ": ", value);
                                            }
                                            return [4 /*yield*/, this.addBelief2Agent(AgentBeliefName_1, value, false, targetAgentName_1)];
                                        case 2:
                                            _a.sent();
                                            return [2 /*return*/];
                                    }
                                });
                            }); })];
                    case 7:
                        subscription = _1.sent();
                        // store the subscription to be able to stop it later
                        if (this.subscriptions[targetAgentName_1] == undefined) {
                            this.subscriptions[targetAgentName_1] = {};
                        }
                        this.subscriptions[targetAgentName_1][keyword_1] = subscription;
                        return [3 /*break*/, 10];
                    case 8:
                        if (!(ia2invoke == "events")) return [3 /*break*/, 10];
                        return [4 /*yield*/, otherAgent.subscribeEvent(property2read_1, function (data) { return __awaiter(_this, void 0, void 0, function () {
                                var value;
                                return __generator(this, function (_a) {
                                    switch (_a.label) {
                                        case 0: return [4 /*yield*/, data.value()];
                                        case 1:
                                            value = _a.sent();
                                            if (this.logginLevel > 0) {
                                                console.log("Got event ", keyword_1, ": ", value);
                                            }
                                            return [4 /*yield*/, this.addBelief2Agent(AgentBeliefName_1, value, false, targetAgentName_1)];
                                        case 2:
                                            _a.sent();
                                            return [2 /*return*/];
                                    }
                                });
                            }); })];
                    case 9:
                        subscription = _1.sent();
                        if (this.subscriptions[targetAgentName_1] == undefined) {
                            this.subscriptions[targetAgentName_1] = {};
                        }
                        this.subscriptions[targetAgentName_1][keyword_1] = subscription;
                        _1.label = 10;
                    case 10: return [3 /*break*/, 19];
                    case 11:
                        if (!(observe == false && this.subscriptions[targetAgentName_1][keyword_1] != undefined)) return [3 /*break*/, 14];
                        // stop observing
                        // ToDo: handle if not observing
                        if (this.logginLevel > 0) {
                            console.log("Stop observing: ", keyword_1);
                        }
                        subscription = this.subscriptions[targetAgentName_1][keyword_1];
                        return [4 /*yield*/, subscription.stop()];
                    case 12:
                        _1.sent();
                        return [4 /*yield*/, this.subscriptions[targetAgentName_1][keyword_1].stop()];
                    case 13:
                        _1.sent();
                        return [3 /*break*/, 19];
                    case 14: return [4 /*yield*/, this.findIA2InvokeInTD(td, keyword_1, path_1)];
                    case 15:
                        _c = _1.sent(), ia2invoke = _c[0], property2read = _c[1];
                        return [4 /*yield*/, otherAgent.readProperty(property2read)];
                    case 16:
                        answer = _1.sent();
                        return [4 /*yield*/, answer.value()];
                    case 17:
                        value = _1.sent();
                        return [4 /*yield*/, this.addBelief2Agent(AgentBeliefName_1, value, false, targetAgentName_1)];
                    case 18:
                        _1.sent();
                        _1.label = 19;
                    case 19: return [3 /*break*/, 44];
                    case 20:
                        if (!(performative == "askAll")) return [3 /*break*/, 21];
                        Object.entries(this.knownAgents).forEach(function (_a) { return __awaiter(_this, [_a], void 0, function (_b) {
                            var AgentBeliefName, _c, td, otherAgent, _d, ia2invoke, property2read, answer, value;
                            var _e, _f;
                            var agentName = _b[0], agenturl = _b[1];
                            return __generator(this, function (_g) {
                                switch (_g.label) {
                                    case 0:
                                        AgentBeliefName = (_f = (_e = agentMessage.content) === null || _e === void 0 ? void 0 : _e.AgentBeliefName) !== null && _f !== void 0 ? _f : keyword_1;
                                        return [4 /*yield*/, this.consumeTargetAgent(agentName)];
                                    case 1:
                                        _c = _g.sent(), td = _c[0], otherAgent = _c[1];
                                        return [4 /*yield*/, this.findIA2InvokeInTD(td, keyword_1, path_1)];
                                    case 2:
                                        _d = _g.sent(), ia2invoke = _d[0], property2read = _d[1];
                                        return [4 /*yield*/, otherAgent.readProperty(property2read)];
                                    case 3:
                                        answer = _g.sent();
                                        return [4 /*yield*/, answer.value()];
                                    case 4:
                                        value = _g.sent();
                                        return [4 /*yield*/, this.addBelief2Agent(AgentBeliefName, value, false, agentName)];
                                    case 5:
                                        _g.sent();
                                        return [2 /*return*/];
                                }
                            });
                        }); });
                        return [3 /*break*/, 44];
                    case 21:
                        if (!(performative == "achieve")) return [3 /*break*/, 32];
                        if (this.logginLevel > 0) {
                            console.log("Achieve: ", agentMessage.content);
                        }
                        targetAgentName = agentMessage.targetAgentId;
                        return [4 /*yield*/, this.consumeTargetAgent(targetAgentName)];
                    case 22:
                        _d = _1.sent(), td = _d[0], otherAgent = _d[1];
                        if (otherAgent == null) {
                            console.log("Achieve: ", agentMessage.content, "failed. Unknown target agent: ", targetAgentName);
                            return [2 /*return*/];
                        }
                        return [4 /*yield*/, this.findIA2InvokeInTD(td, keyword_1, path_1)];
                    case 23:
                        _e = _1.sent(), ia2invoke = _e[0], action2invoke = _e[1];
                        value = (_u = (_t = agentMessage.content) === null || _t === void 0 ? void 0 : _t.value) !== null && _u !== void 0 ? _u : null;
                        data = null;
                        if (!(value != null)) return [3 /*break*/, 25];
                        return [4 /*yield*/, otherAgent.invokeAction(action2invoke, value)];
                    case 24:
                        data = _1.sent();
                        return [3 /*break*/, 27];
                    case 25: return [4 /*yield*/, otherAgent.invokeAction(action2invoke)];
                    case 26:
                        data = _1.sent();
                        _1.label = 27;
                    case 27:
                        output = null;
                        _1.label = 28;
                    case 28:
                        _1.trys.push([28, 30, , 31]);
                        return [4 /*yield*/, data.value()];
                    case 29:
                        output = _1.sent();
                        return [3 /*break*/, 31];
                    case 30:
                        error_3 = _1.sent();
                        console.log("Achieve: ", agentMessage.content, " failed. Output: ", output);
                        return [3 /*break*/, 31];
                    case 31:
                        if (this.logginLevel > 0) {
                            console.log("Achieve: ", agentMessage.content, "at ", targetAgentName, " was successful. Output: ", output);
                        }
                        this.addBelief2Agent(keyword_1, output, false, targetAgentName);
                        return [3 /*break*/, 44];
                    case 32:
                        if (!(performative == "tell")) return [3 /*break*/, 39];
                        targetAgentName = agentMessage.targetAgentId;
                        if (this.parsedTD.properties[keyword_2] != undefined && targetAgentName == "") {
                            this.producer.emitPropertyChange(keyword_2, agentMessage.content.value);
                            return [2 /*return*/];
                        }
                        return [4 /*yield*/, this.consumeTargetAgent(targetAgentName)];
                    case 33:
                        _f = _1.sent(), td = _f[0], otherAgent = _f[1];
                        if (otherAgent == null) {
                            console.log("Tell failed. Unknown target agent: ", targetAgentName);
                            return [2 /*return*/];
                        }
                        keyword_2 = (_w = (_v = agentMessage.content) === null || _v === void 0 ? void 0 : _v.keyword) !== null && _w !== void 0 ? _w : null;
                        value = (_y = (_x = agentMessage.content) === null || _x === void 0 ? void 0 : _x.value) !== null && _y !== void 0 ? _y : null;
                        return [4 /*yield*/, this.findIA2InvokeInTD(td, keyword_2, path_1)];
                    case 34:
                        _g = _1.sent(), ia2invoke = _g[0], property2write = _g[1];
                        if (!(ia2invoke == "properties" && property2write != null && value != null)) return [3 /*break*/, 36];
                        if (this.logginLevel > 0) {
                            console.log("Writing property: ", property2write, " with value: ", value);
                        }
                        return [4 /*yield*/, otherAgent.writeProperty(property2write, value)];
                    case 35:
                        _1.sent();
                        return [3 /*break*/, 38];
                    case 36:
                        content = void 0;
                        if (value == null) {
                            content = { "keyword": keyword_2 };
                        }
                        else {
                            if (Array.isArray(value)) {
                                content = { "belief": keyword_2, "values": value };
                            }
                            else {
                                content = { "belief": keyword_2, "values": [value] };
                            }
                        }
                        if (this.logginLevel > 0) {
                            console.log("Invoking tell action with content: ", content);
                        }
                        return [4 /*yield*/, otherAgent.invokeAction("tell", content)];
                    case 37:
                        _1.sent();
                        _1.label = 38;
                    case 38: return [3 /*break*/, 44];
                    case 39:
                        if (!(performative == "askHow")) return [3 /*break*/, 43];
                        targetAgentName_2 = agentMessage.targetAgentId;
                        return [4 /*yield*/, this.consumeTargetAgent(targetAgentName_2)];
                    case 40:
                        _h = _1.sent(), td = _h[0], otherAgent = _h[1];
                        if (otherAgent == null) {
                            console.log("Unknown target agent: ", targetAgentName_2);
                            return [2 /*return*/];
                        }
                        planTrigger_1 = (_0 = (_z = agentMessage.content) === null || _z === void 0 ? void 0 : _z.keyword) !== null && _0 !== void 0 ? _0 : null;
                        planTrigger_1 = (0, plan_translator_1.planLabel2OntologyLabel)(planTrigger_1, this.additionalOntology);
                        content = { "planLabel": planTrigger_1,
                            "targetTDAdress": this.TDurl
                        };
                        return [4 /*yield*/, otherAgent.invokeAction("askHow", content)];
                    case 41:
                        data = _1.sent();
                        return [4 /*yield*/, data.value()];
                    case 42:
                        PlanObjects = _1.sent();
                        if (PlanObjects.empty) {
                            if (this.logginLevel > 0) {
                                console.log("No plans found for planLabel: ", planTrigger_1);
                            }
                            return [2 /*return*/];
                        }
                        PlanObjects.forEach(function (planObject) {
                            if (_this.logginLevel > 1) {
                                console.log("Received Plan Object for ", planTrigger_1);
                            }
                            else if (_this.logginLevel > 0) {
                                console.log("Received Plan Object for ", planTrigger_1, ": ", planObject);
                            }
                            var planString = (0, plan_translator_1.TargetPlanObject2CommandString)(planObject, _this.additionalOntology);
                            var content = { "keyword": "askHowResponse", "planString": planString }; // Creating content for each plan
                            _this.agentConnector.forward2Agent(new agentConnector_1.AgentMessage("askHowResponse", targetAgentName_2, "", content));
                        });
                        return [2 /*return*/];
                    case 43:
                        console.log("Unknown performative: ", performative);
                        _1.label = 44;
                    case 44: return [3 /*break*/, 48];
                    case 45:
                        targetAgent = agentMessage.targetAgentId;
                        if (targetAgent == undefined) {
                            console.log("Unknown target agent: ", agentMessage.targetAgentId);
                            return [2 /*return*/];
                        }
                        return [4 /*yield*/, this.fetchAgentTD(targetAgent)];
                    case 46:
                        _j = _1.sent(), td = _j[0], otherAgent = _j[1];
                        // console.log("Fetched TD of target agent: ", td.title, " invoking: ", data.content);
                        return [4 /*yield*/, otherAgent.invokeAction("send", {
                                performative: agentMessage.performative,
                                sourceAgentId: agentMessage.sourceAgentId,
                                targetAgentId: agentMessage.targetAgentId,
                                content: agentMessage.content,
                                messageId: -1,
                            })];
                    case 47:
                        // console.log("Fetched TD of target agent: ", td.title, " invoking: ", data.content);
                        _1.sent();
                        _1.label = 48;
                    case 48: return [2 /*return*/];
                }
            });
        });
    };
    AgentThing.prototype.addBelief2Agent = function (beliefName_1, values_1) {
        return __awaiter(this, arguments, void 0, function (beliefName, values, response_expected, source) {
            var agentMessage, data, response;
            var _a, _b;
            if (response_expected === void 0) { response_expected = false; }
            if (source === void 0) { source = "unknown"; }
            return __generator(this, function (_c) {
                switch (_c.label) {
                    case 0:
                        agentMessage = new agentConnector_1.AgentMessage("tell", source, "", { "keyword": beliefName, "values": values });
                        if (!(response_expected == true)) return [3 /*break*/, 2];
                        return [4 /*yield*/, this.agentConnector.send2AgentAndWaitForResponse(agentMessage)];
                    case 1:
                        data = _c.sent();
                        response = (_b = (_a = data.content) === null || _a === void 0 ? void 0 : _a.value) !== null && _b !== void 0 ? _b : null;
                        return [2 /*return*/, response];
                    case 2: return [4 /*yield*/, this.agentConnector.send2Agent(agentMessage)];
                    case 3:
                        _c.sent();
                        _c.label = 4;
                    case 4: return [2 /*return*/];
                }
            });
        });
    };
    AgentThing.prototype.addAchieveGoal2Agent = function (goalName_1, values_1) {
        return __awaiter(this, arguments, void 0, function (goalName, values, response_expected, source) {
            var agentMessage, data, response;
            var _a, _b;
            if (response_expected === void 0) { response_expected = false; }
            if (source === void 0) { source = "unknown"; }
            return __generator(this, function (_c) {
                switch (_c.label) {
                    case 0:
                        agentMessage = new agentConnector_1.AgentMessage("achieve", source, "", { "keyword": goalName, "values": values });
                        if (!(response_expected == true)) return [3 /*break*/, 2];
                        return [4 /*yield*/, this.agentConnector.send2AgentAndWaitForResponse(agentMessage)];
                    case 1:
                        data = _c.sent();
                        response = (_b = (_a = data.content) === null || _a === void 0 ? void 0 : _a.value) !== null && _b !== void 0 ? _b : null;
                        return [2 /*return*/, response];
                    case 2: return [4 /*yield*/, this.agentConnector.send2Agent(agentMessage)];
                    case 3:
                        _c.sent();
                        return [2 /*return*/, null];
                }
            });
        });
    };
    AgentThing.prototype.addTestGoal2Agent = function (goalName_1) {
        return __awaiter(this, arguments, void 0, function (goalName, values, response_expected, source) {
            var agentMessage, data, response;
            var _a, _b;
            if (values === void 0) { values = []; }
            if (response_expected === void 0) { response_expected = false; }
            if (source === void 0) { source = "unknown"; }
            return __generator(this, function (_c) {
                switch (_c.label) {
                    case 0:
                        agentMessage = new agentConnector_1.AgentMessage("ask", source, "", { "keyword": goalName, "values": values });
                        if (!(response_expected == true)) return [3 /*break*/, 2];
                        return [4 /*yield*/, this.agentConnector.send2AgentAndWaitForResponse(agentMessage)];
                    case 1:
                        data = _c.sent();
                        response = (_b = (_a = data.content) === null || _a === void 0 ? void 0 : _a.value) !== null && _b !== void 0 ? _b : null;
                        return [2 /*return*/, response];
                    case 2: return [4 /*yield*/, this.agentConnector.send2Agent(agentMessage)];
                    case 3:
                        _c.sent();
                        return [2 /*return*/, null];
                }
            });
        });
    };
    AgentThing.prototype.readProperty = function (ThingURL, keyword, path) {
        return __awaiter(this, void 0, void 0, function () {
            var _a, td, otherAgent, _b, ia2invoke, property2read, answer, value, content;
            return __generator(this, function (_c) {
                switch (_c.label) {
                    case 0: return [4 /*yield*/, this.fetchAgentTD(ThingURL)];
                    case 1:
                        _a = _c.sent(), td = _a[0], otherAgent = _a[1];
                        return [4 /*yield*/, this.findIA2InvokeInTD(td, keyword, path)];
                    case 2:
                        _b = _c.sent(), ia2invoke = _b[0], property2read = _b[1];
                        if (!(ia2invoke == "properties")) return [3 /*break*/, 5];
                        return [4 /*yield*/, otherAgent.readProperty(property2read)];
                    case 3:
                        answer = _c.sent();
                        return [4 /*yield*/, answer.value()];
                    case 4:
                        value = _c.sent();
                        content = { "keyword": keyword, "value": value };
                        return [2 /*return*/, content];
                    case 5: return [2 /*return*/, -1];
                }
            });
        });
    };
    AgentThing.prototype.findIA2InvokeInTD = function (td, keyword, path) {
        return __awaiter(this, void 0, void 0, function () {
            var paths;
            return __generator(this, function (_a) {
                paths = this.findKeyValuePath(td, path, keyword);
                // Check if paths is not empty and the first element of paths has at least two elements
                if (paths.length > 0 && paths[0].length > 1) {
                    if (this.logginLevel > 2) {
                        console.log("Found IA2Invoke: ", paths[0][1], " with name: ", paths[0][1], "for keyword: ", keyword, " and path: ", path);
                    }
                    return [2 /*return*/, [paths[0][0], paths[0][1]]];
                }
                else {
                    console.log("No IA2Invoke found for keyword: ", keyword, " and path: ", path);
                    // Return [null, null] if no paths found or paths are not valid
                    return [2 /*return*/, [null, null]];
                }
                return [2 /*return*/];
            });
        });
    };
    AgentThing.prototype.findKeyValuePath = function (obj, key2find, value2find, path) {
        /* find the path to a specific key value pair in a json object.
        
        This can be used to e.g. find which property of a TD is of a specific type of ontology.
        */
        var _this = this;
        if (key2find === void 0) { key2find = ""; }
        if (value2find === void 0) { value2find = ""; }
        if (path === void 0) { path = []; }
        var result = [];
        // Iterate over each property in the object
        Object.entries(obj).forEach(function (_a) {
            var key = _a[0], value = _a[1];
            // Construct a new path for this property
            var newPath = path.concat([key]);
            // Check if this property is the one we're looking for
            if (key2find === "" && value === value2find) {
                result.push(newPath);
            }
            else if (key === key2find && value === value2find) {
                // If it is, add the path to the result set
                result.push(newPath);
            }
            else if (value && typeof value === 'object' && !Array.isArray(value)) {
                if (Object.keys(value).includes(value2find)) {
                    // If the object's key matches value2find, add the path to the result set
                    newPath.push(value2find);
                    result.push(newPath);
                }
                else {
                    // If the value is an object, recurse into it
                    result = result.concat(_this.findKeyValuePath(value, key2find, value2find, newPath));
                }
            }
        });
        return result;
    };
    return AgentThing;
}());
exports.AgentThing = AgentThing;
//# sourceMappingURL=agentThing.js.map