"use strict";
/* Can translate an agent plan into another nameSpace Using two TDs and an ontology.

# To create a plan that can be exported
planString2OntologyPlanObject: translates from first TD to plan structure
OntologyPlanObject2TargetPlanObject: reverses plansString2Ontology translation but should be used with a different TD

# To use a plan that was imported
TargetPlanObject2CommandString: translates OntologyPlan

PlanObject(type: Plan): {
    planLabel: string;
    planInput: string[];
    planSteps: PlanStep[];
}
*/
var __assign = (this && this.__assign) || function () {
    __assign = Object.assign || function(t) {
        for (var s, i = 1, n = arguments.length; i < n; i++) {
            s = arguments[i];
            for (var p in s) if (Object.prototype.hasOwnProperty.call(s, p))
                t[p] = s[p];
        }
        return t;
    };
    return __assign.apply(this, arguments);
};
Object.defineProperty(exports, "__esModule", { value: true });
exports.planString2OntologyPlanObject = planString2OntologyPlanObject;
exports.OntologyPlanObject2TargetPlanObject = OntologyPlanObject2TargetPlanObject;
exports.planLabel2OntologyLabel = planLabel2OntologyLabel;
exports.ontologyLabel2planLabel = ontologyLabel2planLabel;
exports.TargetPlanObject2CommandString = TargetPlanObject2CommandString;
// Example of an additional ontology, hardcoded for simplicity
var additionalOntology = {
    "blue": "ObjectBlue",
    "process": "Process"
};
function translateTerm(term, ontology) {
    return ontology[term] || term;
}
function planString2OntologyPlanObject(planString, td, ontology) {
    var translateCommand = function (command, params) {
        var _a, _b;
        var actionData = td.actions[command];
        if (!actionData) {
            console.error("No action data found for command: ".concat(command));
            return null; // Guard against missing action definitions
        }
        var translatedCommand = actionData["@type"] ? actionData["@type"].split(':').pop() : command;
        var inputs = {};
        if (actionData.input && actionData.input.properties && params.length > 0) {
            params.forEach(function (param, index) {
                var key = Object.keys(actionData.input.properties)[index];
                var propName = actionData.input.properties[key]["@type"] ? actionData.input.properties[key]["@type"].split(':').pop() : key;
                inputs[propName] = param.trim();
            });
            return _a = {}, _a[translatedCommand] = { input: inputs }, _a;
        }
        else {
            // Ensure even actions without parameters are included
            return _b = {}, _b[translatedCommand] = {}, _b;
        }
    };
    // Update the regex if necessary, but it seems fine based on your split result.
    // Ensure regex captures the entire action sequence correctly
    var regex = /{ \+!(\w+)(?:\(([^)]*)\))? <- (.+) }/;
    var match = planString.match(regex);
    if (match) {
        var planLabel = translateTerm(match[1], ontology);
        var planInput = [translateTerm(match[2], ontology)];
        var actions = match[3].split(';').map(function (action) { return action.trim(); });
        var planSteps = actions.map(function (action) {
            // const actionMatch = action.match(/!(\w+)\((.*?)\)/);
            var actionMatch = action.match(/!(\w+)(?:\((.*?)\))?/);
            if (actionMatch) {
                var command = actionMatch[1];
                var params = actionMatch[2] ? actionMatch[2].split(',').map(function (param) { return param.trim(); }) : [];
                return translateCommand(command, params);
            }
            return null;
        }).filter(function (step) { return step != null; });
        return {
            planLabel: planLabel,
            planInput: planInput,
            planSteps: planSteps
        };
    }
    return null;
}
function OntologyPlanObject2TargetPlanObject(inputPlan, td) {
    var outputPlan = __assign(__assign({}, inputPlan), { planSteps: [] });
    inputPlan.planSteps.forEach(function (step) {
        var _a;
        var actionKey = Object.keys(step)[0]; // Original action name
        // Find the action in TD based on @type ending matching the original action name
        var actionData = Object.values(td.actions).find(function (a) { var _a; return (_a = a["@type"]) === null || _a === void 0 ? void 0 : _a.endsWith(actionKey); });
        var newActionKey = actionData ? Object.keys(td.actions).find(function (key) { return td.actions[key] === actionData; }) : actionKey;
        var newStep = (_a = {}, _a[newActionKey] = {}, _a);
        if (step[actionKey].input) {
            newStep[newActionKey].input = {};
            Object.entries(step[actionKey].input).forEach(function (_a) {
                var paramKey = _a[0], paramValue = _a[1];
                var foundProperty = Object.entries(actionData.input.properties).find(function (_a) {
                    var _b;
                    var propKey = _a[0], propVal = _a[1];
                    return (_b = propVal["@type"]) === null || _b === void 0 ? void 0 : _b.endsWith(paramKey);
                });
                var newParamKey = foundProperty ? foundProperty[0] : paramKey;
                newStep[newActionKey].input[newParamKey] = paramValue;
            });
        }
        outputPlan.planSteps.push(newStep);
    });
    return outputPlan;
}
function planLabel2OntologyLabel(command, ontology) {
    // Regular expression to extract the command and parameters with variable prefixes ! or ?
    var regex = /([!?])(\w+)\(([\w, ]+)\)/;
    var matches = command.match(regex);
    if (matches && matches.length === 4) {
        var prefix = matches[1]; // e.g., "!" or "?"
        var commandPart = matches[2]; // e.g., "process"
        var paramPart = matches[3]; // e.g., "blue"
        // Translate the command and parameter parts
        var translatedCommand = translateTerm(commandPart, ontology); // Translate "process"
        var params = paramPart.split(',').map(function (param) {
            return translateTerm(param.trim(), ontology);
        } // Translate each parameter individually
        );
        // Reassemble into the desired format with the original prefix
        return "".concat(prefix).concat(translatedCommand, "(").concat(params.join(', '), ")");
    }
    return command; // Return the original command if it doesn't match expected pattern
}
function ontologyLabel2planLabel(command, ontology) {
    // Create a reverse mapping from the provided ontology
    var reverseOntology = Object.fromEntries(Object.entries(ontology).map(function (_a) {
        var key = _a[0], value = _a[1];
        return [value, key];
    }));
    // Regular expression to extract the command and parameters with variable prefixes ! or ?
    var regex = /([!?])(\w+)\(([\w, ]+)\)/;
    var matches = command.match(regex);
    if (matches && matches.length === 4) {
        var prefix = matches[1]; // e.g., "!" or "?"
        var commandPart = matches[2]; // e.g., "Process"
        var paramPart = matches[3]; // e.g., "ObjectBlue"
        // Translate the command and parameter parts using the reverse ontology
        var originalCommand = reverseOntology[commandPart] || commandPart; // Reverse translate "Process"
        var params = paramPart.split(',').map(function (param) {
            return reverseOntology[param.trim()] || param.trim();
        } // Reverse translate each parameter individually
        );
        // Reassemble into the original format with the original prefix
        return "".concat(prefix).concat(originalCommand, "(").concat(params.join(', '), ")");
    }
    return command; // Return the original command if it doesn't match expected pattern
}
function TargetPlanObject2CommandString(plan, ontology) {
    var reverseOntology = Object.fromEntries(Object.entries(ontology).map(function (_a) {
        var key = _a[0], value = _a[1];
        return [value, key];
    }));
    // Translate the plan label using the ontology
    var planLabel = reverseOntology[plan.planLabel] || plan.planLabel;
    // Translate plan inputs using the ontology
    var planInputs = plan.planInput.map(function (input) { return reverseOntology[input] || input; });
    // Construct the command string from plan steps
    var steps = plan.planSteps.map(function (step) {
        var action = Object.keys(step)[0];
        var details = step[action];
        if (details.input && Object.keys(details.input).length > 0) {
            // Sort keys to maintain consistent order (X_Value, Y_Value, Z_Value)
            var params = Object.entries(details.input)
                .sort(function (_a, _b) {
                var key1 = _a[0];
                var key2 = _b[0];
                return key1.localeCompare(key2);
            })
                .map(function (_a) {
                var param = _a[0], value = _a[1];
                return "".concat(value);
            }).join(',');
            return "!".concat(action, "(").concat(params, ")");
        }
        return "!".concat(action);
    });
    // Assemble the final command string
    var commandString = "{ +!".concat(planLabel, "(").concat(planInputs.join(', '), ") <- ").concat(steps.join('; '), " }");
    return commandString;
}
//# sourceMappingURL=plan_translator.js.map