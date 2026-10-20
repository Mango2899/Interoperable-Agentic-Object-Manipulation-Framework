/* An artifact for communicating with the according agentConnector.ts package to include in Thing Descriptions.
 * 
 * Only one agent shall connect to this artifact and only one artifact shall connect to the Websocket of a single Thing.
 * Therefore a broadcast is equivalent to a unicast.
 */

package wotCommunication;

import cartago.*;
import jason.functions.log;

import java.util.List;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;
import org.java_websocket.client.WebSocketClient;
import org.java_websocket.handshake.ServerHandshake;

import java.io.File;
import java.io.IOException;
import java.net.URI;

import com.fasterxml.jackson.annotation.JsonProperty;
import com.fasterxml.jackson.core.JsonProcessingException;
// to read config file
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.ibm.icu.impl.UResource.Array;

import wotCommunication.JasonTermParser;
import jason.asSyntax.*;

import jason.asSyntax.parser.ParseException;


enum AgentInteractionAffordances {
    TELL("tell"),
    REQUEST("request"),
    INFORM("inform"),
    ASK("ask");

    private final String text;

    AgentInteractionAffordances(String text) {
        this.text = text;
    }

    @Override
    public String toString() {
        return this.text;
    }

    public static AgentInteractionAffordances fromString(String text) {
        for (AgentInteractionAffordances affordance : AgentInteractionAffordances.values()) {
            if (affordance.text.equalsIgnoreCase(text)) {
                return affordance;
            }
        }
        throw new IllegalArgumentException("Unknown affordance: " + text);
    }
}


class AgentMessage {
    String performative;
    String sourceAgentId;
    String targetAgentId;
    Object content;
    Number messageId;
    boolean isResponse;

    // ObjectMapper instance for JSON serialization/deserialization
    private static final ObjectMapper objectMapper = new ObjectMapper();

    // Default constructor needed by jackson
    public AgentMessage() {}

    // Constructor with JsonProperty annotations
    public AgentMessage(@JsonProperty("performative") String performative, 
                        @JsonProperty("sourceAgentId") String sourceAgentId, 
                        @JsonProperty("targetAgentId") String targetAgentId, 
                        @JsonProperty("content") Object content,
                        @JsonProperty("messageId") Number messageId,
                        @JsonProperty("isResponse") boolean isResponse) {
        this.performative = performative;
        this.sourceAgentId = sourceAgentId;
        this.targetAgentId = targetAgentId;
        this.content = content;
        this.messageId = messageId;
        this.isResponse = isResponse;
    }

    public static AgentMessage fromString(String agentMessage) {
        try {
            return objectMapper.readValue(agentMessage, AgentMessage.class);
        } catch (IOException e) {
            e.printStackTrace();
            // Optionally, return a default AgentMessage instance or null
            // depending on how you want to handle the error.
            return null;
        }
    }
    @Override
    public String toString() {
        try {
            return objectMapper.writeValueAsString(this);
        } catch (JsonProcessingException e) {
            e.printStackTrace();
            return null;
        }
    }


    // Add Setterns and Getters, needed to parse JSON
    // Add getters
    public String getPerformative() {
        return this.performative;
    }

    public String getSourceAgentId() {
        return this.sourceAgentId;
    }

    public String getTargetAgentId() {
        return this.targetAgentId;
    }

    public Object getContent() {
        return this.content;
    }

    public Number getMessageId() {
        return this.messageId;
    }

    public boolean getIsResponse() {
        return this.isResponse;
    }

    // Setter methods
    public void setPerformative(String performative) { this.performative = performative; }
    public void setSourceAgentId(String sourceAgentId) { this.sourceAgentId = sourceAgentId; }
    public void setTargetAgentId(String targetAgentId) { this.targetAgentId = targetAgentId; }
    public void setContent(Object content) { this.content = content; }
    public void setMessageId(Number messageId) { this.messageId = messageId; }
    public void setIsResponse(boolean isResponse) { this.isResponse = isResponse; }
}

public class wotCommunicator extends Artifact{
    private boolean showMessagesExchange = false;
    private WebSocketClient wsClient;
    private String Name;
    // A closed socket is one fact, however many requests run into it. Printing it
    // per request turned a dead layer into thousands of console writes, and the
    // MAS console is synchronous -- so the arm that had already failed was slowing
    // down the arms that were still working.
    private boolean layerDownReported = false;
    private long droppedSends = 0;
    // Which request was dropped most often while the layer was down. A count on
    // its own says an arm is looping; the keyword says what it is looping on, and
    // without it the only way to find out is to guess.
    private final Map<String, Long> droppedByKeyword = new HashMap<>();

    void init(String device) {
        this.Name = device;
        int websocketPort = readConfig(device);
        if (websocketPort == -1) {
            log("["+this.Name+"] " + "WebSocket Port not found. Client not created. Please check the configuration file.");
            return;
        }

        try {
            wsClient = new WebSocketClient(new URI("ws://localhost:" + websocketPort)) {
                @Override
                public void onMessage(String message) {
                    if (message.trim().startsWith("{")) {
                        AgentMessage agentMessage = AgentMessage.fromString(message);
                        // execute in internal operation to not disturb workflow of Cartago when signaling
                        execInternalOp("handleIncomingAgentMessage", agentMessage);
                    } else {
                        log("Non-JSON message received: " + message);
                    }
                    
                }

                @Override
                public void onOpen(ServerHandshake handshake) {
                    log("WebSocket connection opened");
                    // A layer that comes back is news too, and the next outage has
                    // to be reported rather than suppressed as a repeat of this one.
                    if (droppedSends > 0) {
                        log("[" + Name + "] lower layer reconnected after " + droppedSends
                            + " dropped request(s), by request: " + droppedByKeyword);
                    }
                    layerDownReported = false;
                    droppedSends = 0;
                    droppedByKeyword.clear();
                }

                @Override
                public void onClose(int code, String reason, boolean remote) {
                    log("WebSocket connection closed");
                }

                @Override
                public void onError(Exception ex) {
                    ex.printStackTrace();
                }
            };
            wsClient.connect();
        } catch (Exception e) {
            e.printStackTrace();
        }
        log("["+this.Name+"] " + "WebSocket client created");
    }

    // The action name inside an achieve request, for reporting only.
    private static String requestKeyword(Object content) {
        if (content instanceof Map) {
            Object keyword = ((Map<?, ?>) content).get("keyword");
            if (keyword != null) return String.valueOf(keyword);
        }
        return "unknown";
    }

    private String mapToKeyValueString(Map<String, Object> mapVal) {
        StringBuilder sb = new StringBuilder();
        boolean first = true;
        for (Map.Entry<String, Object> e : mapVal.entrySet()) {
            if (!first) {
                sb.append(",");
            }
            first = false;
            String key = e.getKey();      // e.g. "drinkId"
            Object valObj = e.getValue(); // e.g. "cappuccino" or 2
    
            // Convert the value to a string
            String valStr = valueToString(valObj);
            
            // Produce "key(value)" style, e.g. "drinkId(cappuccino)"
            sb.append(key).append("(").append(valStr).append(")");
        }
        return sb.toString();
    }
    
    private String valueToString(Object val) {
        if (val == null) {
            return "null";
        } else if (val instanceof String) {
            // e.g. "cappuccino"
            // maybe lowercase the first char, similar to your code
            String str = (String) val;
            if (!str.isEmpty()) {
                str = str.substring(0,1).toLowerCase() + str.substring(1);
            }
            return str;
        } else if (val instanceof Number || val instanceof Boolean) {
            // e.g. 2 or true
            return val.toString();
        } else {
            // fallback
            return val.toString(); 
        }
    }
    
    
    String generateValueTerm(Object valuesObj, Number messageID) {
        // create a string term of the values and the message ID that can be used to add the values to the term to add to the agents belief base
        String valuesAsString = "";

        if (valuesObj != null) {
            // First parse the values as string if they exist
            List<Object> values = null; 
            if (valuesObj instanceof List) {
                values = (List<Object>) valuesObj;
                if (! (values == null || values.isEmpty())) {
                    StringBuilder valuesPart = new StringBuilder();
                    for (Object value : values) {
                        if (valuesPart.length() > 0) {
                            valuesPart.append(", ");
                        }
                        // Check if the value is a string and modify it
                        if (value instanceof String) {
                            String stringValue = (String) value;
                            if (!stringValue.isEmpty()) {
                                stringValue = stringValue.substring(0, 1).toLowerCase() + stringValue.substring(1);
                            }
                            valuesPart.append(stringValue);
                        } else {
                            valuesPart.append(value.toString()); // Convert each value to a string for concatenation
                        }
                    }
                    valuesAsString += valuesPart.toString();
                }
            } else if (valuesObj instanceof String) {
                if (! (valuesObj == null && valuesObj.equals(""))) {
                    valuesAsString += (String) valuesObj;
                }
            } else if (valuesObj instanceof Number) {
                Integer valueAsInt = ((Number) valuesObj).intValue();
                if (! (valueAsInt == null)) {
                    valuesAsString += (Number) valuesObj;
                }
            
            } else if (valuesObj instanceof Map) {
                // parse the map into "key(val), key(val2)" style
                @SuppressWarnings("unchecked")
                Map<String, Object> mapVal = (Map<String, Object>) valuesObj;
                String mapAsString = mapToKeyValueString(mapVal);
                valuesAsString += mapAsString;
            } 
            else {
                log("Error: Unsupported type for 'values': " + valuesObj.getClass());
            }
        }

        Integer messageIDasInt = ((Number) messageID).intValue();
        // now add the message ID if it is set
        if (messageID != null && messageIDasInt != -1) {
            if (valuesAsString.equals("")) {
                valuesAsString += messageID;
            } else {
                valuesAsString += "," + messageID;
            }    
        }

        if (!valuesAsString.equals("")) {
            valuesAsString = "(" + valuesAsString + ")";
        }
        return valuesAsString;

    }

    @INTERNAL_OPERATION
    void handleIncomingAgentMessage(AgentMessage agentMessage) {   
        // This gets excecuted when an interaction affordance was invoked on the TD and is now forwarded to the agent. 
        // gets executet when a message is received in the ws.
        
        // is done in an internal operation to not disturb the workflow of Cartago when signaling
        
        
        // ToDo: From here: Do appropriate action with the message
        // first see if this message is a response to a request
        if (showMessagesExchange) {
            log("["+this.Name+"] " + "Incoming Message: " + agentMessage.toString());
        }
        Object content = agentMessage.getContent();
        String performative = agentMessage.getPerformative();
        Number messageID = agentMessage.getMessageId();
        Map contentMap = (Map) content;
        String keyword = (String) contentMap.get("keyword");
        String source = (String) agentMessage.getSourceAgentId();        
        Object valuesObj = (Object) contentMap.get("values");
        if (performative.equals("tell")) {
            // add a new belief to the agents belief base
            // signal the inform action to the agent
            Integer messageId = agentMessage.getMessageId().intValue();
            
            // adjust keyword to make agent able to percept it as event. (starting with lowercase letter)
            keyword = keyword.replace(":", "");
            // Ensure the keyword starts with a lowercase letter, otherwise the signal will be recognized as variable instead of belief
            if (!keyword.isEmpty()) {
                keyword = keyword.substring(0, 1).toLowerCase() + keyword.substring(1);
            } else {
                log("Error: Keyword is empty in message: " + agentMessage.toString());
                return;
            }
            // Get the values from the contentMap
            String valuesAString, signalString;
            if (valuesObj == null) {
                valuesAString = "";
                signalString = keyword + "[source(" + source + ")]";
            } else {
                valuesAString = generateValueTerm(valuesObj, messageId);
                signalString = keyword + valuesAString + "[source(" + source + ")]";
            }            
            signal("addBelief", signalString);

        } else if (performative.equals("ask")) {
            // add a a test goal of name keyword with the message id as signal
            // the signal addTestGoal is read by wotPlans.asl and translated into a test goal

            // signal for use in term2string
            // Ensure the keyword starts with a lowercase letter, otherwise it cannot be used as belief
            keyword = keyword.substring(0, 1).toLowerCase() + keyword.substring(1);
            // Object[] valuesObj = (Object[]) contentMap.get("values");
            if (valuesObj == null) {
                String valuesAString = generateValueTerm(valuesObj, messageID);
            }
            String valuesAString = generateValueTerm(valuesObj, messageID);
            String signalString = keyword + valuesAString + "[source(" + source + ")]";
            signal("addTestGoal", signalString);

        } else if (performative.equals("achieve")) {
            // Signal the auto generated action handlers to the agent
            keyword = keyword.substring(0, 1).toLowerCase() + keyword.substring(1);
            // source is notSelf because it is good to know that its not self, but TD cannot provide actual info
            String valuesAString = generateValueTerm(valuesObj, messageID);
            String signalString = keyword + valuesAString + "[source(" + source + ")]";
            signal("addAchieveGoal2Agent", signalString); 

        } else if (performative.equals("askHowResponse")) {
            // if the answer is a plan which I got by asking another agent using askHow
            String plan = (String) contentMap.get("planString");
            signal("askHowResponse", plan, agentMessage.getSourceAgentId());

        } else if (performative.equals("askHow")) {
            // if another agent asks me for a plan
            String planTrigger = (String) contentMap.get("keyword");
            signal("askHowReceived", planTrigger, agentMessage.getMessageId());
        }       
        else {
            log("Unknown agent message performative type: " + performative);
        }
    }

    @OPERATION
    public void wotSend(String performative, String targetAgentName, Object content, int messageId) {
        AgentId agentID = this.getCurrentOpAgentId();
        if (showMessagesExchange) {
            log("["+this.Name+"] " + "Agent: " + agentID + " sent: " + content);
        }

        boolean isResponse = false;
        if (messageId != -1) { isResponse = true;}

        if (wsClient != null && wsClient.isOpen()) {
            AgentMessage agentMessage = new AgentMessage(performative, agentID.getAgentName(), targetAgentName, content, messageId, isResponse);
            wsClient.send(agentMessage.toString());
        } else {
            // The request was never sent, so the operation did not happen and must
            // not report that it did.
            //
            // Returning normally here made every caller wait out its own deadline
            // for an answer that could not come. An arm whose AgentThing is down
            // spent 30s discovering it could not scan for neighbours, 20s
            // discovering it could not look for task sites, and then made every
            // neighbour that asked it about a route wait a further 30s for a
            // belief that was never going to be asserted -- three separate
            // timeouts standing in for one fact the artifact knew immediately.
            // Every caller of this already has a failure plan; none of them had a
            // way to be told.
            //
            // Scope is exactly right: wotAchieve always targets "ownLayer1", this
            // arm's own lower layer, while agent-to-agent traffic goes through
            // Jason's own .send. So this fails layer calls and nothing else.
            droppedSends++;
            String keyword = requestKeyword(content);
            droppedByKeyword.merge(keyword, 1L, Long::sum);
            if (!layerDownReported) {
                layerDownReported = true;
                log("[" + this.Name + "] lower layer is not connected; " + performative + " "
                    + keyword + " to " + targetAgentName
                    + " was not sent. Further attempts are counted, not printed.");
                // Told to the agent ONCE, as a fact about itself rather than about
                // this request. Without it an arm whose layer died after startup had
                // no way to reach the state it reaches when the layer is dead AT
                // startup: it kept calling, kept failing, and never declared itself
                // out of service, so the rest of the cell went on treating it as a
                // possible route.
                signal("layerUnreachable", this.Name, performative);
            } else if (droppedSends % 200 == 0) {
                log("[" + this.Name + "] lower layer still not connected; " + droppedSends
                    + " request(s) dropped so far, by request: " + droppedByKeyword);
                // Told again, and this time it means something different: the arm
                // has ALREADY declared itself out of service and is still issuing
                // commands, which is a loop it cannot see from the inside. The
                // agent's own plan for this reads the second signal as that, and
                // stops. Without it, one arm's loop ran 95000 dropped requests
                // deep and slowed down the arms that were still working.
                signal("layerUnreachable", this.Name, keyword);
            }
            failed("The lower layer of " + this.Name + " is not connected, so " + performative
                + " to " + targetAgentName + " was not sent");
        }
    }

    @OPERATION
    public void wotSend(String performative, String targetAgentName, Object content) {
        wotSend(performative, targetAgentName, content, -1);
    }

   
    @OPERATION
    public void wotSendResponse(String performative, String targetAgentName, Object content, int messageId) {
        /* this is needed to answer to a request.
         * With the messageID the TD handler can identify to which request this response is the answer.
         * This is necessary to make it able to e.g. deliver a value on a readProperty request.
         */
        this.wotSend(performative, targetAgentName, content, messageId);
    }

    @OPERATION void wotSendTestResponse(Object value, int messageId) {
        /* this is needed to answer to a request.
         * With the messageID the TD handler can identify to which request this response is the answer.
         * This is necessary to make it able to e.g. deliver a value on a readProperty request.
         */
        Map<String, Object> content = new HashMap<>();
        content.put("value", value);
        this.wotSend("testResponse", "", content, messageId);
    }

    @OPERATION void wotTell(String targetAgentName, String keyword, Object value) {
        /* tell the agent targetAgentName the value of the keyword
         * The keyword without the ":" is the believe which gets added as a response to the agent.
         */
        Map<String, Object> content = new HashMap<>();
        content.put("keyword", keyword);
        content.put("value", value);
        wotSend("tell", targetAgentName, content);
    }

    @OPERATION
    public void wotAskOne(String targetAgentName, String keyword, String path, Boolean observe, String myName) {
        /* ask as specific agent about the value of some property described by path
         * The path a string with with the elements separated by a ",".
         * e.g. "saref:measuresProperty"
         * the keyword without the ":" is the believe which gets added as a response to the agent.
         * myName: the name of the belief as which I want to receive the answer (+myName(Value))
         */
        Map<String, Object> content = new HashMap<>();
        content.put("path", path);
        content.put("keyword", keyword);
        content.put("observe", observe);
        content.put("AgentBeliefName", myName);
        wotSend("askOne", targetAgentName, content);
    }
    
    @OPERATION void wotAskOne(String targetAgentName, String keyword, String path) {
        wotAskOne(targetAgentName, keyword, path, null, null);
    }

    @OPERATION void wotAskOne(String targetAgentName, String keyword, String path, String myName) {
        wotAskOne(targetAgentName, keyword, path, null, myName);
    }

    @OPERATION
    public void wotAskAll(String keyword, String path, boolean observe) {
        /* ask all agents about the value of some property described by path
         * The path is a list of strings, where the first element is the property name and the following elements are the path to the property.
         * e.g. ["saref:Light", "saref:measuresProperty"]
         * the keyword without the ":" is the believe which gets added as a response to the agent.
         */
        Map<String, Object> content = new HashMap<>();
        content.put("path", path);
        content.put("keyword", keyword);
        content.put("observe", observe);
        wotSend("askAll", "all", content);
    }

    @OPERATION
    public void wotAskAll(String keyword, String path) {
        wotAskAll(keyword, path, false);
    }

    // @OPERATION void wotAchieve(String targetAgentName, String keyword, Object value) {
    //     /* Invoke the action keyword of agent targetAgentName without a value (only String, Integer or Boolean are supported as value)
    //      * The keyword without the ":" is the believe which gets added as a response to the agent.
    //      */
    //     Map<String, Object> content = new HashMap<>();
    //     content.put("keyword", keyword);
    //     content.put("value", value);
    //     wotSend("achieve", targetAgentName, content);
    // }
    
    @OPERATION void wotAchieve(String targetAgentName, String keyword) {
        /* Invoke the action keyword of agent targetAgentName without any value.
         * The keyword without the ":" is the believe which gets added as a response to the agent.
         */
        Map<String, Object> content = new HashMap<>();
        content.put("keyword", keyword);
        content.put("value", null);
        wotSend("achieve", targetAgentName, content);
    }

    @OPERATION
    public void wotAchieve(String targetAgentName, String keyword, Object valuesObj) {
        Map<String, Object> content = new HashMap<>();
        content.put("keyword", keyword);

        if (valuesObj instanceof Object[]) {
            content.put("value", new ArrayList<>(Arrays.asList((Object[]) valuesObj)));
        } else if (valuesObj instanceof String) {
            String dataStr = (String) valuesObj;
            Term parsedTerm = null;
            try {
                parsedTerm = ASSyntax.parseTerm(dataStr);
            } catch (ParseException e) {
                log("Could not parseTerm: " + e.getMessage());
            }
            Object parsedValue = parsedTerm != null ? JasonTermParser.parseTerm(parsedTerm) : null;
            content.put("value", parsedValue != null ? parsedValue : dataStr);
        } else {
            content.put("value", valuesObj);
        }

        wotSend("achieve", targetAgentName, content);
    }

    public void log(String msg) {
        System.out.println("[WotComm] " + msg);
     }


    @OPERATION void wotAskHow(String targetAgentName, String planLabel) {
        Map<String, Object> content = new HashMap<>();
        content.put("keyword", planLabel);
        wotSend("askHow", targetAgentName, content);
    }
 
    @OPERATION
    public void wotFindProperty(String keywords) {
        log("Finding property with keywords: " + keywords);
        Map<String, Object> content = new HashMap<>();
        content.put("keywords", keywords);
        content.put("request", "findProperty");
        AgentMessage agentMessage = new AgentMessage("ask", "", "all", content, -1, false);
        wsClient.send(agentMessage.toString());
    }



    private int readConfig(String device) {
        try {
            File file = new File("config.json");
            ObjectMapper mapper = new ObjectMapper();
            JsonNode rootNode = mapper.readTree(file);
            JsonNode deviceNode = rootNode.path(device);
            if (!deviceNode.isMissingNode()) { // Check if the device configuration exists
                int websocketPort = deviceNode.path("websocketPort").asInt();
                System.out.println(device + " WebSocket Port: " + websocketPort);
                return websocketPort;
            } else {
                System.err.println("Configuration for " + device + " not found.");
            }
        } catch (IOException e) {
            e.printStackTrace();
        }
        return -1;
    }
}
