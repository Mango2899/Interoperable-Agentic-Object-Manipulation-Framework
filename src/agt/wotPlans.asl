/* this file maps the three possible interactions with the agent TD to agent beliefs / plans.
    * The agent can receive a read property request, an action request or a subscription request.
    * The agent will then call the appropriate plan to handle the request.

    instead of `?asked(Keyword, MessageID).` Better would be: 
    +Keyword(MessageID).
    that would resolve to e.g. ?Brightness(id5).
    This would add the believe with the actual name.

    But unfortunately, the agent does not support dynamic predicates, so we have to use the workaround with ?asked(Keyword, MessageID).

*/




// add the answer to a readProperty as a belief to the agent (with source other agent)
+readPropertyResponse(TermString): true <- 
    if (wotReceivedDebugMsg) {
        .print(" [WotDebug]  Received read property response: ", TermString);
    };
    .term2string(X, TermString);
    +X.

+addBelief(TermString): true <- 
    if (wotReceivedDebugMsg) {
        .print(" [WotDebug]  Received add belief: ", TermString);
    };
    .term2string(X, TermString);
    +X.

+addAchieveGoal(TermString): true <- 
    if (wotReceivedDebugMsg) {
        .print(" [WotDebug]  Received add Achieve Goal: ", TermString);
    };
    .term2string(X, TermString);
    !X.

+addAchieveGoal2Agent(TermString): true <-
    if (wotReceivedDebugMsg) {
        .print(" [WotDebug]  Received add Achieve Goal: ", TermString);
    };
    .term2string(X, TermString);
    !X.

// when a property was read, add testgoal to the agent
+addTestGoal(TermString): true <- 
    if (wotReceivedDebugMsg) {
        .print(" [WotDebug]  Received add Test Goal: ", TermString);
    };
    .term2string(X, TermString);
    ?X.

+actionInvoked(TermString): true <-
    // triggers when an action was invoked on my TD.
    // if the action has an input defined in the TD, !action(Value)
    // if the action has an output defined in the TD !action(MessageID)
    // if the action has both, !action(Value, MessageID)
    // if the action has none, !action
    // always with [source(unknown)] as source (TD invokeAction does not give invoking agent)
    if (wotReceivedDebugMsg) {
        .print(" [WotDebug]  Received action invoked: ", TermString);
    };
    .term2string(X, TermString);
    !X.

+askHowReceived(PlanTrigger, MessageID) : true <- 
    if (wotReceivedDebugMsg) {
        .print(" [WotDebug]  Received askHowReceived for plan trigger: ", PlanTrigger, " with messageID: ", MessageID);
    };
    .print("Got asked for Plan");
    .term2string(PlanLabel, PlanTrigger);
    .relevant_plans(PlanLabel, P);
    if (wotReceivedDebugMsg) {
        .print(" [WotDebug]  Relevant plans: ", P);
    };
    .print(" Found plan with label: ", P);
    wotSendResponse("", "", P, MessageID).  

// response to a askHow action I sent myself, the plans are now added
+askHowResponse(PlanString, Source): true <-
    // first convert the string of plan and source to a term that can be used by the agent
    // the plan has to be a string like e.g. { +!test : true <- .print("Hello World") }
    if (wotReceivedDebugMsg) {
        .print(" [WotDebug]  Received askHowResponse for plan: ", PlanString, " from source: ", Source);
    };
    .print("Got plan from source: ", Source);
    .term2string(X, PlanString);
    .term2string(Y, Source);
    .add_plan(X, Y).

+!wotSetupMsg: true <- 
    // this is the first message the agent receives from the WoT agent
    // it contains the TD of the WoT agent
    // the agent will now add the TD to its beliefs
    if (wotReceivedDebugMsg) {
        .print(" [WotDebug]  Wot Debug Msg on Receive: true");
    } else {
        .print(" [WotDebug]  Wot Debug Msg on Receive: false");
    }.

!wotSetupMsg.
