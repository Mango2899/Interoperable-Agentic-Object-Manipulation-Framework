package e11;

import java.util.Arrays;
import java.util.Map;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.Set;
import java.util.TreeSet;
import jason.asSemantics.DefaultInternalAction;
import jason.asSemantics.TransitionSystem;
import jason.asSemantics.Unifier;
import jason.asSyntax.Term;

/** Explicit measurement boundaries in copies of the production plans. */
public class event extends DefaultInternalAction {
    @Override
    public Object execute(TransitionSystem ts, Unifier un, Term[] args) throws Exception {
        Map<String, Object> fields = new LinkedHashMap<>(Map.of(
                "phase", args.length > 1 ? args[1].toString() : "",
                "data", Arrays.stream(args).skip(2).map(Object::toString).toList()));
        if (args[0].toString().equals("phase_end")) {
            var beliefs = new ArrayList<String>();
            for (var belief : ts.getAg().getBB()) {
                if (java.util.Set.of("neighborScanComplete", "neighborScanFailed", "taskSitesReady",
                        "taskSitesFailed", "interactionSpaceFailed", "interactionSpaceCache").contains(belief.getFunctor())) {
                    beliefs.add(belief.toString());
                }
            }
            fields.put("beliefs", beliefs);
        }
        Trace.record(ts.getAgArch().getAgName(), args[0].toString(), fields);
        String cell = System.getenv("WOMAT_E11_CELL");
        if (cell != null && args[0].toString().equals("phase_end")
                && fields.get("phase").equals("neighbour_discovery")) {
            Set<String> tested = new TreeSet<>();
            boolean completed = false;
            for (var belief : ts.getAg().getBB()) {
                if (belief.getFunctor().equals("reachTest")) tested.add(belief.getTerm(0).toString());
                if (belief.getFunctor().equals("neighborScanComplete")) completed = true;
            }
            if (completed) {
                Set<String> expected = expectedPeers(Integer.parseInt(cell), ts.getAgArch().getAgName());
                if (!tested.equals(expected)) {
                    String message = "E11 cell mismatch: expected peers " + expected + ", scanned " + tested
                            + ". Restart every AgentThing with the selected cell's manifest copies.";
                    Trace.record(ts.getAgArch().getAgName(), "cell_mismatch", Map.of("detail", message));
                    System.err.println(message);
                    return false; // Stop this initialization before exchanging clouds or route queries.
                }
            }
        }
        return true;
    }

    public static Set<String> expectedPeers(int cell, String agent) {
        Set<String> peers = new TreeSet<>();
        for (int i = 1; i <= cell; i++) peers.add("robot" + i);
        peers.remove(agent.replaceFirst("_agent$", ""));
        return peers;
    }
}
