package e11;

import java.util.Map;
import jason.architecture.AgArch;
import jason.asSemantics.Message;

/** Count application sends, including list-recipient sends expanded by Jason. */
public class MessageTrace extends AgArch {
    @Override
    public void sendMsg(Message message) throws Exception {
        super.sendMsg(message);
        Trace.record(getAgName(), "message", Map.of(
                "receiver", message.getReceiver(), "performative", message.getIlForce(),
                "content", message.getPropCont().toString(), "message_id", message.getMsgId()));
    }
}
