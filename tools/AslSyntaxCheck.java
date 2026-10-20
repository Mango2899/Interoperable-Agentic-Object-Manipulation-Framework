// Parses .asl files with the same parser the MAS uses, and says where they break.
//
// A plan file that does not parse does not fail loudly at startup. Jason drops
// the whole file and every agent that included it carries on without ANY of its
// plans, so the first symptom is "Found a goal for which there is no relevant
// plan" for a goal that is plainly written down -- with nothing naming the file,
// the line, or the character that caused it. One Prolog-ism ("=<" where Jason
// wants "<=") took every plan in routePlans.asl out of four agents at once.
//
// Run with Java's single-file launcher, no build step:
//   java -cp <jason-jar> tools/AslSyntaxCheck.java <file.asl> ...

import jason.asSemantics.Agent;
import java.io.File;

public class AslSyntaxCheck {
    public static void main(String[] args) {
        int failures = 0;
        for (String path : args) {
            try {
                // Long plan bodies recurse deeply in the generated parser, so
                // the parse runs on a thread with room for them rather than on
                // main's default stack.
                Agent agent = new Agent();
                // Without this the agent has no belief base, no internal-action
                // table and no plan library, and the parser reports the missing
                // structures instead of the syntax it was asked about.
                agent.initAg();
                // initAg also brings up Jason's mind inspector, which binds a
                // port. A syntax check must not hold one -- least of all the one
                // a running MAS is already on.
                agent.stopAg();
                Throwable[] thrown = new Throwable[1];
                Thread worker = new Thread(null, () -> {
                    try { agent.parseAS(new File(path)); } catch (Throwable error) { thrown[0] = error; }
                }, "parse", 64L * 1024 * 1024);
                worker.start();
                worker.join();
                if (thrown[0] != null) throw thrown[0];
                System.out.println("ok   " + path);
            } catch (Throwable error) {
                failures++;
                String message = error.getMessage();
                System.out.println("FAIL " + path);
                System.out.println("     " + (message == null ? error.toString() : message.replace("\n", "\n     ")));
            }
        }
        System.exit(failures == 0 ? 0 : 1);
    }
}
