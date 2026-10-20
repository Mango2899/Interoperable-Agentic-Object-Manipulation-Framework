import jason.asSyntax.parser.as2j;
import jason.asSemantics.Agent;
import java.io.FileReader;
public class ParseCheck {
  public static void main(String[] a) throws Exception {
    int bad = 0;
    for (String f : a) {
      try { Agent ag = new Agent(); ag.initAg(); new as2j(new FileReader(f)).agent(ag); }
      catch (Throwable t) { bad++; System.out.println("FAIL " + f + ": " + t.getMessage()); }
    }
    System.exit(bad == 0 ? 0 : 1);
  }
}
