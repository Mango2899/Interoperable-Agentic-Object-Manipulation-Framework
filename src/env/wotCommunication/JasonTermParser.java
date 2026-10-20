package wotCommunication;

import jason.asSyntax.ListTerm;
import jason.asSyntax.NumberTerm;
import jason.asSyntax.StringTerm;
import jason.asSyntax.Structure;
import jason.asSyntax.Term;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

public final class JasonTermParser {
    private JasonTermParser() {
    }

    public static Object parseTerm(Term term) {
        if (term == null) {
            return null;
        }

        if (term instanceof NumberTerm) {
            try {
                return ((NumberTerm) term).solve();
            } catch (Exception ignored) {
                return term.toString();
            }
        }

        if (term instanceof StringTerm) {
            return ((StringTerm) term).getString();
        }

        if (term instanceof ListTerm) {
            List<Object> values = new ArrayList<>();
            for (Term item : (ListTerm) term) {
                values.add(parseTerm(item));
            }
            return values;
        }

        if (term instanceof Structure) {
            Structure structure = (Structure) term;
            String functor = structure.getFunctor();

            if (structure.getArity() == 0) {
                if ("true".equals(functor)) {
                    return true;
                }
                if ("false".equals(functor)) {
                    return false;
                }
                if ("null".equals(functor)) {
                    return null;
                }
                return functor;
            }

            Map<String, Object> value = new LinkedHashMap<>();
            value.put("functor", functor);

            List<Object> args = new ArrayList<>();
            for (int i = 0; i < structure.getArity(); i++) {
                args.add(parseTerm(structure.getTerm(i)));
            }
            value.put("args", args);

            return value;
        }

        return term.toString();
    }
}
