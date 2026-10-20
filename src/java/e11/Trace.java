package e11;

import com.fasterxml.jackson.databind.ObjectMapper;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardOpenOption;
import java.util.LinkedHashMap;
import java.util.Map;

/** Enabled only in an E11 JVM. No network or simulator access. */
public final class Trace {
    private static final ObjectMapper JSON = new ObjectMapper();
    private static final String FILE = System.getenv("WOMAT_E11_TRACE");

    public static synchronized void record(String agent, String event, Map<String, Object> fields) throws Exception {
        if (FILE == null || FILE.isBlank()) return;
        Map<String, Object> row = new LinkedHashMap<>();
        row.put("t", System.currentTimeMillis());
        row.put("mono_ms", System.nanoTime() / 1e6);
        row.put("agent", agent);
        row.put("event", event);
        row.putAll(fields);
        Files.writeString(Path.of(FILE), JSON.writeValueAsString(row) + "\n",
                StandardOpenOption.CREATE, StandardOpenOption.APPEND);
    }
}
