package frc.lib.misc;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Map;
import java.util.TreeMap;
import java.util.TreeSet;
import java.util.function.Function;
import java.util.stream.Collectors;

public class CollectionUtil {
  private CollectionUtil() {}

  /**
   * Returns the given {@code map} with the keys put in reverse natural order, and zipped back
   * together. For example, {@code Map.of(1, "one", 2, "two", 3, "three")} turns into {@code
   * Map.of(3, "one", 2, "two", 1, "three")}.
   */
  public static <K extends Comparable<?>, V> Map<K, V> reverseMap(Map<K, V> map) {
    List<K> keySet = new ArrayList<>(new TreeSet<K>(Collections.reverseOrder()));
    keySet.addAll(map.keySet());
    Map<K, V> newMap = new TreeMap<>();
    int size = keySet.size();
    for (int i = 0; i < size; i++) {
      newMap.put(keySet.get(i), map.get(keySet.get(size - i - 1)));
    }
    return newMap;
  }

  public static <K, V> Map<K, V> modifyMapValue(Map<K, V> map, Function<V, V> modifier) {
    return map.entrySet().stream()
        .collect(Collectors.toMap(Map.Entry::getKey, entry -> modifier.apply(entry.getValue())));
  }
}
