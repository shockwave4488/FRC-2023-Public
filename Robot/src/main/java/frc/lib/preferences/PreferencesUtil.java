package frc.lib.preferences;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.ObjectMapper;
import org.json.simple.JSONObject;

public class PreferencesUtil {
  // TODO: Change everything to use the Jackson library
  // So the conversion between JSON Simple -> String -> Jackson isn't required
  public static <T> T toObj(JSONObject json, Class<T> clazz) {
    ObjectMapper mapper = new ObjectMapper();
    try {
      return mapper.readValue(json.toJSONString(), clazz);
    } catch (JsonProcessingException e) {
      // Since the json comes from a JSONObject
      // The json must be formatted correctly already
      // So this will only occur if there are differences in fields
      // Which won't randomly start occurring
      throw new RuntimeException("Error reading json", e);
    }
  }
}
