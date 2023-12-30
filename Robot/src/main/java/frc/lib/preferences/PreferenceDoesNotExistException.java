package frc.lib.preferences;

import frc.lib.logging.LogLevel;
import frc.lib.logging.LogManager;

public class PreferenceDoesNotExistException extends RuntimeException {
  private static final long serialVersionUID = 7609312699826252204L;
  public final String key;

  public PreferenceDoesNotExistException(String key, LogManager logger) {
    this(key);
    logger.getMainLog().println(LogLevel.ERROR, "Key " + key + " does not exist in preferences!");
  }

  public PreferenceDoesNotExistException(String key) {
    super("Preferences key " + key + " does not exist!");
    this.key = key;
    System.out.println("Key " + key + " does not exist in preferences!");
  }
}
