package frc.lib.logging;

public enum LogLevel {
  HIGH("[High] ", true, false),
  INFO("[Info] ", false, false),
  DEBUG("[Debug] ", false, false),
  ERROR("[Error] ", true, true),
  ERROR_CONSOLE("[Error] ", true, true);

  private final String prefix;
  private final boolean important;
  private final boolean error;

  private LogLevel(String prefix, boolean important, boolean error) {
    this.prefix = prefix;
    this.important = important;
    this.error = error;
  }

  public String getPrefix() {
    return prefix;
  }

  public boolean isImportant() {
    return important;
  }

  public boolean isError() {
    return error;
  }
}
