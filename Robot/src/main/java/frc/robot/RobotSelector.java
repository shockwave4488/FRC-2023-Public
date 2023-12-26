package frc.robot;

import frc.lib.IRobotContainer;
import frc.lib.logging.LeveledSmartDashboard;
import frc.lib.logging.LogLevel;
import frc.lib.logging.LogManager;
import frc.lib.preferences.PreferenceDoesNotExistException;
import frc.lib.preferences.PreferencesParser;
import frc.robot.robotspecifics.bareroborio.BareRoboRIORobotContainer;
import frc.robot.robotspecifics.eruption.EruptionRobotContainer;
import frc.robot.robotspecifics.fluffnado.FluffnadoRobotContainer;
import frc.robot.robotspecifics.spider.SpiderBotContainer;
import frc.robot.robotspecifics.supercell.SupercellRobotContainer;
import frc.robot.robotspecifics.swerve.SwerveRobotContainer;

public class RobotSelector {
  private final PreferencesParser prefs;
  private final LogManager logger;
  private final String robotName;

  public RobotSelector(PreferencesParser prefs, LogManager logger) {
    this.prefs = prefs;
    this.logger = logger;

    String name;
    try {
      name = prefs.getString("RobotName");
    } catch (PreferenceDoesNotExistException e) {
      name = "BareRIO - No RobotName key!";
    }
    robotName = name;
    LeveledSmartDashboard.HIGH.putString("Selected Robot", robotName);
  }

  public IRobotContainer getRobot() {
    return switch (robotName) {
      case "Eruption":
        yield new EruptionRobotContainer(prefs, logger);
      case "Practice":
        yield new SupercellRobotContainer(prefs, logger);
      case "Spider":
        yield new SpiderBotContainer(prefs, logger);
      case "Swerve":
        yield new SwerveRobotContainer(prefs, logger);
      case "BareRoboRIO":
        yield new BareRoboRIORobotContainer(prefs, logger);
      case "Supercell":
        yield new SupercellRobotContainer(prefs, logger);
      case "Fluffnado":
        yield new FluffnadoRobotContainer(prefs, logger);
      default:
        logger.getMainLog().println(LogLevel.ERROR_CONSOLE, "Invalid RobotName value in prefs!");
        throw new RuntimeException("Invalid RobotName value in prefs! ROBOT CANNOT INITIALIZE");
    };
  }
}
