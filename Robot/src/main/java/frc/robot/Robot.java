// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.umd.cs.findbugs.annotations.SuppressFBWarnings;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.lib.IRobotContainer;
import frc.lib.commands.CommandLogger;
import frc.lib.controlsystems.SimManager;
import frc.lib.dashboard.DashboardServer;
import frc.lib.dashboard.IDashboardSupportedContainer;
import frc.lib.dashboard.packets.PacketProtocolFactory;
import frc.lib.logging.Console;
import frc.lib.logging.LogLevel;
import frc.lib.logging.LogManager;
import frc.lib.misc.MatchUtil;
import frc.lib.misc.MatchUtil.MatchPhase;
import frc.lib.preferences.PreferencesParser;
import frc.lib.sensors.vision.LimelightHelpers;
import java.io.IOException;
import java.util.Optional;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  // There can only ever be 1 driver station
  // And servers can handle multiple connections anyway
  private static DashboardServer dashboard;

  private static void initDashboard(LogManager logger, IRobotContainer container) {
    try {
      PacketProtocolFactory.registerDefaultProtocols(logger);
      dashboard =
          new DashboardServer(
                  logger,
                  container instanceof IDashboardSupportedContainer dashContainer
                      ? Optional.of(dashContainer)
                      : Optional.empty(),
                  PacketProtocolFactory.create(PacketProtocolFactory.NETWORK_TABLE_PROTOCOL))
              .bindToSystem()
              .registerDefaultActions()
              .startPacketHandler()
              .ready();
    } catch (IOException e) {
      logger.getMainLog().println(LogLevel.ERROR, "Error while starting dashboard", e);
      return;
    }

    if (container instanceof IDashboardSupportedContainer dashContainer) {
      dashContainer.onDashboardInit(dashboard);
      dashboard.getWebsite().sendWidgets();
    }
  }

  @SuppressFBWarnings("MS_EXPOSE_REP")
  public static Console getConsole() {
    if (dashboard == null) {
      return Console.VOID;
    }
    return dashboard;
  }

  private Command m_autonomousCommand;
  private final LogManager logger = new LogManager().withDefaultLoggers();
  private final PreferencesParser prefs = new PreferencesParser(logger);
  private final RobotSelector robotSelector = new RobotSelector(prefs, logger);
  private IRobotContainer m_robotContainer;

  {
    LimelightHelpers.init(logger);
  }

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    MatchUtil.findIfRealMatch(logger, isSimulation());

    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = robotSelector.getRobot();
    m_robotContainer.runZeroSensors();
    m_robotContainer.runSetUpTrackables(logger);

    initDashboard(logger, m_robotContainer);

    CommandLogger.getInstance().setLoggerTrackable(logger);
    LiveWindow.enableTelemetry(CommandLogger.getInstance());

    PortForwarder.add(5800, "limelight.local", 5800);
    PortForwarder.add(5801, "limelight.local", 5801);
    PortForwarder.add(5805, "limelight.local", 5805);
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    CommandLogger.getInstance().update();
    logger.update();
    m_robotContainer.runUpdateSmartDashboard();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    m_robotContainer.runOnStop();

    logger.beginPhase(null);
  }

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    // In practice matches, logger.init will only be initialized on enable, after an iteration of
    // robotPeriodic. The modeInit methods run before robotPeriodic, so logger.init will be null
    // here and beginPhase() won't be called. Instead, the Logger takes care of that for practice
    // matches.
    // In real matches, logger.init will initialize when the FMS is connected (before enabling).
    logger.beginPhase(MatchPhase.Autonomous);
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    m_robotContainer.runZeroSensors();
    m_robotContainer.runOnStart();
    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
    logger.getMainLog().println(LogLevel.HIGH, "Autonomous Command Called");
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    logger.beginPhase(MatchPhase.Teleop);

    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.

    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    m_robotContainer.runOnStart();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    logger.beginPhase(MatchPhase.Test);
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {
    SimManager.simulationInit();
  }

  @Override
  public void simulationPeriodic() {
    SimManager.simulationPeriodic();
  }
}
