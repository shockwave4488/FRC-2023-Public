package frc.robot.subsystems.eruption;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import frc.lib.logging.LeveledSmartDashboard;
import frc.lib.logging.LogManager;
import frc.lib.sensors.BeamBreak;
import frc.lib.wpiextensions.ShockwaveSubsystemBase;

public class Intake extends ShockwaveSubsystemBase {

  public static enum RollerState {
    ReverseFull(-1),
    ReverseMedium(-0.5),
    ReverseCrawl(
        -0.35), // This is different than ForwardCrawl so we can have a precise speed we purge balls
    // at
    Off(0),
    ForwardCrawl(0.25),
    ForwardMedium(0.5),
    ForwardFull(0.8);

    double power;

    private RollerState(double power) {
      this.power = power;
    }
  }

  private Alliance allianceColor;
  private RollerState topRollerState = RollerState.Off;
  private RollerState bottomRollerState = RollerState.Off;
  private boolean intakeOut;
  private final TalonSRX topRoller;
  private final TalonSRX bottomRoller;
  private final Solenoid intakePiston;
  private final BeamBreak ballSensor;
  private final BeamBreak colorSensor;

  public Intake(
      int topRollerID,
      int bottomRollerID,
      int PCM_ID,
      int pistonID,
      int colorSensorID,
      int ballSensorID) {
    topRoller = new TalonSRX(topRollerID);
    topRoller.configFactoryDefault();
    bottomRoller = new TalonSRX(bottomRollerID);
    bottomRoller.configFactoryDefault();
    topRoller.configVoltageCompSaturation(12);
    topRoller.enableVoltageCompensation(true);
    bottomRoller.configVoltageCompSaturation(12);
    bottomRoller.enableVoltageCompensation(true);
    bottomRoller.setInverted(true);
    intakePiston = new Solenoid(PCM_ID, PneumaticsModuleType.CTREPCM, pistonID);
    colorSensor = new BeamBreak(colorSensorID);
    ballSensor = new BeamBreak(ballSensorID);
    allianceColor = DriverStation.getAlliance();
  }

  public void setTopRollerState(RollerState power) {
    topRollerState = power;
  }

  public void setBottomRollerState(RollerState power) {
    bottomRollerState = power;
  }

  public void intakeIn() {
    intakeOut = false;
  }

  public void intakeOut() {
    intakeOut = true;
  }

  public boolean getIntakeOut() {
    return intakeOut;
  }

  public double getTopRollerPower() {
    return topRollerState.power;
  }

  public double getBottomRollerPower() {
    return bottomRollerState.power;
  }

  /**
   * Returns if, based on the color sensors on the intake, the intake has a cargo.
   *
   * @return If the intake thinks it has a cargo
   */
  public boolean hasCargo() {
    return (ballSensor.get());
  }

  /**
   * Returns if the robot has the correct color cargo
   *
   * @return
   */
  public boolean hasCorrectColor() {
    boolean correct =
        colorSensor.get() && allianceColor == Alliance.Blue
            || !colorSensor.get() && allianceColor == Alliance.Red;
    return correct;
  }

  @Override
  public void onStart() {
    setTopRollerState(RollerState.Off);
    setBottomRollerState(RollerState.Off);
    intakeIn();
    allianceColor = DriverStation.getAlliance();
  }

  @Override
  public void periodic() {
    topRoller.set(ControlMode.PercentOutput, topRollerState.power);
    bottomRoller.set(ControlMode.PercentOutput, bottomRollerState.power);
    intakePiston.set(intakeOut);
  }

  @Override
  public void onStop() {
    setTopRollerState(RollerState.Off);
    setBottomRollerState(RollerState.Off);
    intakeIn();
  }

  @Override
  public void zeroSensors() {}

  @Override
  public void updateSmartDashboard() {
    LeveledSmartDashboard.HIGH.putBoolean("Has Cargo", hasCargo());
    LeveledSmartDashboard.HIGH.putBoolean("Correct Color", hasCorrectColor());
  }

  @Override
  public void setUpTrackables(LogManager logger) {
    // logger.makeLogFile("IntakeExtensionState", 2)
    // .withTrackable("IntakeExtensionState", this::getIntakeOut);

    logger
        .getLogFile("Intake/TopRollerPower")
        .addTracker("IntakeTopRollerPower", () -> topRollerState.power, 4);

    logger
        .getLogFile("Intake/BottomRollerPower")
        .addTracker("IntakeBottomRollerPower", () -> bottomRollerState.power, 4);
    // below trackables have higher frequencies to prevent the logger missing the beam breaks' quick
    // on/off changes
    logger.getLogFile("Intake/HasCargo").addTracker("HasCargo", this::hasCargo, 10);

    logger
        .getLogFile("Intake/HasCorrectColor")
        .addTracker("HasCorrectColor", this::hasCorrectColor, 10);
  }
}
