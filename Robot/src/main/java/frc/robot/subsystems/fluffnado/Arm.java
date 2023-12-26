package frc.robot.subsystems.fluffnado;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.logging.LeveledSmartDashboard;
import frc.lib.logging.LogManager;
import frc.lib.preferences.PreferencesParser;
import frc.lib.sensors.MagneticEncoder;
import frc.lib.wpiextensions.ShockwaveSubsystemBase;
import frc.robot.commands.supercell.arm.MoveArmWithPID.AnglePIDSetpoint;
import frc.robot.constants.Constants2024.ArmConstants;
import java.util.function.Consumer;

public class Arm extends ShockwaveSubsystemBase {
  private static final double DEFAULT_OFFSET = -3.14;
  private AnglePIDSetpoint armMinimumSetpoint;
  private final MagneticEncoder armEncoder;
  protected final ProfiledPIDController pidController;
  private final CANSparkMax armMotor; // master
  private final ArmFeedforward armFeedForward;
  private static final double ARM_HOLD_VOLTAGE = -0.5;
  private double desiredAngle;
  private double encoderOffset;
  private double feedForwardKs;
  private double feedForwardKg;
  private double feedForwardKv;
  private double feedForwardKa;
  private boolean onDashboard;
  private double armKp;
  private double armKi;
  private double armKd;

  public Arm(PreferencesParser prefs, LogManager logger) {
    updateFromPrefs(prefs);
    armEncoder = new MagneticEncoder(5, 2048, encoderOffset, true, 1025);

    armMotor = new CANSparkMax(prefs.getInt("ArmMotorID"), MotorType.kBrushless);
    for (CANSparkMax motor : new CANSparkMax[] {armMotor}) {
      motor.restoreFactoryDefaults();
      motor.setSmartCurrentLimit(ArmConstants.ARM_CURRENT_LIMIT);
    }
    armMotor.setInverted(true);

    armFeedForward = new ArmFeedforward(feedForwardKs, feedForwardKg, feedForwardKv, feedForwardKa);
    onDashboard = LeveledSmartDashboard.INFO.isEnabled();
    LeveledSmartDashboard.addChangeListener(
        (prev, now) -> {
          boolean nowOnDashboard = LeveledSmartDashboard.INFO.isEnabled();
          if (!onDashboard && nowOnDashboard) {
            putToDashboard();
          } else if (onDashboard && !nowOnDashboard) {
            readFromDashboard();
          }
          onDashboard = nowOnDashboard;
        });
    if (onDashboard) {
      putToDashboard();
    }
    pidController =
        new ProfiledPIDController(armKp, armKi, armKd, new TrapezoidProfile.Constraints(3, 3));
  }

  private void updateFromPrefs(PreferencesParser prefs) {
    armKp = prefs.tryGetValue(prefs::getDouble, "ArmP", 0.0);
    armKi = prefs.tryGetValue(prefs::getDouble, "ArmI", 0.0);
    armKd = prefs.tryGetValue(prefs::getDouble, "ArmD", 0.0);
    feedForwardKs = prefs.tryGetValue(prefs::getDouble, "ArmKs", 1.5);
    feedForwardKg = prefs.tryGetValue(prefs::getDouble, "ArmKg", 0.04);
    feedForwardKv = prefs.tryGetValue(prefs::getDouble, "ArmKv", 3.0);
    feedForwardKa = prefs.tryGetValue(prefs::getDouble, "ArmKa", 0.0);
    armMinimumSetpoint =
        new AnglePIDSetpoint(
            new Rotation2d(
                prefs.tryGetValue(prefs::getDouble, "ArmMinimumValue", ArmConstants.DOWN)),
            armKp,
            armKi,
            armKd);
    encoderOffset = prefs.tryGetValue(prefs::getDouble, "ArmEncoderOffset", DEFAULT_OFFSET);
  }

  public void modifyPIDController(Consumer<ProfiledPIDController> modifier) {
    modifier.accept(pidController);
  }

  public void putToDashboard() {
    SmartDashboard.putNumber("ArmP", armKp);
    SmartDashboard.putNumber("ArmI", armKi);
    SmartDashboard.putNumber("ArmD", armKd);
  }

  public void readFromDashboard() {
    armKp = SmartDashboard.getNumber("ArmP", armKp);
    armKi = SmartDashboard.getNumber("ArmI", armKi);
    armKd = SmartDashboard.getNumber("ArmD", armKd);
  }

  public AnglePIDSetpoint getMinimumSetpoint() {
    return armMinimumSetpoint;
  }

  public void setArmPosition(double goalAngle) {
    desiredAngle = goalAngle;
    goalAngle = Math.max(goalAngle, armMinimumSetpoint.getAngle());
    goalAngle = Math.min(goalAngle, ArmConstants.ARM_MAXIMUM_VALUE);
    SmartDashboard.putNumber("Goal Angle", goalAngle);
    pidController.reset(getMeasurement());
    pidController.setGoal(goalAngle);
  }

  public double getMeasurement() {
    return (armEncoder.getAngle() - encoderOffset);
  }

  private void setMotorVoltage() {
    if (desiredAngle <= armMinimumSetpoint.getAngle()) {
      armMotor.setVoltage(ARM_HOLD_VOLTAGE);
    } else {
      double pidVolts = pidController.calculate(getMeasurement());
      double feedforwardVolts =
          armFeedForward.calculate(
              pidController.getSetpoint().position, pidController.getSetpoint().velocity);
      double volts = feedforwardVolts + pidVolts;
      armMotor.setVoltage(volts);

      LeveledSmartDashboard.INFO.putNumber("Applied arm voltage", volts);
      LeveledSmartDashboard.INFO.putNumber("Applied feedforward voltage", feedforwardVolts);
    }
  }

  @Override
  public void periodic() {
    if (onDashboard) {
      readFromDashboard();
    }

    setMotorVoltage();
  }

  @Override
  public void onStart() {
    armMotor.setIdleMode(IdleMode.kBrake);
    pidController.reset(getMeasurement());
  }

  @Override
  public void onStop() {
    armMotor.setIdleMode(IdleMode.kCoast);
  }

  @Override
  public void zeroSensors() {}

  @Override
  public void updateSmartDashboard() {
    SmartDashboard.putNumber("ArmPosition", getMeasurement());
    SmartDashboard.putNumber("ArmPositionRaw", armEncoder.getAngle());
  }

  @Override
  public void setUpTrackables(LogManager logger) {}
}
