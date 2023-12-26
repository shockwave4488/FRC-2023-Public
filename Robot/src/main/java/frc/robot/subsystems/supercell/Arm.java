package frc.robot.subsystems.supercell;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.REVPhysicsSim;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.controlsystems.SimManager;
import frc.lib.logging.LeveledSmartDashboard;
import frc.lib.logging.LogManager;
import frc.lib.preferences.PreferencesParser;
import frc.lib.sensors.HallEffect;
import frc.lib.sensors.MagneticEncoder;
import frc.lib.wpiextensions.ShockwaveSubsystemBase;
import frc.robot.commands.supercell.arm.MoveArmWithPID;
import frc.robot.commands.supercell.arm.MoveArmWithPID.AnglePIDSetpoint;
import frc.robot.constants.Constants2023.RobotConstants.ArmConstants;
import frc.robot.constants.Constants2023.RobotConstants.ArmConstants.ArmSetpoint;
import java.util.Collections;
import java.util.HashMap;
import java.util.Map;
import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.Supplier;

public class Arm extends ShockwaveSubsystemBase {

  private static final double DEFAULT_OFFSET = 0.2915;
  private static final double IN_THRESHOLD = -1.71;
  private static final double ARM_HOLD_VOLTAGE = -0.5;
  protected final ProfiledPIDController pidController;
  private final CANSparkMax rightArmMotor; // master
  private final CANSparkMax leftArmMotor; // follower
  private final ArmFeedforward armFeedForward;
  private final Consumer<Double> setVoltage;
  private final Supplier<Double> getAngle;
  private final Optional<SingleJointedArmSim> armSim;
  private double desiredAngle;
  private boolean encoderFlip;
  private double upKp;
  private double upKi;
  private double upKd;
  private double downKp;
  private double downKi;
  private double downKd;
  private double pickUpKp;
  private double pickUpKi;
  private double pickUpKd;
  private double inKp;
  private boolean onDashboard;
  private boolean hasHallEffects;
  private AnglePIDSetpoint armMinimumSetpoint;
  private double encoderOffset;
  private double armOffset = 0;
  private double feedForwardKs;
  private double feedForwardKg;
  private double feedForwardKv;
  private double feedForwardKa;
  private final Optional<HallEffect> upperHallEffect;
  private final Optional<HallEffect> lowerHallEffect;
  private int lowerHallEffectPort;
  private int upperHallEffectPort;

  public final Map<ArmSetpoint, AnglePIDSetpoint> armSetpoints;

  public Arm(PreferencesParser prefs, LogManager logger) {
    updateFromPrefs(prefs);

    rightArmMotor = new CANSparkMax(prefs.getInt("RightArmMotorID"), MotorType.kBrushless);
    leftArmMotor = new CANSparkMax(prefs.getInt("LeftArmMotorID"), MotorType.kBrushless);
    for (CANSparkMax motor : new CANSparkMax[] {rightArmMotor, leftArmMotor}) {
      motor.restoreFactoryDefaults();
      motor.setSmartCurrentLimit(ArmConstants.ARM_CURRENT_LIMIT);
    }
    rightArmMotor.setInverted(!encoderFlip);
    leftArmMotor.follow(rightArmMotor, true);

    if (RobotBase.isSimulation()) {
      REVPhysicsSim.getInstance().addSparkMax(rightArmMotor, DCMotor.getNEO(2));
      REVPhysicsSim.getInstance().addSparkMax(leftArmMotor, DCMotor.getNEO(2));
      SingleJointedArmSim armSim =
          new SingleJointedArmSim(
              DCMotor.getNEO(2),
              220,
              5.75,
              ArmConstants.ARM_LENGTH,
              ArmSetpoint.DEFAULT_MINIMUM.angleRadians,
              ArmConstants.ARM_MAXIMUM_VALUE,
              true);
      setVoltage = armSim::setInputVoltage;
      getAngle = () -> armSim.getAngleRads() * (encoderFlip ? -1 : 1) + encoderOffset;
      this.armSim = Optional.of(armSim);
    } else {
      MagneticEncoder armEncoder = new MagneticEncoder(4, 2048, 0, !encoderFlip, 1025);
      setVoltage = rightArmMotor::setVoltage;
      getAngle = armEncoder::getAngle;
      this.armSim = Optional.empty();
    }

    armFeedForward = new ArmFeedforward(feedForwardKs, feedForwardKg, feedForwardKv, feedForwardKa);
    if (hasHallEffects) {
      lowerHallEffect = Optional.of(new HallEffect(lowerHallEffectPort));
      upperHallEffect = Optional.of(new HallEffect(upperHallEffectPort));
    } else {
      lowerHallEffect = Optional.empty();
      upperHallEffect = Optional.empty();
    }
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
        new ProfiledPIDController(upKp, upKi, upKd, new TrapezoidProfile.Constraints(3, 3));

    final Map<ArmSetpoint, AnglePIDSetpoint> armSetpointsMap = new HashMap<>();
    putSetpointInMap(armSetpointsMap, ArmSetpoint.LOW_SCORE, downKp, downKi, downKd);
    putSetpointInMap(armSetpointsMap, ArmSetpoint.MID_SCORE, upKp, upKi, upKd);
    putSetpointInMap(armSetpointsMap, ArmSetpoint.HIGH_SCORE, upKp, upKi, upKd);
    putSetpointInMap(armSetpointsMap, ArmSetpoint.SUBSTATION, upKp, upKi, upKd);
    putSetpointInMap(armSetpointsMap, ArmSetpoint.PIECE_PICKUP, pickUpKp, pickUpKi, pickUpKd);
    putSetpointInMap(armSetpointsMap, ArmSetpoint.DEFAULT_MINIMUM, downKp, downKi, downKd);
    armSetpoints = Collections.unmodifiableMap(armSetpointsMap);

    new MoveArmWithPID(this, getMinimumSetpoint())
        .ignoringDisable(true)
        .withName("StowArm")
        .schedule();
  }

  private void putSetpointInMap(
      Map<ArmSetpoint, AnglePIDSetpoint> map, ArmSetpoint setpoint, double p, double i, double d) {
    map.put(setpoint, new AnglePIDSetpoint(new Rotation2d(setpoint.angleRadians), p, i, d));
  }

  private void updateFromPrefs(PreferencesParser prefs) {
    upKp = prefs.tryGetValue(prefs::getDouble, "UpArmP", 0.0);
    upKi = prefs.tryGetValue(prefs::getDouble, "UpArmI", 0.0);
    upKd = prefs.tryGetValue(prefs::getDouble, "UpArmD", 0.0);
    downKp = prefs.tryGetValue(prefs::getDouble, "DownArmP", 0.0);
    downKi = prefs.tryGetValue(prefs::getDouble, "DownArmI", 0.0);
    downKd = prefs.tryGetValue(prefs::getDouble, "DownArmD", 0.0);
    pickUpKp = prefs.tryGetValue(prefs::getDouble, "PickUpArmP", 0.0);
    pickUpKi = prefs.tryGetValue(prefs::getDouble, "PickUpArmI", 0.0);
    pickUpKd = prefs.tryGetValue(prefs::getDouble, "PickUpArmD", 0.0);
    inKp = prefs.tryGetValue(prefs::getDouble, "InArmP", 0.0);
    encoderFlip = prefs.tryGetValue(prefs::getBoolean, "ArmEncoderFlipped", false);
    hasHallEffects = prefs.tryGetValue(prefs::getBoolean, "HasHallEffects", false);
    if (hasHallEffects) {
      upperHallEffectPort = prefs.tryGetValue(prefs::getInt, "UpperHallEffectPort", 6);
      lowerHallEffectPort = prefs.tryGetValue(prefs::getInt, "LowerHallEffectPort", 5);
    }
    feedForwardKs = prefs.tryGetValue(prefs::getDouble, "ArmKs", 1.5);
    feedForwardKg = prefs.tryGetValue(prefs::getDouble, "ArmKg", 0.04);
    feedForwardKv = prefs.tryGetValue(prefs::getDouble, "ArmKv", 3.0);
    feedForwardKa = prefs.tryGetValue(prefs::getDouble, "ArmKa", 0.0);
    armMinimumSetpoint =
        new AnglePIDSetpoint(
            new Rotation2d(
                prefs.tryGetValue(
                    prefs::getDouble, "ArmMinimumValue", ArmSetpoint.DEFAULT_MINIMUM.angleRadians)),
            downKp,
            downKi,
            downKd);
    encoderOffset = prefs.tryGetValue(prefs::getDouble, "ArmEncoderOffset", DEFAULT_OFFSET);
  }

  public void putToDashboard() {
    SmartDashboard.putNumber("UpP", upKp);
    SmartDashboard.putNumber("UpI", upKi);
    SmartDashboard.putNumber("UpD", upKd);
    SmartDashboard.putNumber("DownP", downKp);
    SmartDashboard.putNumber("DownI", downKi);
    SmartDashboard.putNumber("DownD", downKd);
    SmartDashboard.putNumber("PickUpP", pickUpKp);
    SmartDashboard.putNumber("PickUpI", pickUpKi);
    SmartDashboard.putNumber("PickUpD", pickUpKd);
    SmartDashboard.putNumber("In P", inKp);
  }

  public void readFromDashboard() {
    upKp = SmartDashboard.getNumber("UpP", upKp);
    upKi = SmartDashboard.getNumber("UpI", upKi);
    upKd = SmartDashboard.getNumber("UpD", upKd);
    downKp = SmartDashboard.getNumber("DownP", downKp);
    downKi = SmartDashboard.getNumber("DownI", downKi);
    downKd = SmartDashboard.getNumber("DownD", downKd);
    pickUpKp = SmartDashboard.getNumber("PickUpP", pickUpKp);
    pickUpKi = SmartDashboard.getNumber("PickUpI", pickUpKi);
    pickUpKd = SmartDashboard.getNumber("PickUpD", pickUpKd);
    inKp = SmartDashboard.getNumber("In P", inKp);
  }

  public AnglePIDSetpoint getMinimumSetpoint() {
    return armMinimumSetpoint;
  }

  @Override
  public void onStart() {
    rightArmMotor.setIdleMode(IdleMode.kBrake);
    leftArmMotor.setIdleMode(IdleMode.kBrake);
    pidController.reset(getMeasurement());
  }

  public void modifyPIDController(Consumer<ProfiledPIDController> modifier) {
    modifier.accept(pidController);
  }

  public void setArmPosition(double goalAngle) {
    desiredAngle = goalAngle;
    goalAngle += armOffset;
    goalAngle = Math.max(goalAngle, armMinimumSetpoint.getAngle());
    goalAngle = Math.min(goalAngle, ArmConstants.ARM_MAXIMUM_VALUE);
    pidController.reset(getMeasurement());
    pidController.setGoal(goalAngle);
  }

  public void raiseOffset() {
    armOffset += ArmConstants.ARM_CHANGE;
    setArmPosition(desiredAngle);
  }

  public void lowerOffset() {
    armOffset -= ArmConstants.ARM_CHANGE;
    setArmPosition(desiredAngle);
  }

  public double getMeasurement() {
    return (getAngle.get() - encoderOffset) * (encoderFlip ? -1 : 1);
  }

  private void setMotorVoltage() {
    if (((lowerHallEffect.isPresent() && lowerHallEffect.get().get())
            || (upperHallEffect.isPresent() && upperHallEffect.get().get()))
        && desiredAngle <= armMinimumSetpoint.getAngle()) {
      setVoltage.accept(ARM_HOLD_VOLTAGE);
    } else {
      double pidVolts = pidController.calculate(getMeasurement());
      double feedforwardVolts =
          armFeedForward.calculate(
              pidController.getSetpoint().position, pidController.getSetpoint().velocity);
      double volts = feedforwardVolts + pidVolts;
      setVoltage.accept(volts);

      LeveledSmartDashboard.INFO.putNumber("Applied arm voltage", volts);
      LeveledSmartDashboard.INFO.putNumber("Applied feedforward voltage", feedforwardVolts);
    }
  }

  @Override
  public void periodic() {
    if (onDashboard) {
      readFromDashboard();
    }

    if (getMeasurement() <= IN_THRESHOLD && desiredAngle <= IN_THRESHOLD) {
      pidController.setPID(inKp, downKi, downKd);
    }

    setMotorVoltage();
  }

  @Override
  public void simulationPeriodic() {
    armSim.ifPresent(arm -> arm.update(SimManager.getDeltaSimTime() / 1000.0));
  }

  @Override
  public void onStop() {
    rightArmMotor.setIdleMode(IdleMode.kCoast);
    leftArmMotor.setIdleMode(IdleMode.kCoast);
  }

  @Override
  public void zeroSensors() {}

  @Override
  public void updateSmartDashboard() {
    LeveledSmartDashboard.INFO.putNumber("Arm Encoder Angle With Offset", getMeasurement());
    LeveledSmartDashboard.INFO.putNumber("Arm Encoder Angle Raw", getAngle.get());

    LeveledSmartDashboard.INFO.putNumber("Desired Encoder Angle", desiredAngle);
    LeveledSmartDashboard.INFO.putNumber(
        "Arm Position SetPoint", pidController.getSetpoint().position);
    LeveledSmartDashboard.INFO.putNumber(
        "Arm Velocity SetPoint", pidController.getSetpoint().velocity);
    LeveledSmartDashboard.INFO.putBoolean("Encoder flip", encoderFlip);
    LeveledSmartDashboard.INFO.putData("Arm PID controller", pidController);
    if (upperHallEffect.isPresent()) {
      LeveledSmartDashboard.INFO.putBoolean("Upper Hall Effect", upperHallEffect.get().get());
    }
    if (lowerHallEffect.isPresent()) {
      LeveledSmartDashboard.INFO.putBoolean("Lower Hall Effect", lowerHallEffect.get().get());
    }
    LeveledSmartDashboard.INFO.putNumber(
        "Arm Motor Current", rightArmMotor.getOutputCurrent() + leftArmMotor.getOutputCurrent());
  }

  @Override
  public void setUpTrackables(LogManager logger) {
    int loggingFrequency = 5;

    logger
        .getLogFile("Arm State")
        .setDefaultFrequency(loggingFrequency)
        .addTracker("Encoder Angle", this::getMeasurement)
        .addTracker("Desired_Encoder_Angle", () -> desiredAngle)
        .addTracker("Position Setpoint", () -> pidController.getSetpoint().position)
        .addTracker("Velocity Setpoint", () -> pidController.getSetpoint().velocity);
  }
}
