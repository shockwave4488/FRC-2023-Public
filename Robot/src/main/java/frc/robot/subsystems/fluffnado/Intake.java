package frc.robot.subsystems.fluffnado;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.lib.logging.LeveledSmartDashboard;
import frc.lib.logging.LogManager;
import frc.lib.wpiextensions.ShockwaveSubsystemBase;

public class Intake extends ShockwaveSubsystemBase {
  public enum Speed {
    FAST(0.65),
    STOPPED(0),
    REV_FAST(-0.65);

    private final double value;

    private Speed(double value) {
      this.value = value;
    }

    public double getValue() {
      return value;
    }
  }

  private final TalonSRX rollerMotor;
  private Speed speed = Speed.STOPPED;

  public Intake(int bottomMotorPort) {
    rollerMotor = new TalonSRX(bottomMotorPort);
    rollerMotor.configFactoryDefault();
    rollerMotor.setNeutralMode(NeutralMode.Brake);
    rollerMotor.enableVoltageCompensation(true);
    rollerMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 30, 25, 0));
    rollerMotor.setInverted(true);
  }

  @Override
  public void onStart() {}

  @Override
  public void onStop() {}

  @Override
  public void zeroSensors() {}

  @Override
  public void updateSmartDashboard() {
    LeveledSmartDashboard.INFO.putNumber("Intake > Motor Current", rollerMotor.getStatorCurrent());
  }

  @Override
  public void setUpTrackables(LogManager logger) {
    logger.getLogFile("Intake").addTracker("Speed", () -> speed.getValue(), 5);
  }

  public void setSpeed(Speed speed) {
    this.speed = speed;
    rollerMotor.set(ControlMode.PercentOutput, speed.getValue());
  }

  public double getMotorCurrent() {
    return rollerMotor.getStatorCurrent();
  }

  public Speed getSpeed() {
    return speed;
  }
}
