package frc.robot.commands.supercell.arm;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.commands.DoneCycleCommand;
import frc.lib.commands.LogCommand;
import frc.lib.controlsystems.DoneCycleMachine;
import frc.robot.constants.Constants2023.RobotConstants.ArmConstants.ArmSetpoint;
import frc.robot.subsystems.supercell.Arm;

public class MoveArmWithPID extends CommandBase {
  public interface IAnglePIDSetpoint {
    double getAngle();

    double getP();

    double getI();

    double getD();
  }

  public record AnglePIDSetpoint(Rotation2d angle, double p, double i, double d)
      implements IAnglePIDSetpoint {
    @Override
    public double getAngle() {
      return angle.getRadians();
    }

    @Override
    public double getP() {
      return p;
    }

    @Override
    public double getI() {
      return i;
    }

    @Override
    public double getD() {
      return d;
    }
  }

  public static Command createForAuto(Arm arm, IAnglePIDSetpoint goal) {
    return LogCommand.endAfter(
        new DoneCycleCommand<>(new MoveArmWithPID(arm, goal), true)
            .withDoneCycles(
                DoneCycleMachine.supplierWithMinCycles(
                    () -> Math.abs(arm.getMeasurement() - goal.getAngle()) <= Math.toRadians(10),
                    10)),
        5);
  }

  public static Command createForAuto(Arm arm, ArmSetpoint goal) {
    return createForAuto(arm, arm.armSetpoints.get(goal));
  }

  private final Arm arm;
  private final IAnglePIDSetpoint setpoint;

  public MoveArmWithPID(Arm arm, IAnglePIDSetpoint setpoint) {
    this.arm = arm;
    this.setpoint = setpoint;
    addRequirements(arm);
  }

  public MoveArmWithPID(Arm arm, ArmSetpoint setpoint) {
    this(arm, arm.armSetpoints.get(setpoint));
  }

  @Override
  public void initialize() {
    arm.modifyPIDController(
        pidController -> pidController.setPID(setpoint.getP(), setpoint.getI(), setpoint.getD()));
    arm.setArmPosition(setpoint.getAngle());
  }
}
