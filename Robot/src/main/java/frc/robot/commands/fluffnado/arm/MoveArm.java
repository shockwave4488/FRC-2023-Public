package frc.robot.commands.fluffnado.arm;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.fluffnado.Arm;

public class MoveArm extends CommandBase {
  private final Arm arm;
  private final double setpoint;

  public MoveArm(Arm arm, double setpoint) {
    this.arm = arm;
    this.setpoint = setpoint;
    addRequirements(arm);
  }

  @Override
  public void initialize() {
    arm.setArmPosition(setpoint);
  }
}
