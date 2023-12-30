// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.fluffnado.catapult;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.fluffnado.Catapult;

public class ResetCatapult extends CommandBase {

  private static final double RESET_SPEED = 1;
  private static final double RESET_ACCELERATION = 2;

  private final Catapult catapult;

  /** Creates a new reset. */
  public ResetCatapult(Catapult catapult) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.catapult = catapult;
    addRequirements(catapult);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    catapult.setTargetAngle(RESET_SPEED, RESET_ACCELERATION, Rotation2d.fromDegrees(0));
    /*
    if (arm.getMeasurement() < ArmConstants.DOWN + Rotation2d.fromDegrees(5).getRadians()) {
      catapult.setTargetAngle(RESET_SPEED, RESET_ACCELERATION, Rotation2d.fromDegrees(0));
    }
    */
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /*
    if (arm.getMeasurement() < ArmConstants.DOWN + Rotation2d.fromDegrees(5).getRadians() && catapult.getAngle().getDegrees() < 5) {
      catapult.setTargetAngle(RESET_SPEED, RESET_ACCELERATION, Rotation2d.fromDegrees(0));
    } else {
      catapult.setTargetAngle(
          RESET_SPEED,
          RESET_ACCELERATION,
          Rotation2d.fromDegrees(101));
    }
    */
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
