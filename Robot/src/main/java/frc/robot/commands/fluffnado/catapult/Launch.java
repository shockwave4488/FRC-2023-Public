// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.fluffnado.catapult;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.fluffnado.Catapult;

public class Launch extends CommandBase {

  private final Catapult catapult;
  private final boolean invalidArguments;
  private final double speed;
  private final double desiredAngle;
  private final boolean smartMotion;

  /** Creates a new reset. */
  public Launch(Catapult catapult, double speed, double goalAngle) {
    this(catapult, speed, goalAngle, false);
  }

  public Launch(Catapult catapult, double speed, double goalAngle, boolean smartMotion) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.speed = speed;
    invalidArguments = goalAngle > catapult.MAX_CATAPULT_ANGLE;
    if (invalidArguments) {
      desiredAngle = catapult.MAX_CATAPULT_ANGLE;
    } else {
      desiredAngle = goalAngle;
    }
    this.smartMotion = smartMotion;
    this.catapult = catapult;
    addRequirements(catapult);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (smartMotion == false) {
      catapult.setTargetVelocity(speed);
    } else {
      catapult.setTargetAngle(speed, speed, Rotation2d.fromDegrees(desiredAngle));
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return desiredAngle <= catapult.getAngle().getDegrees();
  }
}
