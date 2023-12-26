package frc.robot.commands.eruption.climber;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.lib.sensors.gyro.NavX;
import frc.robot.commands.drive.LockedSwerveDrive;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.eruption.Climber;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class DriveAndClimb extends SequentialCommandGroup {
  private ClimbMotionTest climbMotionTest;
  private ClimberLiftToHeight climberLiftToHeight;

  public DriveAndClimb(
      SwerveDrive swerve,
      Climber climber,
      NavX gyro,
      int desiredFirstHeight,
      DoubleSupplier power,
      BooleanSupplier doneDriving,
      BooleanSupplier abort) {
    climberLiftToHeight = new ClimberLiftToHeight(climber, desiredFirstHeight, abort);
    climbMotionTest = new ClimbMotionTest(climber, power);

    addCommands(
        climberLiftToHeight,
        new WaitUntilCommand(doneDriving),
        climbMotionTest.alongWith(new LockedSwerveDrive(swerve).asProxy()));
  }
}
