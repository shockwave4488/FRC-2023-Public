package frc.robot.commands.supercell.drive;

import edu.wpi.first.math.controller.ProfiledPIDController;
import frc.lib.sensors.gyro.NavX;
import frc.robot.commands.drive.RotateTowardsPosition;
import frc.robot.constants.Constants2023.Node;
import frc.robot.subsystems.drive.SwerveDrive;
import java.util.function.Supplier;

public class FaceNodePosition extends RotateTowardsPosition {
  public FaceNodePosition(
      SwerveDrive swerve, NavX gyro, ProfiledPIDController thetaController, Supplier<Node> node) {
    super(swerve, gyro, thetaController, () -> node.get().getPosition());
  }
}
