package frc.robot.commands.eruption.drive;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.lib.commands.DoneCycleCommand;
import frc.robot.commands.drive.RotateTowardsPosition;

public class TurnToHubPoseThenVision extends SequentialCommandGroup {
  public TurnToHubPoseThenVision(
      RotateTowardsPosition hubCentricSwerveDriveCommand,
      DoneCycleCommand<VisionAlignToTarget> visionAlignToTargetCommand) {
    super(
        new DoneCycleCommand<>(hubCentricSwerveDriveCommand, true)
            .withDoneCycles(cmd -> cmd.getComposedCommand().getYawDoneCycleMachine(5)),
        visionAlignToTargetCommand);
  }
}
