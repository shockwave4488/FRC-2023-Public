package frc.robot.commands.eruption.shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.lib.commands.DoneCycleCommand;
import frc.lib.commands.LogCommand;
import frc.lib.controlsystems.DoneCycleMachine;
import frc.lib.sensors.gyro.NavX;
import frc.lib.sensors.vision.VisionCameras.TargetCamera;
import frc.robot.commands.drive.HeadingRotation;
import frc.robot.commands.drive.RotateTowardsPosition;
import frc.robot.commands.eruption.drive.VisionAlignToTarget;
import frc.robot.commands.eruption.indexer.IndexerAdvance2;
import frc.robot.subsystems.SmartPCM;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.eruption.Indexer;
import frc.robot.subsystems.eruption.Shooter;

public class CalculatedShot extends ParallelRaceGroup {
  public static final int MIN_HAS_TARGET_CYCLES = 8;

  public CalculatedShot(
      Shooter shooter,
      Indexer conveyor,
      SmartPCM compressor,
      SwerveDrive swerve,
      NavX gyro,
      double rotationMultiplier,
      TargetCamera limelight,
      RotateTowardsPosition turnToHubCommand,
      HeadingRotation headingCommand,
      DoneCycleCommand<VisionAlignToTarget> alignToTarget) {

    DoneCycleCommand<?> spinFlywheel =
        new DoneCycleCommand<>(
                new SpinFlywheel(
                    shooter,
                    limelight,
                    conveyor.getIndexerStates()::getFlywheelBeamBreak,
                    swerve::getOdometry,
                    true,
                    true),
                false)
            .withDoneCycles(DoneCycleMachine.fromSupplier(shooter::hoodReady).withName("hoodReady"))
            .withDoneCycles(shooter.flywheelVelocityMachine);

    InstantCommand stopCompressor =
        new InstantCommand(() -> compressor.stopCompressor(), compressor);

    addCommands(
        LogCommand.arrayOf(
            this,
            LogCommand.parallel(spinFlywheel, stopCompressor),
            LogCommand.sequence(
                turnToHubCommand,
                headingCommand,
                LogCommand.race(
                    alignToTarget,
                    LogCommand.sequence(
                        new WaitUntilCommand(
                            () -> alignToTarget.isReady() && spinFlywheel.isReady()),
                        new IndexerAdvance2(conveyor))))));
  }
}
