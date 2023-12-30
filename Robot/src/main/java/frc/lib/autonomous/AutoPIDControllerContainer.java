package frc.lib.autonomous;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;

public class AutoPIDControllerContainer {
  public final PIDController xPidController;
  public final PIDController yPidController;
  public final ProfiledPIDController thetaPidController;

  public AutoPIDControllerContainer(
      PIDController xPidController,
      PIDController yPidController,
      ProfiledPIDController thetaPidController) {
    this.xPidController = xPidController;
    this.yPidController = yPidController;
    this.thetaPidController = thetaPidController;
    this.thetaPidController.enableContinuousInput(-Math.PI, Math.PI);
  }
}
