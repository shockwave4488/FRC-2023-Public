package frc.lib.sensors.vision;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.lib.logging.LeveledSmartDashboard;
import frc.lib.logging.LogManager;
import frc.lib.wpiextensions.ShockwaveSubsystemBase;
import org.photonvision.common.hardware.VisionLEDMode;

public abstract class CameraSubsystem extends ShockwaveSubsystemBase implements VisionCamera {
  protected final String name;
  protected final LogManager logger;
  protected final CameraPositionConstants cameraConsts;

  protected SendableChooser<VisionLEDMode> forceLedSelector = new SendableChooser<>();
  protected VisionLEDMode currentLedSelection = forceLedSelector.getSelected();

  public CameraSubsystem(String name, CameraPositionConstants cameraConsts, LogManager logger) {
    this.name = name;
    this.logger = logger;
    this.cameraConsts = cameraConsts;

    String upperName = name.toUpperCase();
    forceLedSelector.setDefaultOption(upperName + "-NormalLedControl", VisionLEDMode.kDefault);
    forceLedSelector.addOption(upperName + "-ForceLedOn", VisionLEDMode.kOn);
    forceLedSelector.addOption(upperName + "-ForceLedOff", VisionLEDMode.kOff);
    forceLedSelector.addOption(upperName + "-LedBlink", VisionLEDMode.kBlink);
    LeveledSmartDashboard.HIGH.putData(upperName + " LED Control", forceLedSelector);
  }

  @Override
  public CameraPositionConstants getCameraPositionConsts() {
    return cameraConsts;
  }

  @Override
  public void onStart() {}

  @Override
  public void onStop() {}

  @Override
  public void zeroSensors() {}

  @Override
  public void periodic() {}

  @Override
  public void updateSmartDashboard() {
    VisionLEDMode ledSelection = forceLedSelector.getSelected();
    if (ledSelection != currentLedSelection) {
      currentLedSelection = ledSelection;
      setLed(ledSelection);
    }
  }

  @Override
  public void setUpTrackables(LogManager logger) {
    logger.getLogFile(name + "/HasTarget").addTracker("HasTarget", this::hasTargets, 10);
  }
}
