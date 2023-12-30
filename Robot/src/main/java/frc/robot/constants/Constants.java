package frc.robot.constants;

import java.util.Collections;
import java.util.List;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 *
 * <p>Further, this class should only be used for constants that are universal across all of our
 * robots, put robot specifc constants in the correct Constants class in the robotspecifics folder
 *
 * <p>Game-specific constants should go in their respective derived class. The base class is
 * universal.
 */
public class Constants {
  public static final class DriveTrainConstants {
    public static final int LEFT_FRONT_PORT = 0;
    public static final int RIGHT_FRONT_PORT = 1;
    public static final int LEFT_BACK_PORT = 2;
    public static final int RIGHT_BACK_PORT = 3;

    public static final double DRIVE_TRAIN_SPEED = 0.5;
    public static final double SWERVE_DRIVE_MAX_SPEED = 4;
    public static final double SWERVE_DRIVE_MAX_ACCEL = 4 / 2;
    public static final double SWERVE_ROTATION_SPEED = 3 * Math.PI;

    public static final double SWERVE_DRIVE_ROTATION_P = 0.042;
    public static final double SWERVE_DRIVE_ROTATION_I = 0.001;
    public static final double SWERVE_DRIVE_ROTATION_D = 0.003;
  }

  public static final class OIConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int BUTTON_BOX_PORT = 1;
    public static final double DEFAULT_CONTROLLER_DEADZONE = 0.3;
    public static final double BIG_CONTROLLER_DEADZONE = 0.6;
  }

  public static final class CandyConundrumConstants extends Constants {
    public static final List<Integer> cubeTagIds =
        Collections.unmodifiableList(List.of(0, 5, 7, 26, 29));
    public static final List<Integer> stationTagIds =
        Collections.unmodifiableList(List.of(2, 6, 12, 20));
    public static final int HUMAN_PLAYER_STATION_TAG_ID = 10;
  }
}
