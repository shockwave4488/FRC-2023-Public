// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

import edu.wpi.first.math.geometry.Translation2d;

public final class Constants2022 extends Constants {

  public static final class ShooterConstants {
    public static final int MASTER_PORT = 0;
    public static final int FOLLOWER_PORT = 0;
    public static final double RAMP_RATE = 0.1;

    public static final double FENDER_RPM = 2050; // Needs to be adjusted/tested
    public static final double FENDER_HOOD_INPUT = 30; // 6.6 -> 8
    public static final double BACK_OF_TARMAC_RPM = 2225; // Adjusted but untested
    public static final double BACK_OF_TARMAC_HOOD_INPUT = 40; // Adjusted but untested
    public static final double MIN_HOOD_POSITION = 30; // Based on the CAD
    public static final double FRONT_LAUNCHPAD_RPM = 2675; // Theoretically correct but untested
    public static final double FRONT_LAUNCHPAD_HOOD_INPUT =
        59.8; // Theoretically correct but untested
    public static final double SIDE_LAUNCHPAD_RPM = 2775; // Theoretically correct but untested
    public static final double SIDE_LAUNCHPAD_HOOD_INPUT = 65; // Theoretically correct but untested
  }

  public static final class ClimberConstants {
    public static final int CLIMBER_LOCKED_TICKS = 323000; // Just a guess, should be changed later
    public static final int CLIMBER_MIN_TICKS_TO_LOCK = 320000;
  }

  public static final class FieldConstants {
    public static final Translation2d HUB_CENTER = new Translation2d(8.22, 4.15);
    public static final double HUB_RADIUS_METERS =
        0.63; // This is the distance between the center of the HUB and the INSIDE edge (0.61) plus
    // 0.02 (for the width of what the reflective tape is on, this is an approximation).
    public static final double HUB_SAFE_SHOT_RADIUS_METERS = 0.3; // 0.4 -> 0.3
    public static final double TARGET_HEIGHT_METERS = 2.62;
  }

  public static final class AutonomousConstants {
    public static final double PRACTICE_SHOOTER_RPM_OUTSIDE_TARMAC = 1500;
    public static final double PRACTICE_SHOOTER_RPM_INSIDE_TARMAC = 1500; // 2800 when in comp

    public static final double COMP_SHOOTER_RPM_INSIDE_TARMAC =
        2275; // 1974 -> 2050 -> 2250 -> 2200
    public static final double COMP_SHOOTER_HOOD_INPUT_INSIDE_TARMAC = 43; // updated 33 -> 38 -> 43
    public static final double COMP_SHOOTER_RPM_OUTSIDE_TARMAC = 2360; // 2265 -> 2285 -> 2360
    public static final double COMP_SHOOTER_HOOD_INPUT_OUTSIDE_TARMAC = 52;
  }
}
