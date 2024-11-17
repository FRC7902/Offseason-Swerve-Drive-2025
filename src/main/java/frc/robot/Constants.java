// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;

    // Joystick deadbands
    public static final double LEFT_X_DEADBAND = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
  }

  public static class DriveConstants {
    public static final double MAX_SPEED = Units.feetToMeters(4.5);
  }

  public static class AutonConstants {
    public static final PIDConstants TRANSLATION_PID = new PIDConstants(5, 2, 3, 5);
    public static final PIDConstants ANGLE_PID = new PIDConstants(5, 2, 3, 5);;
    public static final double MAX_MODULE_SPEED = 5;
    public static final boolean isCompetition = false;

  }

}
