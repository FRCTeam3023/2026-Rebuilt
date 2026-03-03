// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;
import frc.robot.util.Gains;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final double DEADBAND = 0.05;
  }
  public static final double maxSpeed = Units.feetToMeters(4.5);

  public static class CAN_ID {
    public static final int IMU_ID = 20;
  }

  public static class GAINS { // Start of Jarvis 2025 code portion
      public static Gains DRIVE = new Gains(5, 0, 0.15, 2.65, 12);
      public static Gains TURN = new Gains(.6, 1);
      public static Gains ELEVATOR = new Gains(20, 0, 0, 0, 0, 12);
      public static Gains END_EFFECTOR = new Gains(0.005, 0, 0, 0, 12);
      public static Gains PIVOT = new Gains(3, 0, 0, 0, 12);
      public static Gains CLIMBER = new Gains(100, 0, 0, 0, 12);
  }

  public static class ModuleConstants {
    public static final double DRIVE_GEARING = 8;
    public static final double WHEEL_DIA = Units.inchesToMeters(3.875);
    public static final double TURN_GEARING = 2.89 * 2.89 * 6;
  } // End of Jarvis 2025 code portion
}
