// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
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

  public static class IntakeActuatorConstants { // ALL VALUES ARE WIP (03/05/26)
    //Diameter of the intake actuator sprocket in meters
    public static final double PULLEY_DIA = Units.inchesToMeters(1.5);
    public static final double PULLEY_CIRCUMFERENCE = Math.PI * PULLEY_DIA;
    
    //Gear ratio of the intake actuator motor
    public static final double GEARING = 3 * 3 * 3;

    public static final double RETRACTED_POSITION = 0;

    //Tolerance for elevator height
    public static final Rotation2d SETPOINT_RANGE = Rotation2d.fromDegrees(15);
  }

  public static class CAN_ID {
    public static final int INTAKE_ACTUATOR_ID = 11;
    public static final int IMU_ID = 20;
  }

  public static class GAINS {
    public static Gains INTAKE_ACTUATOR = new Gains(20, 0, 0, 0, 12);
  }
}
