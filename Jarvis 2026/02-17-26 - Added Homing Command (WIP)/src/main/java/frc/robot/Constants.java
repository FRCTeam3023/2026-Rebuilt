// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import swervelib.math.Matter;
import frc.robot.Util.Gains;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static int LED_LENGTH = 10;

  public enum CAN_DEVICES {
    // These are assigned algorithmically in Drivetrain; do not use for anything else
    // DRIVE_FRONT_LEFT(1),
    // DRIVE_FRONT_RIGHT(2),
    // DRIVE_BACK_LEFT(3),
    // DRIVE_BACK_RIGHT(4),
    // TURN_FRONT_LEFT(5),
    // TURN_FRONT_RIGHT(6),
    // TURN_BACK_LEFT(7),
    // TURN_BACK_RIGHT(8),

    ROBORIO(0),
    ELEVATOR_MOTOR(9),
    END_EFFECTOR(10),
    PIVOT(11),
    CLIMBER(12),
    PIGEON_2(20);

    public int id;
    private CAN_DEVICES(int id) {
      this.id = id;
    }
  }

  public static class DEBUG {
    public static boolean PREVENT_TAB_SWITCHING = false;
    public static boolean FORCE_UPDATE_POSE = false;
  }

  public static class GAINS {
    public static Gains DRIVE = new Gains(5, 0, 0.15, 2.65, 12);
    public static Gains TURN = new Gains(.6, 1);
    public static Gains ELEVATOR = new Gains(20, 0, 0, 0, 0, 12);
    public static Gains END_EFFECTOR = new Gains(0.005, 0, 0, 0, 12);
    public static Gains PIVOT = new Gains(3, 0, 0, 0, 12);
    public static Gains CLIMBER = new Gains(100, 0, 0, 0, 12);
  }

  public static class DrivetrainConstants {
    public static final double WHEEL_BASE = Units.inchesToMeters(22.5);
    public static final double MAX_DRIVE_SPEED = 5;
    public static final double MAX_ANGULAR_SPEED = 3;
    public static final double ALIGN_CONTROL_MULTIPLIER = 0.2;
    public static final double DRIVE_TOLERANCE_PERCENT = 0.05;
  }

  public static class ModuleConstants{
    /** Overall max speed of the module in m/s */
    public static final double MAX_SPEED = DrivetrainConstants.MAX_DRIVE_SPEED;

    /** rotational offset in radians of modules during homing */
    public static final double[] MODULE_OFFSETS = new double[] {
      -Math.PI/4,
      Math.PI/4,
      Math.PI/4,
      -Math.PI/4
    };

    /** Gear Ratio of the drive motor */
    public static final double DRIVE_GEARING = 8;

    /**Gear ratio of the turning motor */
    public static final double TURN_GEARING = 2.89 * 2.89 * 6;

    /**Diameter of the billet wheel */
    public static final double WHEEL_DIA = Units.inchesToMeters(3.875);

    public static final Translation2d[] MODULE_POSITIONS = {
      new Translation2d(DrivetrainConstants.WHEEL_BASE/2 , DrivetrainConstants.WHEEL_BASE/2),
      new Translation2d(DrivetrainConstants.WHEEL_BASE/2 , -DrivetrainConstants.WHEEL_BASE/2),
      new Translation2d(-DrivetrainConstants.WHEEL_BASE/2 , DrivetrainConstants.WHEEL_BASE/2),
      new Translation2d(-DrivetrainConstants.WHEEL_BASE/2 , -DrivetrainConstants.WHEEL_BASE/2)
    };
  }

  public static class PhotonConstants {
    public static final double MAX_DISTANCE = 1.5;
    public static final double MAX_AMBIGUITY = 0.05;
    public static final double STANDARD_DEVIATION = 0.6; //Decrease to trust more

    public static final Transform3d[] ROBOT_TO_CAMERAS = {
      new Transform3d(
        new Translation3d(0.3427920204264734, 0.2743797923096529, 0.2552700284612291), 
        new Rotation3d(0.043313011402100504, -0.6286858552653907, 0.06756944691730396)
      ),
      new Transform3d(
        new Translation3d(0.36989407217844056, -0.34934358915182817, 0.1960746849609884), 
        new Rotation3d(-0.009310973845960677, -0.46471606065446996, -0.049877760937597264)
      ),
      new Transform3d(
        new Translation3d(-0.3341210310485878, 0.25044460157618835, 0.2325700723726466), 
        new Rotation3d(0.023193918482595763, -0.5565355951577495, 3.0599241251685845)
      ),
      new Transform3d(
        new Translation3d(-0.3531964431772765, -0.32643772352780843, 0.26445212888245784), 
        new Rotation3d(0.061995396118376844, -0.6501071778295155, -3.0874453173905994)
      )
    };

    public static final Transform3d[] CAMERAS_TO_ROBOT = {
      ROBOT_TO_CAMERAS[0].inverse(),
      ROBOT_TO_CAMERAS[1].inverse(),
      ROBOT_TO_CAMERAS[2].inverse(),
      ROBOT_TO_CAMERAS[3].inverse()
    };

    public static final AprilTagFieldLayout APRIL_TAG_FIELD_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    public static final Pose3d ROBOT_TO_CALIBRATION = new Pose3d(
      new Translation3d(
        Units.inchesToMeters(-44.75), 
        Units.inchesToMeters(0.05),
        Units.inchesToMeters(35.875)
      ),
      new Rotation3d(0, 0, 0)
    );
    public static final int NUM_CALIBRATION_ENTRIES = 100;
  }

  public static class OperatorConstants {
    // Joystick Deadband
    public static final double DEADBAND        = 0.1;
    public static final double LEFT_Y_DEADBAND = 0.1;
    public static final double RIGHT_X_DEADBAND = 0.1;
    public static final double TURN_CONSTANT    = 6;
  }
}
