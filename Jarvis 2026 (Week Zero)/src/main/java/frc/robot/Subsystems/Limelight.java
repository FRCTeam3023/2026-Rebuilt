// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
  Drivetrain drivetrain = new Drivetrain();

  private final NetworkTable table =
        NetworkTableInstance.getDefault().getTable("limelight");

    public boolean hasTarget() {
        return table.getEntry("tv").getDouble(0) == 1;
    }

    public Pose2d getBotPose() {
        var alliance = DriverStation.getAlliance();

        if (alliance.isPresent()) {
            String entryName = alliance.get() == DriverStation.Alliance.Blue
                ? "botpose_wpiblue"
                : "botpose_wpired";

            double[] pose = table.getEntry(entryName).getDoubleArray(new double[6]);

            return new Pose2d(
                pose[0],
                pose[1],
                Rotation2d.fromDegrees(pose[5])
            );
        }

        return new Pose2d();
    }

    public double getLatencySeconds() {
        return table.getEntry("tl").getDouble(0) / 1000.0;
    }

  /** Creates a new Limelight. */
  public Limelight() {}

  @Override
  public void periodic() {
    if (hasTarget()) {

    Pose2d visionPose = getBotPose();

    // Reject crazy jumps (important for Week Zero)
    if (visionPose.getTranslation()
        .getDistance(drivetrain.getPose().getTranslation()) < 1.5) {

        Drivetrain.addVisionMeasurement(
            visionPose,
            Timer.getFPGATimestamp() - getLatencySeconds()
        );
    }
}
  }
}

