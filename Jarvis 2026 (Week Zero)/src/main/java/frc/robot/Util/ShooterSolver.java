// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Util;

import java.util.Optional;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.Subsystems.Drivetrain;

/** Add your docs here. */
public class ShooterSolver {
    private final Drivetrain drivetrain;
    private DriverStation.Alliance lastAlliance = null;
    private Pose2d cachedHubPosition = null;

    public ShooterSolver(Drivetrain drivetrain) {
    this.drivetrain = drivetrain;

}

   // public final Pose2d HUB_POSE = getHubCenterPose();

     private static final AprilTagFieldLayout layout =
        AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);

        
    public Pose2d getHubCenterPose() {

        Optional<DriverStation.Alliance> allianceOpt = DriverStation.getAlliance();
        

        if (allianceOpt.isEmpty()) {
        throw new RuntimeException("Alliance unknown!");
        } else if (lastAlliance == allianceOpt.get()){
            return cachedHubPosition;
        }

        int tagID;
        double hubOffset;
        
        DriverStation.Alliance alliance = allianceOpt.get();
        lastAlliance = alliance;

        switch (alliance) {
            case Blue:
                tagID = Constants.HUB.BLUE_TAG;
                hubOffset = Constants.HUB.BLUE_OFFSET;
                break;
            case Red:
                tagID = Constants.HUB.RED_TAG;
                hubOffset = Constants.HUB.RED_OFFSET;
                break;
            default:
                throw new RuntimeException("Unknown alliance: cannot determine hub!");
        }

        Optional<Pose2d> tagPoseOpt = layout.getTagPose(tagID)
                                           .map(pose3d -> pose3d.toPose2d());

        if (tagPoseOpt.isPresent()) {
            Pose2d tagPose = tagPoseOpt.get();

            // Transform: -1 meter in the tag's X direction (toward hub center)
            Transform2d hubPoseOffset = new Transform2d(
                new Translation2d(hubOffset, 0.0),   // -1m forward from tag
                new Rotation2d()                 // no rotation change
            );

            cachedHubPosition = tagPose.plus(hubPoseOffset);
            return tagPose.plus(hubPoseOffset);
        } else {
            throw new RuntimeException("Blue hub tag not found in field layout!");
        }
    }

    public double getTargetDistance(){

    Pose2d hubPose = getHubCenterPose();
    double distance = hubPose.getTranslation()
    .getDistance(drivetrain.getPose().getTranslation());

    return distance;

    }

    public double getTargetRPM(){
       

        double distance = getTargetDistance();

        double targetVelocity = Math.sqrt(distance*distance*(9.81) 
         / Math.pow(2*Math.cos(Math.toRadians(Constants.SHOOTER.SHOOTER_ANGLE)), 2) * Constants.HUB.HUB_TARGET_HEIGHT - distance * Math.tan(Math.toRadians(Constants.SHOOTER.SHOOTER_ANGLE)));
        
        return targetVelocity * Constants.SHOOTER.VEL_TO_RPM;
    }
 


public double getAngleToHub(){
    Translation2d robot = drivetrain.getPose().getTranslation();
    Translation2d hub = getHubCenterPose().getTranslation();

    return hub.minus(robot).getAngle().getRadians();
}


/*public double velocityResultant(Drivetrain drivetrain){

    Rotation2d robotHeading = drivetrain.getHeadingRaw();
    Pose2d velx = drivetrain.getPose();

    

    return 0;
}
*/

}
