// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.Constants;

/** Add your docs here. */
public class ReefScoringLocation {
    public final Translation3d position;
    public final Rotation2d alpha, beta;
    public final double innerRadius, outerRadius;
    public final Rotation2d lowerAngle, upperAngle;

    public class SubsystemState {
        public final double height;
        public final Rotation2d pivotAngle;
        public final Pose2d robotPose;

        public SubsystemState(double height, Rotation2d pivotAngle, Pose2d robotPose) {
            this.height = height;
            this.pivotAngle = pivotAngle;
            this.robotPose = robotPose;
        }
    }

    public ReefScoringLocation(Translation3d position, Rotation2d alpha, Rotation2d beta) {
        this.position = position;
        this.alpha = alpha;
        this.beta = beta;
        double[] range = computeAcceptableScoringRange();
        innerRadius = range[0];
        outerRadius = range[1];
        lowerAngle = beta.minus(Constants.EndEffectorConstants.ACCEPTABLE_SCORING_RANGE);
        upperAngle = beta.plus(Constants.EndEffectorConstants.ACCEPTABLE_SCORING_RANGE);
    }

    public double getDistanceWhenScoring(Rotation2d pivotAngle) {
        return 
            Constants.EndEffectorConstants.CORAL_PROTRUSION_LENGTH
                * Constants.PivotConstants.END_MOUNT_ANGLE.plus(pivotAngle).getCos()
            + Constants.PivotConstants.LENGTH * pivotAngle.getCos();
    }

    public double[] computeAcceptableScoringRange() {
        Rotation2d difference = alpha.minus(Constants.PivotConstants.END_MOUNT_ANGLE);
        double[] candidates = {
            alpha.minus(Constants.PivotConstants.END_MOUNT_ANGLE).plus(Constants.EndEffectorConstants.ACCEPTABLE_SCORING_RANGE).getRadians(),
            alpha.minus(Constants.PivotConstants.END_MOUNT_ANGLE).minus(Constants.EndEffectorConstants.ACCEPTABLE_SCORING_RANGE).getRadians(),
            -Math.atan2(
                Constants.EndEffectorConstants.CORAL_PROTRUSION_LENGTH * alpha.getSin()
                + Constants.PivotConstants.LENGTH * difference.getSin(),
                Constants.EndEffectorConstants.CORAL_PROTRUSION_LENGTH * alpha.getCos()
                + Constants.PivotConstants.LENGTH * difference.getCos()
            )
        };

        double maxAngle = Math.max(candidates[0], candidates[1]);
        double minAngle = Math.min(candidates[0], candidates[1]);
        double maxDistance = getDistanceWhenScoring(Rotation2d.fromRadians(maxAngle));
        double minDistance = getDistanceWhenScoring(Rotation2d.fromRadians(minAngle));

        if (candidates[2] < maxAngle && candidates[2] > minAngle) {
            double criticalDistance = getDistanceWhenScoring(Rotation2d.fromRadians(candidates[2]));
            maxDistance = Math.max(maxDistance, criticalDistance);
            minDistance = Math.min(minDistance, criticalDistance);
        }

        return new double[] {minDistance, maxDistance};
    }

    private static boolean validate(double phi, Rotation2d alpha) {
        return 
            phi != Double.NaN
            && Math.abs(phi + Constants.PivotConstants.END_MOUNT_ANGLE.getRadians() - alpha.getRadians()) < Constants.EndEffectorConstants.ACCEPTABLE_SCORING_RANGE.getRadians();
    }

    public SubsystemState getStateAt(Pose2d robot) {
        double A = 
            Constants.EndEffectorConstants.CORAL_PROTRUSION_LENGTH * Constants.PivotConstants.END_MOUNT_ANGLE.getCos()
            + Constants.PivotConstants.LENGTH;
        double B =
            -Constants.EndEffectorConstants.CORAL_PROTRUSION_LENGTH * Constants.PivotConstants.END_MOUNT_ANGLE.getSin();

        double arccos = 
            Math.acos(new Translation2d(position.getX(), position.getY()).getDistance(robot.getTranslation()) / Math.sqrt(A * A + B * B));
        double arctan = 
            Math.atan2(B, A);

        double phi1 = arctan - arccos;
        double phi2 = arctan + arccos;
        double angle;

        if (validate(phi1, alpha)) angle = phi1;
        else if (validate(phi2, alpha)) angle = phi2;
        else {
            System.out.println("No possible pivot angle was found given the current robot position and target!");
            return new SubsystemState(0, Rotation2d.fromDegrees(-90), robot);
        }

        return new SubsystemState(
            position.getZ() 
                - Constants.EndEffectorConstants.CORAL_PROTRUSION_LENGTH 
                    * Math.sin(Constants.PivotConstants.END_MOUNT_ANGLE.getRadians() + angle)
                - Constants.PivotConstants.LENGTH * Math.sin(angle), 
            Rotation2d.fromRadians(angle),
            new Pose2d(
                robot.getTranslation(),
                Rotation2d.fromRadians(Math.atan2(position.getY() - robot.getY(), position.getX() - robot.getX()))
            )
        );
    }
}
