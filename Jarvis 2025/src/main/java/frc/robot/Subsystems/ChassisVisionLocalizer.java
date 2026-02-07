// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import java.util.ArrayList;
import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Commands.Notifications;

public class ChassisVisionLocalizer extends SubsystemBase {
  public boolean enabled = true;
  private int overrunCount = 0;
  private int previousIndex = -1;

  private class PoseEntry {
    public final Pose2d pose;
    public final double timeStamp;

    public PoseEntry(Pose2d pose, double timeStamp) {
      this.pose = pose;
      this.timeStamp = timeStamp;
    }
  }

  private class NavCam {
    public final int index;
    private final PhotonCamera cam;
    private final PhotonPoseEstimator poseEstimator;
    private final Field2d field = new Field2d();
    private final GenericEntry trustEntry;

    private double[] calibrationEntries = new double[7];
    private int currentCalibrationEntry = 0;
    
    public NavCam(int index) {
      this.index = index;
      cam = new PhotonCamera(getName());
      trustEntry = nTable.getTopic("Trusting %s".formatted(index)).getGenericEntry();

      poseEstimator = new PhotonPoseEstimator(
        Constants.PhotonConstants.APRIL_TAG_FIELD_LAYOUT, 
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, 
        Constants.PhotonConstants.ROBOT_TO_CAMERAS[index]
      );
      poseEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

      trustEntry.setBoolean(false);
      SmartDashboard.putData("Drivetrain/Vision/" + getName(), field);
    }

    private boolean shouldReject(PhotonTrackedTarget bestTarget) {
      return 
           bestTarget.getPoseAmbiguity() > Constants.PhotonConstants.MAX_AMBIGUITY
        || bestTarget.getBestCameraToTarget().getTranslation().getNorm() > Constants.PhotonConstants.MAX_DISTANCE
      ;
    }

    public List<PoseEntry> getRobotPoses() {
      List<PoseEntry> poses = new ArrayList<PoseEntry>();
      cam.getAllUnreadResults().forEach(r -> {
        if (r.hasTargets() && !shouldReject(r.getBestTarget())) {
          poses.add(new PoseEntry(poseEstimator.update(r).get().estimatedPose.toPose2d(), r.getTimestampSeconds()));

          if (calibrationEntry.getDouble(-1) == index && currentCalibrationEntry != Constants.PhotonConstants.NUM_CALIBRATION_ENTRIES) {
            Transform3d targetToCam = r.getBestTarget().getBestCameraToTarget().inverse();
            Pose3d robotToCam = Constants.PhotonConstants.ROBOT_TO_CALIBRATION.transformBy(targetToCam);

            currentCalibrationEntry++;
            calibrationEntriesEntry.setInteger(currentCalibrationEntry);

            calibrationEntries[0] += robotToCam.getX();
            calibrationEntries[1] += robotToCam.getY();
            calibrationEntries[2] += robotToCam.getZ();
            calibrationEntries[3] += robotToCam.getRotation().getX();
            calibrationEntries[4] += robotToCam.getRotation().getY();
            calibrationEntries[5] += robotToCam.getRotation().getZ();
            calibrationEntries[6] += r.getBestTarget().getPoseAmbiguity();

            if (currentCalibrationEntry == Constants.PhotonConstants.NUM_CALIBRATION_ENTRIES) {
              for (int i = 0; i < 7; i++) calibrationEntries[i] /= Constants.PhotonConstants.NUM_CALIBRATION_ENTRIES;

              calibrationXEntry.setDouble(calibrationEntries[0]);
              calibrationYEntry.setDouble(calibrationEntries[1]);
              calibrationZEntry.setDouble(calibrationEntries[2]);
              calibrationRollEntry.setDouble(calibrationEntries[3]);
              calibrationPitchEntry.setDouble(calibrationEntries[4]);
              calibrationYawEntry.setDouble(calibrationEntries[5]);
              calibrationAmbiguity.setDouble(calibrationEntries[6]);
            }
          }
        }
      });

      trustEntry.setBoolean(poses.size() > 0);
      if (poses.size() > 0) field.setRobotPose(poses.get(poses.size() - 1).pose);
      SmartDashboard.putData("Drivetrain/Vision/" + getName(), field);

      return poses;
    }

    public String getName() {
      return "navCam%s".formatted(index);
    }
  }

  private final NetworkTable nTable = NetworkTableInstance.getDefault().getTable("SmartDashboard/Drivetrain/Vision");

  private final GenericEntry enabledEntry = nTable.getTopic("Enabled").getGenericEntry();
  private final GenericEntry calibrationEntry = nTable.getTopic("Calibration Index").getGenericEntry();
  private final GenericEntry calibrationXEntry = nTable.getTopic("Cal Translation X").getGenericEntry();
  private final GenericEntry calibrationYEntry = nTable.getTopic("Cal Translation Y").getGenericEntry();
  private final GenericEntry calibrationZEntry = nTable.getTopic("Cal Translation Z").getGenericEntry();
  private final GenericEntry calibrationRollEntry = nTable.getTopic("Cal Roll").getGenericEntry();
  private final GenericEntry calibrationPitchEntry = nTable.getTopic("Cal Pitch").getGenericEntry();
  private final GenericEntry calibrationYawEntry = nTable.getTopic("Cal Yaw").getGenericEntry();
  private final GenericEntry calibrationAmbiguity = nTable.getTopic("Cal Ambiguity").getGenericEntry();
  private final GenericEntry calibrationEntriesEntry = nTable.getTopic("Cal Entries").getGenericEntry();

  private NavCam[] navCams = {
    new NavCam(0),
    new NavCam(1),
    new NavCam(2),
    new NavCam(3)
  };

  public ChassisVisionLocalizer() {
    PortForwarder.add(5800, "navCams01.local", 5800);
    PortForwarder.add(5800, "navCams23.local", 5800);
    enabledEntry.setBoolean(enabled);

    calibrationEntry.setInteger(-1);
    calibrationXEntry.setDouble(0);
    calibrationYEntry.setDouble(0);
    calibrationZEntry.setDouble(0);
    calibrationRollEntry.setDouble(0);
    calibrationPitchEntry.setDouble(0);
    calibrationYawEntry.setDouble(0);
    calibrationAmbiguity.setDouble(0);
    calibrationEntriesEntry.setInteger(0);
  }
  
  @Override
  public void periodic() {
    if (!enabled) return;
    if (overrunCount > 2) {
      enabled = false;
      enabledEntry.setBoolean(false);
      Notifications.VISION_TIMER_EXCEEDED.sendImmediate();
    }

    int currentIndex =(int)calibrationEntry.getInteger(-1);
    if (previousIndex != currentIndex && currentIndex >= 0 && currentIndex <= 3) {
      navCams[currentIndex].currentCalibrationEntry = 0;
      for (int i = 0; i < 7; i++) navCams[currentIndex].calibrationEntries[i] = 0;
    }
    previousIndex = currentIndex;

    for (NavCam navCam : navCams) {
      navCam.getRobotPoses().forEach(pose -> {
        Drivetrain.addVisionMeasurement(pose.pose, pose.timeStamp);
      });
    }
  }
}
