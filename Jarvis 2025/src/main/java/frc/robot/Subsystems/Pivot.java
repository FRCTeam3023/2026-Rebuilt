// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Util.Elastic;
import frc.robot.Util.PIDDisplay;
import frc.robot.Util.SparkBaseSetter;
import frc.robot.Util.SparkBaseSetter.SparkConfiguration;

public class Pivot extends SubsystemBase {
  private final SparkFlex pivot;
  private final SparkFlexConfig pivotConfig;
  private final SparkClosedLoopController pivotController;

  private final NetworkTable nTables = NetworkTableInstance.getDefault().getTable("SmartDashboard/Pivot");
  private final GenericEntry targetPositionEntry = nTables.getTopic("Target").getGenericEntry();
  private final GenericEntry encoderEntry = nTables.getTopic("Encoder").getGenericEntry();
  private final GenericEntry speedEntry = nTables.getTopic("Encoder Speed").getGenericEntry();

  public Pivot() {
    pivot = new SparkFlex(Constants.CAN_DEVICES.PIVOT.id, MotorType.kBrushless);
    
    pivotController = pivot.getClosedLoopController();

    pivotConfig = new SparkFlexConfig();
    pivotConfig
      .inverted(true)
      .idleMode(IdleMode.kBrake)
      .voltageCompensation(12);
    pivotConfig.encoder
      .positionConversionFactor(2 * Math.PI / Constants.PivotConstants.GEARING)
      .velocityConversionFactor(1 / Constants.PivotConstants.GEARING);
    pivotConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .positionWrappingEnabled(false)
      .outputRange(-Constants.GAINS.PIVOT.peakOutput, Constants.GAINS.PIVOT.peakOutput);

    pivot.getEncoder().setPosition(-Math.PI / 2);

    targetPositionEntry.setDouble(Math.toDegrees(pivot.getEncoder().getPosition()));
    encoderEntry.setDouble(0);
    speedEntry.setDouble(0);

    SmartDashboard.putData("Pivot/Go To Target", new InstantCommand(() -> setAngle(Rotation2d.fromDegrees(targetPositionEntry.getDouble(-90)))));

    SparkBaseSetter closedLoopSetter = new SparkBaseSetter(new SparkConfiguration(pivot, pivotConfig));
    closedLoopSetter.setPID(Constants.GAINS.PIVOT);
    PIDDisplay.PIDList.addOption("Pivot", closedLoopSetter);
  }

  public void setAngle(Rotation2d angle) {
    pivotController.setReference(angle.getRadians(), ControlType.kPosition);
    targetPositionEntry.setDouble(angle.getDegrees());
  }

  public Rotation2d getAngle() {
    return Rotation2d.fromRadians(pivot.getEncoder().getPosition());
  }

  public double getSpeed() {
    return pivot.getEncoder().getVelocity();
  }

  public Command setAngleCommand(Rotation2d angle) {
    return new ParallelDeadlineGroup(
      new WaitUntilCommand(() -> Math.abs(getAngle().minus(angle).getRadians()) < Constants.PivotConstants.POSITION_TOLERANCE.getRadians()),
      new InstantCommand(() -> {
        setAngle(angle);
        if (angle.getDegrees() != Constants.PivotConstants.TRAVEL_POSITION.getDegrees()) {
          Elastic.selectTab(angle.getDegrees() > 0 ? "EE View Up" : "EE View Down");
          Drivetrain.alignMode = true;
        } else {
          Elastic.selectTab("Teleoperated");
          Drivetrain.alignMode = false;
        }
      })
    );
  }

  public Command climbModeCommand() {
    return new InstantCommand(() -> {
      Elastic.selectTab("ËE View Down");
      setAngle(Constants.PivotConstants.CLIMB_POSITION);
    }).andThen(new WaitUntilCommand(() -> false));
  }

  @Override
  public void periodic() {
    encoderEntry.setDouble(getAngle().getDegrees());
    speedEntry.setDouble(getSpeed());
  }
}
