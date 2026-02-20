// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Util.PIDDisplay;
import frc.robot.Util.SparkBaseSetter;

public class Shooter extends SubsystemBase {
  SparkFlex shooter;
  SparkFlexConfig shooterConfig;
  SparkClosedLoopController shooterController;
  
  SparkMax indexer;
  SparkMaxConfig indexerConfig;
  SparkClosedLoopController indexerController;

  public Shooter() {
    shooter = new SparkFlex(Constants.CAN_DEVICES.SHOOTER_MOTOR.id, MotorType.kBrushless);
    indexer = new SparkMax(Constants.CAN_DEVICES.INDEX_MOTOR.id, MotorType.kBrushless);

    shooterConfig = new SparkFlexConfig();
    shooterConfig
    .inverted(false)
    .idleMode(IdleMode.kCoast)
    .voltageCompensation(12);
    shooterConfig.encoder
    .positionConversionFactor(1)
    .velocityConversionFactor(1);
    shooterConfig.closedLoop
    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .positionWrappingEnabled(false)
    .outputRange(-Constants.GAINS.SHOOTER.peakOutput, Constants.GAINS.SHOOTER.peakOutput);

     SparkBaseSetter closedLoopSetter = new SparkBaseSetter(new SparkBaseSetter.SparkConfiguration(shooter, shooterConfig));
    closedLoopSetter.setPID(Constants.GAINS.SHOOTER);
    PIDDisplay.PIDList.addOption("End Effector", closedLoopSetter);

    indexerConfig = new SparkMaxConfig();
    indexerConfig
    .inverted(false)
    .idleMode(IdleMode.kBrake)
    .voltageCompensation(12);
    indexerConfig.encoder
    .positionConversionFactor(1)
    .velocityConversionFactor(1);
    indexerConfig.closedLoop
    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .positionWrappingEnabled(false)
    .outputRange(-Constants.GAINS.INDEXER.peakOutput, Constants.GAINS.INDEXER.peakOutput);


  }

  public void setShooterVelocity(double rpm){
    shooterController.setSetpoint(rpm, ControlType.kVelocity);
  }

  
  public void setIndexVelocity(double rpm){
    indexerController.setSetpoint(rpm, ControlType.kVelocity);
  }
  


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
