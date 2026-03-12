// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Util.PIDDisplay;
import frc.robot.Util.SparkBaseSetter;

public class Indexer extends SubsystemBase {

  SparkMax indexer;
  SparkMaxConfig indexerConfig;
  SparkClosedLoopController indexerController;

  SparkMax agitator;
  SparkMaxConfig agitatorConfig;
  SparkClosedLoopController agitatorController;

  /** Creates a new Indexer. */
  public Indexer() {

    indexer = new SparkMax(Constants.CAN_DEVICES.INDEX_MOTOR.id, MotorType.kBrushless);

    indexerController = indexer.getClosedLoopController();

    //Indexer motor controller config
    indexerConfig = new SparkMaxConfig();
    indexerConfig
    .inverted(false)
    .idleMode(IdleMode.kBrake)
    .smartCurrentLimit(38)
    .voltageCompensation(12);
    indexerConfig.encoder
    .positionConversionFactor(1.0/Constants.INDEXER.GEAR_RATIO)
    .velocityConversionFactor(1.0/Constants.INDEXER.GEAR_RATIO);
    indexerConfig.closedLoop
    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .positionWrappingEnabled(false)
    .outputRange(-Constants.GAINS.INDEXER.peakOutput, Constants.GAINS.INDEXER.peakOutput);

    //makes the PID values accesible to elastic for testing
    SparkBaseSetter indexerSetter = new SparkBaseSetter(new SparkBaseSetter.SparkConfiguration(indexer, indexerConfig));
    indexerSetter.setPID(Constants.GAINS.INDEXER);
    PIDDisplay.PIDList.addOption("Indexer", indexerSetter);

  }

   public void setIndexVelocity(double rpm){
    indexerController.setSetpoint(rpm, ControlType.kVelocity);
  }

   public Command runIndexCommand(double RPM) {
    return new StartEndCommand(
      () -> setIndexVelocity(RPM),
      () -> setIndexVelocity(0)
      );
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
