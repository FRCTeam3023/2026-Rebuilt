// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;



import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Util.PIDDisplay;
import frc.robot.Util.SparkBaseSetter;

public class Intake extends SubsystemBase {
  SparkFlex actuator;
  SparkFlexConfig actuatorConfig;
  SparkClosedLoopController actuatorController;
  
  SparkMax endEffector;
  SparkMaxConfig endEffectorConfig;
  SparkClosedLoopController endEffectorController;

  DigitalInput frameLimitSwitch = new DigitalInput(1);
  private boolean wasAtLimit = false;
  double actuatorEncoderOffset = 0;
  
  /** Creates a new Intake. */
  public Intake() {
    actuator = new SparkFlex(Constants.CAN_DEVICES.INTAKE_ACTUATOR.id, MotorType.kBrushless);
    endEffector = new SparkMax(Constants.CAN_DEVICES.INTAKE_MOTOR.id, MotorType.kBrushless);

    actuatorController = actuator.getClosedLoopController();
    endEffectorController = endEffector.getClosedLoopController();

    // Actuator Config
    actuatorConfig = new SparkFlexConfig();
    actuatorConfig
    .inverted(false)
    .idleMode(IdleMode.kBrake)
    .voltageCompensation(12);
    actuatorConfig.encoder
    .positionConversionFactor(1.0/Constants.INTAKE.ACTUATOR_GEAR_RATIO)
    .velocityConversionFactor(1.0/Constants.INTAKE.ACTUATOR_GEAR_RATIO);
    actuatorConfig.closedLoop
    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .positionWrappingEnabled(false)
    .outputRange(-Constants.GAINS.INTAKE_ACTUATOR.peakOutput, Constants.GAINS.INTAKE_ACTUATOR.peakOutput);
    actuatorConfig.softLimit
    .reverseSoftLimit(90)
    .forwardSoftLimit(actuatorEncoderOffset)
    .reverseSoftLimitEnabled(true)
    .forwardSoftLimitEnabled(true);

     SparkBaseSetter intakeActuatorSetter = new SparkBaseSetter(new SparkBaseSetter.SparkConfiguration(actuator, actuatorConfig));
    intakeActuatorSetter.setPID(Constants.GAINS.INTAKE_ACTUATOR);
    PIDDisplay.PIDList.addOption("Intake actuator", intakeActuatorSetter);


    //End Efector Config
    endEffectorConfig = new SparkMaxConfig();
    endEffectorConfig
    .inverted(false)
    .idleMode(IdleMode.kBrake)
    .voltageCompensation(12);
    endEffectorConfig.encoder
    .positionConversionFactor(1.0/Constants.INTAKE.MANIPULATOR_GEAR_RATIO)
    .velocityConversionFactor(1.0/Constants.INTAKE.MANIPULATOR_GEAR_RATIO); 
    endEffectorConfig.closedLoop
    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .positionWrappingEnabled(false)
    .outputRange(-Constants.GAINS.INTAKE_MANIPULATOR.peakOutput, Constants.GAINS.INTAKE_MANIPULATOR.peakOutput);

    SparkBaseSetter intakeEndEffectorSetter = new SparkBaseSetter(new SparkBaseSetter.SparkConfiguration(endEffector, endEffectorConfig));
    intakeEndEffectorSetter.setPID(Constants.GAINS.INTAKE_ACTUATOR);
    PIDDisplay.PIDList.addOption("Intake End Efector", intakeEndEffectorSetter);


  }

  public boolean isAtLimit() {
    return frameLimitSwitch.get();
  }

  public void moveIntake(double target){
    if(target > 0 || target < -90){
    DriverStation.reportWarning("Invalid intake target: " + target, false);
    return;
  }
  actuatorController.setSetpoint(target, ControlType.kMAXMotionPositionControl);
  }

  public void runIntake(double rpm){
    endEffectorController.setSetpoint(rpm, ControlType.kVelocity);
  }

  public Command intakeManualCommand(){
    return new StartEndCommand(
      () -> runIntake(200),
      () -> runIntake(0)
    );
  }

  public Command intakeCommand(){
    return new StartEndCommand(
    () -> { 
      moveIntake(0);
      runIntake(200); 
        },
    () -> {
      runIntake(0);
      moveIntake(-90);
        }
    );
  }

  @Override
  public void periodic() {
    boolean atLimit = isAtLimit();
    if(atLimit && !wasAtLimit){
      actuator.getEncoder().setPosition(0);
    }
    wasAtLimit = atLimit;
  }
}
