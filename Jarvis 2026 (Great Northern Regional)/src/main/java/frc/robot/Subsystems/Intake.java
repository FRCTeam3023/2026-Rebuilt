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

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
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

  // DigitalInput frameLimitSwitch = new DigitalInput(5);   // Old code by Nathan (03/09/26)
  private boolean wasAtLimit = false;
  double actuatorEncoderOffset = 0;
  
  DigitalInput lowerLimitSwitch = new DigitalInput(5);
  private boolean lowerLimit = false;
  DigitalInput upperLimitSwitch = new DigitalInput(6);
  private boolean upperLimit = false;
  private double intakeActuatorAngle = 0;
  private double intakeActuatorAbsoluteAngle = 0;

  private NetworkTableInstance inst;
  private NetworkTable intakeTable = NetworkTableInstance.getDefault().getTable("Intake");
  private BooleanPublisher lowerLimitPub;
  private BooleanPublisher upperLimitPub;
  private DoublePublisher intakeActuatorAnglePub;
  private DoublePublisher intakeActuatorAbsoluteAnglePub;

  // For troubleshooting (03/09/26)
  private boolean intakeTest = false;
  private BooleanPublisher intakeTestPub;

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
    .smartCurrentLimit(30)
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

    // Network tables
    inst = NetworkTableInstance.getDefault();
    intakeTable = inst.getTable("Limit Switches");
    lowerLimitPub = intakeTable.getBooleanTopic("Lower limit switch").publish();
    upperLimitPub = intakeTable.getBooleanTopic("Upper limit switch").publish();
    intakeActuatorAnglePub = intakeTable.getDoubleTopic("Intake actuator angle").publish();
    intakeActuatorAbsoluteAnglePub = intakeTable.getDoubleTopic("Intake actuator absolute angle").publish();

    // For troubleshooting (03/09/26)
    intakeTestPub = intakeTable.getBooleanTopic("Intake test").publish();
  }

  public boolean isAtLimit() {
    // return frameLimitSwitch.get();
    return lowerLimitSwitch.get();
  }

  public boolean LowerLimit() {
    // lowerLimitSwitch = frameLimitSwitch;
    return lowerLimitSwitch.get();
  }

  public boolean UpperLimit() {
    return upperLimitSwitch.get();
  }

  public double IntakeActuatorAngle() {
    // intakeActuatorAngle = 10.0;
    return intakeActuatorAngle;
  }

  public double IntakeActuatorAbsoluteAngle() {
    return intakeActuatorAbsoluteAngle;
  }

  public boolean IntakeTest() {
    return intakeTest;
  }

  public void setLowerLimit() {
    /*if (lowerLimit = true) {
      actuatorController.setSetpoint()    // Stops the actuator
    } else {*/
      moveIntakeTest();    // Moves the actuator into place
    // }
  }

  public void moveIntake(double target){
    /*if(target > 0 || target < -90){   // Commented for troubleshooting (03/10/26)
    DriverStation.reportWarning("Invalid intake target: " + target, false);
    return;
  }*/
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
  //Make Better Intake Sequence

  public Command moveIntakeTest() {
    return new StartEndCommand(
    () -> { 
      moveIntake(0);
      intakeTest = true;    // For troubleshooting (03/09/26)
        },
    () -> {
      moveIntake(-90/360);
        }
    );
  }

  @Override
  public void periodic() {
    boolean atLimit = isAtLimit();
    /*if(atLimit && !wasAtLimit){   // Commented for troubleshooting (03/10/26)
      actuator.getEncoder().setPosition(0);
    }*/
    wasAtLimit = atLimit;

    lowerLimitPub.set(LowerLimit());
    upperLimitPub.set(UpperLimit());
    intakeActuatorAnglePub.set(Math.toDegrees(actuator.getEncoder().getPosition()));
    intakeActuatorAbsoluteAnglePub.set(Math.toDegrees(actuator.getAbsoluteEncoder().getPosition()));

    intakeTestPub.set(IntakeTest());
  }
}
