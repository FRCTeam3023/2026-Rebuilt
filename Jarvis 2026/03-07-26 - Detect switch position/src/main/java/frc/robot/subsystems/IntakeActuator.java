package frc.robot.subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;


public class IntakeActuator extends SubsystemBase {
  /*private final DigitalInput topLimit = new DigitalInput(5);
  private final DigitalInput bottomLimit = new DigitalInput(4);
  private boolean previousTopLimit, previousBottomLimit;*/

  private SparkFlex intakeActuator;
  private SparkFlexConfig intakeActuatorConfig;
  private SparkClosedLoopController intakeActuatorController;
/*
  private final NetworkTable nTable = NetworkTableInstance.getDefault().getTable("SmartDashboard/Elevator");
  private final GenericEntry targetPositionEntry = nTable.getTopic("Target").getGenericEntry();
  private final GenericEntry outputEntry = nTable.getTopic("Output").getGenericEntry();
  private final GenericEntry topSwitchEntry = nTable.getTopic("Top Switch").getGenericEntry();
  private final GenericEntry bottomSwitchEntry = nTable.getTopic("Bottom Switch").getGenericEntry();
  private final GenericEntry encoderEntry = nTable.getTopic("Encoder").getGenericEntry();
*/    // Previously used for the elevator, leaving as a reference

  public IntakeActuator() {
    intakeActuator = new SparkFlex(Constants.CAN_ID.INTAKE_ACTUATOR_ID, MotorType.kBrushless);
    intakeActuatorController = intakeActuator.getClosedLoopController();

    intakeActuatorConfig = new SparkFlexConfig();
    intakeActuatorConfig
      .inverted(false)
      .idleMode(IdleMode.kBrake)
      .voltageCompensation(12);
    intakeActuatorConfig.encoder
      .positionConversionFactor(2 * Math.PI / Constants.IntakeActuatorConstants.GEARING)
      .velocityConversionFactor(1 / Constants.IntakeActuatorConstants.GEARING);
    intakeActuatorConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .positionWrappingEnabled(false)
      .outputRange(-Constants.GAINS.INTAKE_ACTUATOR.peakOutput, Constants.GAINS.INTAKE_ACTUATOR.peakOutput);

      intakeActuator.getEncoder().setPosition(Constants.IntakeActuatorConstants.RETRACTED_POSITION);
      
      /*targetPositionEntry.setDouble(intakeActuator.getEncoder().getPosition());
      deployedEntry.setBoolean(false);
      retractedEntry.setBoolean(false);
      encoderEntry.setDouble(0);*/    // All 4 are used to display info; names have only been edited here
    
    // SmartDashboard.putData("Intake Actuator/Go To Target", new InstantCommand(() -> move(targetPositionEntry.getDouble(Constants.IntakeActuatorConstants.RETRACTED_POSITION))));

    /*SparkBaseSetter motorClosedLoopSetter = new SparkBaseSetter(new SparkConfiguration(intakeActuator, intakeActuatorConfig));
    motorClosedLoopSetter.setPID(Constants.GAINS.INTAKE_ACTUATOR);
    PIDDisplay.PIDList.addOption("Intake Actuator Motors", motorClosedLoopSetter);*/
  }
  
  public void move(double targetPosition){
    if (targetPosition < Constants.IntakeActuatorConstants.RETRACTED_POSITION /* || targetPosition > Constants.IntakeActuatorConstants.MAX_ELEVATOR_EXTENSION */) { // FIX ME - Not sure what the equivalent constant would be (03/06/26)
      // Notifications.ELEVATOR_INVALID_HEIGHT.sendImmediate(targetPosition);
      return;
    }
    intakeActuatorController.setSetpoint(targetPosition, SparkFlex.ControlType.kPosition);
    // targetPositionEntry.setDouble(targetPosition);
  }

  public double getPosition() {
    return intakeActuator.getEncoder().getPosition();
  }

  /*public Command homeCommand() {
    return new SequentialCommandGroup(
      new InstantCommand(() -> intakeActuator.set(-0.1)),
      new ParallelRaceGroup(
        new SequentialCommandGroup(
          new WaitUntilCommand(() -> !bottomLimit.get()),
          new InstantCommand(() -> intakeActuator.getEncoder().setPosition(Constants.ElevatorConstants.BOTTOM_LIMIT_POSITION)),
          moveCommand(Constants.IntakeActuatorConstants.RETRACTED_POSITION),
          Notifications.ELEVATOR_HOME_SUCCESS.send()
        ),
        new SequentialCommandGroup(
          new WaitCommand(3),
          new InstantCommand(() -> intakeActuator.set(0)),
          Notifications.ELEVATOR_HOME_FAIL.send()
        )
      )
    );
  }*/

  public Command moveCommand(double targetPosition) {
    return new SequentialCommandGroup(
      new InstantCommand(() -> move(targetPosition)),
      new WaitUntilCommand(() -> Math.abs(intakeActuator.getEncoder().getPosition() - targetPosition) < Constants.IntakeActuatorConstants.SETPOINT_RANGE.getRadians())
    );
  }

  @Override
  public void periodic() {
    /*if (!topLimit.get() && !previousTopLimit) {
      intakeActuator.getEncoder().setPosition(Constants.ElevatorConstants.TOP_LIMIT_POSITION);
      move(Constants.ElevatorConstants.TOP_LIMIT_POSITION);
    }
    if (!bottomLimit.get() && !previousBottomLimit) {
      intakeActuator.getEncoder().setPosition(Constants.ElevatorConstants.BOTTOM_LIMIT_POSITION);
      move(Constants.ElevatorConstants.BOTTOM_LIMIT_POSITION);
    }
    previousTopLimit = !topLimit.get();
    previousBottomLimit = !bottomLimit.get();*/

    /*outputEntry.setDouble(intakeActuator.get());
    topSwitchEntry.setBoolean(previousTopLimit);
    bottomSwitchEntry.setBoolean(previousBottomLimit);
    encoderEntry.setDouble(intakeActuator.getEncoder().getPosition());*/
  }

  public void intakePosition() {
    moveCommand(30);
  }
}