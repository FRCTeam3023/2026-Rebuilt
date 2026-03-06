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
import frc.robot.commands.Notifications;
import frc.robot.util.PIDDisplay;
import frc.robot.util.SparkBaseSetter;
import frc.robot.util.SparkBaseSetter.SparkConfiguration;


public class IntakeActuator extends SubsystemBase {
  private final DigitalInput topLimit = new DigitalInput(5);
  private final DigitalInput bottomLimit = new DigitalInput(4);
  // private boolean previousTopLimit, previousBottomLimit;   // Probably used for homing the elevator

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
    intakeActuatorConfig.encoder // Look at pivot code from Jarvis 2025 (03/04/2026)
      .positionConversionFactor(Constants.ElevatorConstants.SPROKET_CIRCUMFERENCE / Constants.ElevatorConstants.GEARING)
      .velocityConversionFactor(Constants.ElevatorConstants.SPROKET_CIRCUMFERENCE / Constants.ElevatorConstants.GEARING);
    elevatorConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .positionWrappingEnabled(false)
      .outputRange(-Constants.GAINS.ELEVATOR.peakOutput, Constants.GAINS.ELEVATOR.peakOutput);

      elevator.getEncoder().setPosition(Constants.ElevatorConstants.MIN_ELEVATOR_EXTENSION);
      
      targetPositionEntry.setDouble(elevator.getEncoder().getPosition());
      topSwitchEntry.setBoolean(false);
      bottomSwitchEntry.setBoolean(false);
      encoderEntry.setDouble(0);
    
    SmartDashboard.putData("Elevator/Go To Target", new InstantCommand(() -> move(targetPositionEntry.getDouble(Constants.ElevatorConstants.MIN_ELEVATOR_EXTENSION))));

    SparkBaseSetter motorClosedLoopSetter = new SparkBaseSetter(new SparkConfiguration(elevator, elevatorConfig));
    motorClosedLoopSetter.setPID(Constants.GAINS.ELEVATOR);
    PIDDisplay.PIDList.addOption("Elevator Motors", motorClosedLoopSetter);
  }
  
  public void move(double targetPosition){
    if (targetPosition < Constants.ElevatorConstants.MIN_ELEVATOR_EXTENSION || targetPosition > Constants.ElevatorConstants.MAX_ELEVATOR_EXTENSION) {
      Notifications.ELEVATOR_INVALID_HEIGHT.sendImmediate(targetPosition);
      return;
    }
    elevatorController.setReference(targetPosition, SparkFlex.ControlType.kPosition);
    targetPositionEntry.setDouble(targetPosition);
  }

  public double getPosition() {
    return elevator.getEncoder().getPosition();
  }

  public Command homeCommand() {
    return new SequentialCommandGroup(
      new InstantCommand(() -> elevator.set(-0.1)),
      new ParallelRaceGroup(
        new SequentialCommandGroup(
          new WaitUntilCommand(() -> !bottomLimit.get()),
          new InstantCommand(() -> elevator.getEncoder().setPosition(Constants.ElevatorConstants.BOTTOM_LIMIT_POSITION)),
          moveCommand(Constants.ElevatorConstants.MIN_ELEVATOR_EXTENSION),
          Notifications.ELEVATOR_HOME_SUCCESS.send()
        ),
        new SequentialCommandGroup(
          new WaitCommand(3),
          new InstantCommand(() -> elevator.set(0)),
          Notifications.ELEVATOR_HOME_FAIL.send()
        )
      )
    );
  }

  public Command moveCommand(double targetPosition) {
    return new SequentialCommandGroup(
      new InstantCommand(() -> move(targetPosition)),
      new WaitUntilCommand(() -> Math.abs(elevator.getEncoder().getPosition() - targetPosition) < Constants.ElevatorConstants.SETPOINT_RANGE)
    );
  }

  @Override
  public void periodic() {
    if (!topLimit.get() && !previousTopLimit) {
      elevator.getEncoder().setPosition(Constants.ElevatorConstants.TOP_LIMIT_POSITION);
      move(Constants.ElevatorConstants.TOP_LIMIT_POSITION);
    }
    if (!bottomLimit.get() && !previousBottomLimit) {
      elevator.getEncoder().setPosition(Constants.ElevatorConstants.BOTTOM_LIMIT_POSITION);
      move(Constants.ElevatorConstants.BOTTOM_LIMIT_POSITION);
    }
    previousTopLimit = !topLimit.get();
    previousBottomLimit = !bottomLimit.get();

    outputEntry.setDouble(elevator.get());
    topSwitchEntry.setBoolean(previousTopLimit);
    bottomSwitchEntry.setBoolean(previousBottomLimit);
    encoderEntry.setDouble(elevator.getEncoder().getPosition());
  }
}