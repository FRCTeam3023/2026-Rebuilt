package frc.robot.Subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.ControlPanel;
import frc.robot.Util.PIDDisplay;
import frc.robot.Util.SparkBaseSetter;

public class Climber extends SubsystemBase {
  private final SparkMax climber;
  private final SparkMaxConfig climberConfig;
  private final SparkClosedLoopController climberController;
  private final DigitalInput innerLimit = new DigitalInput(7);

  private boolean previousLimit = false;

  private final NetworkTable nTable = NetworkTableInstance.getDefault().getTable("SmartDashboard/Climber");
  private final GenericEntry targetPositionEntry = nTable.getTopic("Target").getGenericEntry();
  private final GenericEntry innerSwitchEntry = nTable.getTopic("Switch").getGenericEntry();
  private final GenericEntry encoderEntry = nTable.getTopic("Encoder").getGenericEntry();

  public Climber() {
    climber = new SparkMax(Constants.CAN_DEVICES.CLIMBER.id, MotorType.kBrushless);

    climberController = climber.getClosedLoopController();

    climberConfig = new SparkMaxConfig();
    climberConfig
      .inverted(true)
      .idleMode(IdleMode.kBrake)
      .voltageCompensation(12);
    climberConfig.encoder
      .positionConversionFactor(1 / Constants.ClimberConstants.GEARING)
      .velocityConversionFactor(1 / Constants.ClimberConstants.GEARING);
    climberConfig.closedLoop
      .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
      .positionWrappingEnabled(false)
      .outputRange(-Constants.GAINS.CLIMBER.peakOutput, Constants.GAINS.CLIMBER.peakOutput);

    climber.getEncoder().setPosition(Constants.ClimberConstants.MAX_ROTATION.getRotations());
      
    targetPositionEntry.setDouble(climber.getEncoder().getPosition());
    innerSwitchEntry.setBoolean(false);
    encoderEntry.setDouble(0);
      
    SmartDashboard.putData("Climber/Go To Target", new InstantCommand(() -> setPosition(Rotation2d.fromDegrees(targetPositionEntry.getDouble(Constants.ClimberConstants.MAX_ROTATION.getDegrees())))));

    SparkBaseSetter closedLoopSetter = new SparkBaseSetter(new SparkBaseSetter.SparkConfiguration(climber, climberConfig));
    closedLoopSetter.setPID(Constants.GAINS.CLIMBER);
    PIDDisplay.PIDList.addOption("Climber", closedLoopSetter);
  }

  public void setPosition(Rotation2d target) {
    targetPositionEntry.setDouble(target.getDegrees());
    climberController.setReference(target.getRotations(), ControlType.kPosition);
  }

  public void setVelocity(double speed) {
    climberController.setReference(speed, ControlType.kVelocity);
  }

  public Command climbCommand(Rotation2d target) {
    return new SequentialCommandGroup(
      new InstantCommand(() -> setPosition(target)),
      new WaitUntilCommand(() -> Math.abs(climber.getEncoder().getPosition() - target.getRotations()) < Constants.ClimberConstants.SETPOINT_RANGE.getRotations())
    ).finallyDo(() -> setPosition(Rotation2d.fromRotations(climber.getEncoder().getPosition())));
  }

  // public Command climbCommand(double speed) {
  //   return new RunCommand(() -> setVelocity(speed))
  //     .finallyDo(() -> {
  //       setVelocity(0);
  //       setPosition(Rotation2d.fromRotations(climber.getEncoder().getPosition()));
  //     });
  // }

  @Override
  public void periodic() {
    //if (!innerLimit.get() && !previousLimit) climber.getEncoder().setPosition(Constants.ClimberConstants.MAX_ROTATION.getRotations());
    previousLimit = !innerLimit.get();

    innerSwitchEntry.setBoolean(previousLimit);
    encoderEntry.setDouble(360 * climber.getEncoder().getPosition());

    if (climber.getEncoder().getPosition() * 360 < 125) {
      ControlPanel.setClimbMode(true);
    }
  }
}
