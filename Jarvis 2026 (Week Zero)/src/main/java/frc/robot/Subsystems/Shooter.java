// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Util.PIDDisplay;
import frc.robot.Util.ShooterSolver;
import frc.robot.Util.SparkBaseSetter;

public class Shooter extends SubsystemBase {

  private ShooterSolver shooterSolver;
  private Drivetrain drivetrain;

  SparkFlex shooter;
  SparkFlexConfig shooterConfig;
  SparkClosedLoopController shooterController;
  
  SparkMax indexer;
  SparkMaxConfig indexerConfig;
  SparkClosedLoopController indexerController;

  private final PIDController aimPID = new PIDController(5, 0, 0);

    
  
    public Shooter(ShooterSolver shooterSolver, Drivetrain drivetrain) {
      this.shooterSolver = shooterSolver;
      this.drivetrain = drivetrain;

    shooter = new SparkFlex(Constants.CAN_DEVICES.SHOOTER_MOTOR.id, MotorType.kBrushless);
    indexer = new SparkMax(Constants.CAN_DEVICES.INDEX_MOTOR.id, MotorType.kBrushless);

    shooterController = shooter.getClosedLoopController();
    indexerController = indexer.getClosedLoopController();

    //Shooter motor controller config
    shooterConfig = new SparkFlexConfig();
    shooterConfig
    .inverted(false)
    .idleMode(IdleMode.kCoast)
    .voltageCompensation(12);
    shooterConfig.encoder
    .positionConversionFactor(1.0/Constants.SHOOTER.GEAR_RATIO)
    .velocityConversionFactor(1.0/Constants.SHOOTER.GEAR_RATIO);
    shooterConfig.closedLoop
    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    .positionWrappingEnabled(false)
    .outputRange(-Constants.GAINS.SHOOTER.peakOutput, Constants.GAINS.SHOOTER.peakOutput);

    //makes the PID values accesible to elastic for testing
     SparkBaseSetter shooterSetter = new SparkBaseSetter(new SparkBaseSetter.SparkConfiguration(shooter, shooterConfig));
    shooterSetter.setPID(Constants.GAINS.SHOOTER);
    PIDDisplay.PIDList.addOption("Shooter", shooterSetter);

    //Indexer motor controller config
    indexerConfig = new SparkMaxConfig();
    indexerConfig
    .inverted(false)
    .idleMode(IdleMode.kBrake)
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

    aimPID.enableContinuousInput(-Math.PI, Math.PI);
    aimPID.setTolerance(Math.toRadians(1.5)); // stop jittering

  }

  public void setShooterVelocity(double rpm){
    shooterController.setSetpoint(rpm, ControlType.kVelocity);
  }
  
  public void setIndexVelocity(double rpm){
    indexerController.setSetpoint(rpm, ControlType.kVelocity);
  }
  
  public Command shootCommand() {
      return new StartEndCommand(
        () -> setShooterVelocity(3250),
        () -> setShooterVelocity(Constants.SHOOTER.NOMINAL_RPM)
        );
  }

  

  //continuosly calculates and sets the needed RPM and angle to target (only direct to target right now) based on distance, will add caching if it runs bad
  public Command autoAimCommand(DoubleSupplier xSpeed, DoubleSupplier ySpeed) {
    return run(() -> {
      setShooterVelocity(shooterSolver.getTargetRPM()); 

      double omega = 
        aimPID.calculate(
          drivetrain.getPose().getRotation().getRadians(),
          shooterSolver.getAngleToHub().getRadians()
        );

        drivetrain.drive(new ChassisSpeeds(xSpeed.getAsDouble(), ySpeed.getAsDouble(), omega), true);
      
    } 
      ).finallyDo(interrupted -> {
        setShooterVelocity(Constants.SHOOTER.NOMINAL_RPM);
      }
    );
    }
      
  public Command manualIndexCommand() {
    return new StartEndCommand(
      () -> setIndexVelocity(40),
      () -> setIndexVelocity(0)
      );
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
