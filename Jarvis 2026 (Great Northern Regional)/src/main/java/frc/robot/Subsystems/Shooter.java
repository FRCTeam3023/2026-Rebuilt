// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Util.ShooterSolver;

public class Shooter extends SubsystemBase {

  private NetworkTable shooterTable = NetworkTableInstance.getDefault().getTable("Elastic/Shooter");
  private NetworkTableInstance inst;

  private final GenericEntry velocityEntry = shooterTable.getTopic("Shooter Velocity").getGenericEntry();

  private ShooterSolver shooterSolver;
  private Drivetrain drivetrain;

  TalonFX shooterMotor1;
  TalonFX shooterMotor2;

  private TalonFXConfiguration shooterConfiguration;

  private final PIDController aimPID = new PIDController(4, 0, 0);

  // Network tables
  private boolean shooterMotors = false;
  private BooleanPublisher shooterMotorsPub;
  private boolean indexerEnabled = false;
  private BooleanPublisher indexerEnabledPub;

  public Shooter(ShooterSolver shooterSolver, Drivetrain drivetrain) {

    this.shooterSolver = shooterSolver;
    this.drivetrain = drivetrain;

    // Shooter Motor 1
    shooterMotor1 = new TalonFX(Constants.CAN_DEVICES.SHOOTER_MOTOR.id);
    shooterMotor2 = new TalonFX(Constants.CAN_DEVICES.SHOOTER_MOTOR_2.id);

    shooterConfiguration = new TalonFXConfiguration();
    shooterConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    shooterConfiguration.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    shooterConfiguration.Voltage.PeakForwardVoltage = Constants.SHOOTER.TALON_PEAK_OUTPUT;
    shooterConfiguration.Voltage.PeakReverseVoltage = Constants.SHOOTER.TALON_PEAK_OUTPUT;

    shooterConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

    // Shooter Motors PID Values
    var slot0Configs = new Slot0Configs();
    slot0Configs.kS = 0.1;
    slot0Configs.kV = 0.01;
    slot0Configs.kP = .01;
    slot0Configs.kI = 0;
    slot0Configs.kD = 0;
  
    shooterMotor1.getConfigurator().apply(shooterConfiguration);
    shooterMotor1.getConfigurator().apply(slot0Configs);

    shooterMotor2.setControl(new Follower(shooterMotor1.getDeviceID(), MotorAlignmentValue.Opposed));



    //#region Old Shooter Motors
    // shooter = new SparkFlex(Constants.CAN_DEVICES.SHOOTER_MOTOR.id, MotorType.kBrushless);
    // shooter2 = new SparkFlex(Constants.CAN_DEVICES.SHOOTER_MOTOR_2.id, MotorType.kBrushless);
    

    // shooterController = shooter.getClosedLoopController();
    // shooterController2 = shooter2.getClosedLoopController();
    

    //Shooter motor controller config
    // shooterConfig = new SparkFlexConfig();
    // shooterConfig
    // .inverted(false)
    // .idleMode(IdleMode.kCoast)
    // .smartCurrentLimit(38)
    // .voltageCompensation(12);
    // shooterConfig.encoder
    // .positionConversionFactor(1.0/Constants.SHOOTER.GEAR_RATIO)
    // .velocityConversionFactor(1.0);
    // shooterConfig.closedLoop
    // .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    // .positionWrappingEnabled(false)
    // .outputRange(-1.0, 1.0);

    // //Shooter motor controller 2 config
    // shooterConfig2 = new SparkFlexConfig();
    // shooterConfig2
    // .inverted(true)
    // .idleMode(IdleMode.kCoast)
    // .smartCurrentLimit(38)
    // .voltageCompensation(12);
    // shooterConfig2.encoder
    // .positionConversionFactor(1.0/Constants.SHOOTER.GEAR_RATIO)
    // .velocityConversionFactor(1.0);
    // shooterConfig2.closedLoop
    // .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
    // .positionWrappingEnabled(false)
    // .outputRange(-1.0, 1.0);
    //#endregion

   

    aimPID.enableContinuousInput(-Math.PI, Math.PI);
    aimPID.setTolerance(Math.toRadians(1.5)); // stop jittering
    
    // Network tables
    inst = NetworkTableInstance.getDefault();
    shooterTable = inst.getTable("Elastic/Shooter");
    shooterMotorsPub = shooterTable.getBooleanTopic("Shooter On").publish();
    indexerEnabledPub = shooterTable.getBooleanTopic("Indexer On").publish();
  }

    public boolean ShooterMotors() {
    return shooterMotors;
  }

  public boolean IndexerEnabled() {
    return indexerEnabled;
  }

  public void setShooterVelocity(double rpm){
    double rps = rpm/60; //Converts from RPM to RPS
    final VelocityDutyCycle m_request = new VelocityDutyCycle(0);
    shooterMotor1.setControl(m_request.withVelocity(rps).withSlot(0));
  }

  public void stopShooterCoast(){
    shooterMotor1.set(0);
  }
  
  // Basic set to rpm shoot command
  public Command shootCommand(double rpm) {
    return new StartEndCommand(
    () -> { 
      setShooterVelocity(rpm);
      shooterMotors = true; 
        },
    () -> {
      stopShooterCoast();
      shooterMotors = false; 
        }
    );
  }
  
  //continuosly calculates and sets the needed RPM and angle to target (only direct to target right now) based on distance, will add caching if it runs bad
  public Command autoAimCommand(DoubleSupplier xSpeed, DoubleSupplier ySpeed) {
    return runOnce(() -> aimPID.reset())
    .andThen(
    run(() -> {
      setShooterVelocity(shooterSolver.getTargetRPM()); 

      double omega = 
        aimPID.calculate(
          drivetrain.getPose().getRotation().getRadians(),
          shooterSolver.getAngleToHub().getRadians()
        );
      
      omega = MathUtil.clamp(omega, -4, 4);

        if (aimPID.atSetpoint()) {
        omega = 0;
        }

        drivetrain.drive(new ChassisSpeeds(xSpeed.getAsDouble(), ySpeed.getAsDouble(), omega), true);
      
    } 
      ).finallyDo(interrupted -> {
        stopShooterCoast();

          }
        )
      );
    }
      

  @Override
  public void periodic() {
    velocityEntry.setDouble(shooterMotor1.getRotorVelocity().getValueAsDouble()*60);

    shooterMotorsPub.set(ShooterMotors());
    indexerEnabledPub.set(IndexerEnabled());
  }
}
