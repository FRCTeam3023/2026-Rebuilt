// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meter;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.SparkBaseSetter;
import frc.robot.util.SparkBaseSetter.SparkConfiguration;
import frc.robot.util.TalonFXSetter;

import java.io.File;
import java.util.function.Supplier;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.RelativeEncoder;

import swervelib.parser.SwerveParser;
import swervelib.SwerveDrive;
import swervelib.SwerveInputStream;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

// Imports for Jarvis 2025 code portion
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.Constants.ModuleConstants;

public class SwerveSubsystem extends SubsystemBase {  
  double maxSpeed = Units.feetToMeters(4.5);
  File directory = new File(Filesystem.getDeployDirectory(),"swerve");
  SwerveDrive   swerveDrive;

  private final static Pigeon2 IMU = new Pigeon2(Constants.CAN_ID.IMU_ID); // TO-DO: Scan swervedrive.json for IMU ID
  // private SparkClosedLoopController turnPIDController;

  private TalonFX driveMotor; // Start of Jarvis 2025 code portion
  private SparkMax turnMotor;
  private SparkMaxConfig turnConfig;
  private SparkClosedLoopController turnPIDController;
  private RelativeEncoder turnEncoder;
  private double moduleOffset;
  private DigitalInput hallEffectSensor;
  public int moduleID;

  public boolean homed = false;
  private boolean wasHomed;

  private VelocityVoltage desiredVelocity = new VelocityVoltage(0);
  private TalonFXConfiguration driveConfiguration;

  public static final TalonFXSetter driveSetters = new TalonFXSetter();
  public static final SparkBaseSetter turnSetters = new SparkBaseSetter();

  private final NetworkTable nTable = NetworkTableInstance.getDefault().getTable("SmartDashboard/Drivetrain/Swerve/Modules");
  private final GenericEntry homedEntry, switchEntry; // End of Jarvis 2025 code portion

    public SwerveSubsystem(int moduleID, int driveID, int turnID, double moduleOffset, InvertedValue inverted, int sensorID) {
      SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
  
      try
      {
        swerveDrive = new SwerveParser(directory).createSwerveDrive(Constants.maxSpeed, new Pose2d(new Translation2d(Meter.of(1),
                                                                                                                Meter.of(4)),
                                                                                                        Rotation2d.fromDegrees(0)));
        // Alternative method if you don't want to supply the conversion factor via JSON files.
        // swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed, angleConversionFactor, driveConversionFactor);
      } catch (Exception e)
      {
        throw new RuntimeException(e);
      }
      
      this.moduleOffset = moduleOffset; // Start of Jarvis 2025 code portion
      this.moduleID = moduleID;

      hallEffectSensor = new DigitalInput(sensorID);

      // Drive Motor
      driveMotor = new TalonFX(driveID);

      driveConfiguration = new TalonFXConfiguration();
      driveConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
      driveConfiguration.MotorOutput.Inverted = inverted;
      driveConfiguration.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.25;

      driveConfiguration.Voltage.PeakForwardVoltage = Constants.GAINS.DRIVE.peakOutput;
      driveConfiguration.Voltage.PeakReverseVoltage = -Constants.GAINS.DRIVE.peakOutput;

      driveConfiguration.Feedback.SensorToMechanismRatio = ModuleConstants.DRIVE_GEARING / (ModuleConstants.WHEEL_DIA * Math.PI);
      driveConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

      driveMotor.getConfigurator().apply(driveConfiguration);
      driveSetters.addConfigurator(driveMotor.getConfigurator());
      desiredVelocity.UpdateFreqHz = 100;
      
      // Turn Motor
      turnMotor = new SparkMax(turnID, MotorType.kBrushless);
      turnPIDController = turnMotor.getClosedLoopController();
      turnConfig = new SparkMaxConfig();
      turnConfig
          .inverted(true)
          .idleMode(IdleMode.kBrake)
          .voltageCompensation(12);
      turnConfig.encoder
          .positionConversionFactor(2 * Math.PI / ModuleConstants.TURN_GEARING)
          .velocityConversionFactor(2 * Math.PI / ModuleConstants.TURN_GEARING);
      turnConfig.closedLoop
          .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
          .positionWrappingEnabled(true)
          .positionWrappingMaxInput(Math.PI)
          .positionWrappingMinInput(-Math.PI)
          .outputRange(-Constants.GAINS.TURN.peakOutput, Constants.GAINS.TURN.peakOutput);

      turnSetters.addConfigurator(new SparkConfiguration(turnMotor, turnConfig));
      turnEncoder = turnMotor.getEncoder();

      homedEntry = nTable.getTopic("Homed [" + moduleID + "]").getGenericEntry();
      switchEntry = nTable.getTopic("Switch [" + moduleID + "]").getGenericEntry(); // End of Jarvis 2025 code portion
    }
    
    public void home(){
        boolean switchState = getSwitch();
        //only triggers on rising edge of switch, set new home state and target location
        if (switchState != wasHomed && switchState) {
            turnEncoder.setPosition(moduleOffset);
            homed = true;
        }

        //if it is triggered, set to target; else keep rotating
        if (homed)
            turnPIDController.setSetpoint(moduleOffset, SparkMax.ControlType.kPosition);
        else
            turnMotor.set(0.25);

        wasHomed = switchState;
    }
  
    public boolean getSwitch(){
        return !hallEffectSensor.get();
    }

    public void stop(){
        driveMotor.set(0);
        turnMotor.set(0);
    }

    public void zeroEncoders() {
        driveMotor.setPosition(0);
        turnEncoder.setPosition(0);
    }

    @Override
    public void periodic() {
      // This method will be called once per scheduler run
    }
  
    @Override
    public void simulationPeriodic() {
      // This method will be called once per scheduler run during simulation
    }
  
    public SwerveDrive getSwerveDrive() {
      return swerveDrive;
    }
  
    public void driveFieldOriented(ChassisSpeeds velocity) {
      swerveDrive.driveFieldOriented(velocity);
    }
  
    public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity) {
      return run(() -> {
        swerveDrive.driveFieldOriented(velocity.get());;
      });
    }
  
    public void zeroGyro() {
      swerveDrive.zeroGyro();
    }
  
    public static void resetIMU() {
      IMU.reset();
  }

    public static void resetIMUCommand() {
      Pigeon2Configuration IMUconfig = new Pigeon2Configuration();
      IMU.getConfigurator().apply(IMUconfig);
      resetIMU();
  }

    public Command homeCommand() {
      return new SequentialCommandGroup(
        new RunCommand(() -> turnEncoder.setPosition(0.25))
      );
  }
}