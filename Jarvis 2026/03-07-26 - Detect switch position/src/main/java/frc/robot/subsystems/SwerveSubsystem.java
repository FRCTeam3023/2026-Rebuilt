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
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import java.io.File;
import java.util.ArrayList;
import java.util.function.Supplier;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.core.CorePigeon2;

import swervelib.parser.SwerveParser;
import swervelib.SwerveDrive;
import swervelib.SwerveInputStream;
import swervelib.SwerveModule;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveSubsystem extends SubsystemBase {  
  double maxSpeed = Units.feetToMeters(4.5);
  File directory = new File(Filesystem.getDeployDirectory(),"swerve");
  SwerveDrive   swerveDrive;

  private final static Pigeon2 IMU = new Pigeon2(Constants.CAN_ID.IMU_ID); // TO-DO: Scan swervedrive.json for IMU ID
  
  // Array of the magnetic sensors. The order must match the order of the swerve modules returned by getModules().
  private final DigitalInput[] moduleMagSensors = new DigitalInput[]  {
      new DigitalInput(0),
      new DigitalInput(1),
      new DigitalInput(2),
      new DigitalInput(3)};
  
  private NetworkTableInstance inst;
  private NetworkTable swerveTable;
  private DoublePublisher xPub;
  private double xValue = 0.0;

  private NetworkTableInstance switchInst;
  private NetworkTable switchTable;
  private BooleanPublisher switchPub;
  private boolean switchValue;

    public SwerveSubsystem() {
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
      
      inst = NetworkTableInstance.getDefault();
      swerveTable = inst.getTable("Swerve datatable");
      xPub = swerveTable.getDoubleTopic("x").publish();

      switchInst = NetworkTableInstance.getDefault();
      switchTable = inst.getTable("Switch datatable");
      switchPub = switchTable.getBooleanTopic("Switch status").publish();
    }
  
    /**
     * Example command factory method.
     *
     * @return a command
     */
    public Command exampleMethodCommand() {
      // Inline construction of command goes here.
      // Subsystem::RunOnce implicitly requires `this` subsystem.
      return runOnce(
          () -> {
            /* one-time action goes here */
          });
    }
  
    /**
     * An example method querying a boolean state of the subsystem (for example, a digital sensor).
     *
     * @return value of some boolean subsystem state, such as a digital sensor.
     */
    public boolean exampleCondition() {
      // Query some boolean state, such as a digital sensor.
      return false;
    }
  
    @Override
    public void periodic() {
      xValue += 0.05;
      xPub.set(xValue);

      switchPub.set(switchID0());
      switchPub.set(switchID1());
      switchPub.set(switchID2());
      switchPub.set(switchID3());
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

    public void resetDriveEncoders() {
      swerveDrive.resetDriveEncoders();
    }

    public void useInternalFeedbackSensor() {
      swerveDrive.useInternalFeedbackSensor();
  }

  /**
   * Gets the swerve modules from the swerve drive.
   * 
   * @return an array of the swerve modules
   */
  public SwerveModule[] getModules() {
    return swerveDrive.getModules();
  }

  /**
   * Gets the magnetic sensors for the swerve modules. These are used for homing the modules to a known position.
   * 
   * @return an array of the magnetic sensors. The order will match the order of the swerve modules returned by getModules().
   */
  public DigitalInput[] getModuleMagSensors() {
    return moduleMagSensors;
  }

  public DigitalInput moduleMagSensors0 = moduleMagSensors[0];
  public DigitalInput moduleMagSensors1 = moduleMagSensors[1];
  public DigitalInput moduleMagSensors2 = moduleMagSensors[2];
  public DigitalInput moduleMagSensors3 = moduleMagSensors[3];

  public boolean switchID0() {
    return !moduleMagSensors0.get();
  }

  public boolean switchID1() {
    return !moduleMagSensors1.get();
  }

  public boolean switchID2() {
    return !moduleMagSensors2.get();
  }

  public boolean switchID3() {
    return !moduleMagSensors3.get();
  }
}