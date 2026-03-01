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
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ModuleConstants;

import java.io.File;
import java.util.function.Consumer;
import java.util.function.Supplier;

import com.ctre.phoenix6.signals.InvertedValue;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;

import swervelib.parser.SwerveParser;
import swervelib.SwerveDrive;
import swervelib.SwerveInputStream;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveSubsystem extends SubsystemBase {  
  private SparkClosedLoopController turnPIDController; // Added to set home angle
  private double moduleOffset;
  
  /** Creates a new swerve subsystem. */
  double maxSpeed = Units.feetToMeters(4.5);
  File directory = new File(Filesystem.getDeployDirectory(),"swerve");
  SwerveDrive   swerveDrive;

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
  }

  enum MODULES {
    FRONT_LEFT(5),
    FRONT_RIGHT(6),
    BACK_LEFT(7),
    BACK_RIGHT(8);
  }
/*
    private MODULES(int moduleID) {
      this.base = new SwerveSubsystem(
        moduleID + 5, // Turn motor ID
        ModuleConstants.MODULE_OFFSETS[moduleID]
      );
    }
    public static void forAll(Consumer<SwerveSubsystem> lambda) {
      for (MODULES modules : MODULES.values()) {
        lambda.accept(modules.base);
      }
    }
  } */

  public Command homeCommand() {
    return new SequentialCommandGroup(
      new ParallelRaceGroup(
        new RunCommand(() -> MODULES.forAll(SwerveSubsystem::home), this)
      )
    );
  }

  public void home() {
    turnPIDController.setSetpoint(moduleOffset, SparkMax.ControlType.kPosition);
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
}
