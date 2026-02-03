package frc.robot.Commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Subsystems.Drivetrain;

public class JoystickDrive extends Command {
  Drivetrain drivetrain;
  Joystick controller;

  public JoystickDrive(Drivetrain drivetrain, Joystick controller) {
    this.drivetrain = drivetrain;
    this.controller = controller;
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    double xInputRight = -applyDeadband(controller.getRawAxis(4), Constants.DrivetrainConstants.DRIVE_TOLERANCE_PERCENT);
    double yInputRight = -applyDeadband(controller.getRawAxis(5), Constants.DrivetrainConstants.DRIVE_TOLERANCE_PERCENT);

    xInputRight = Math.abs(xInputRight) * xInputRight;
    yInputRight = Math.abs(yInputRight) * yInputRight;

    double multiplier = (Drivetrain.alignMode ? Constants.DrivetrainConstants.ALIGN_CONTROL_MULTIPLIER : 1) * Constants.DrivetrainConstants.MAX_DRIVE_SPEED * (RobotContainer.isBlue() ? 1 : -1);
    double xSpeed = yInputRight * multiplier;
    double ySpeed = xInputRight * multiplier;

    double xInputLeft = applyDeadband(controller.getRawAxis(0), Constants.DrivetrainConstants.DRIVE_TOLERANCE_PERCENT);
    double rotationSpeed = -xInputLeft * Math.abs(xInputLeft) * Constants.DrivetrainConstants.MAX_ANGULAR_SPEED * (Drivetrain.alignMode ? Constants.DrivetrainConstants.ALIGN_CONTROL_MULTIPLIER : 1);

    drivetrain.drive(new ChassisSpeeds(xSpeed, ySpeed, rotationSpeed), true);
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(new ChassisSpeeds(), false);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  private double applyDeadband(double joystickValue, double tolerance){
    if(Math.abs(joystickValue) > tolerance){
      return Math.signum(joystickValue) * (Math.abs(joystickValue) - tolerance)/(1-tolerance);
    } 
    return 0;
  }
}
