// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Subsystems.Shooter;
import frc.robot.Util.ShooterSolver;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ShootSequence extends Command {
  Shooter shooter;
  ShooterSolver shooterSolver;
  
  /** Creates a new ShootSequence. */
  public ShootSequence() {
   
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //set shooter motor to solved rpm
    shooter.setShooterVelocity(Math.abs(shooterSolver.getTargetRPM()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //set motor RPM nominal
    shooter.setShooterVelocity(Constants.SHOOTER.NOMINAL_RPM);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
