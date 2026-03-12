package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.JoystickDrive;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Indexer;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;

public class ControlPanel {
    private static final Joystick controller = new Joystick(0);
    
    private static final NetworkTable nTable = NetworkTableInstance.getDefault().getTable("Elastic/Control Panel");

    private static Drivetrain drivetrain;

 

    public static void configureBinding(Drivetrain drivetrain, Shooter shooter, Intake intake, Indexer indexer) {
        drivetrain.setDefaultCommand(new JoystickDrive(drivetrain, controller));
        ControlPanel.drivetrain = drivetrain;


        /*
          Right Trigger - Feed/Shoot Command
          Left Trigger - Run Intake
          Right Bumper - Auto Distance Aim Command
          Start Button - Home Robot
          Back Button - Reset Gyro
        */
        
        /*
          Shoot Targeting (Distance math + auto align to target)
          Intake Cycle (Run agitator with intake to allow more intake)
          Shoot Command (Run Indexer while cycling intake)
         */
        
        
        // All Binds Follow  BOUND_BUTTON | COMMAND/ACTION

        //new Trigger(() -> controller.getRawAxis(3) > 0.7).whileTrue(shooter.shootCommand()); // RIGHT TRIGGER | Dumb Shoot Command
         
        //new Trigger(() -> controller.getRawAxis(2) > 0.5).whileTrue(intake.intakeCycleCommand()); // LEFT TRIGGER | Intake Down and Run

        // RIGHT BUMPER | AUTO AIM (Changes RPM based on distance to target) [Add Auto Align]
        // new JoystickButton(controller, 6).whileTrue(shooter.autoAimCommand( 
        // () -> -controller.getRawAxis(1),
        // () -> -controller.getRawAxis(0)
        // ));

        //new Trigger(() -> controller.getRawAxis(3) > 0.5).whileTrue(indexer.runIndexCommand(200)); // Right Trigger | Indexes into the shooter (SPIN UP SHOOTER FIRST)

        //new JoystickButton(controller, 4).onTrue(drivetrain.hubPoseResetCommand()); // Y | Doesnt Work

        //new Trigger(() -> controller.getRawAxis(2) > 0.5).whileTrue(intake.runIntakeCommand(3500)); // Intake Manual - No Pivot | Left Bumper
        //new JoystickButton(controller, 5).whileTrue(intake.runIntakeCommand(3500));

        new JoystickButton(controller, 7).whileTrue(drivetrain.homeCommand()); // BACK | Home Command for Swerve
        new JoystickButton(controller, 8).onTrue(new InstantCommand(() -> drivetrain.resetIMU())); // START | Gyro Reset for Swerve

        // new JoystickButton(controller, 1).whileTrue(intake.moveIntakeTest()); // A
        }




   
    }
    


