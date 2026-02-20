package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

import edu.wpi.first.wpilibj.Joystick;

import edu.wpi.first.wpilibj2.command.InstantCommand;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Commands.JoystickDrive;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;

public class ControlPanel {
    private static final Joystick controller = new Joystick(0);
    
    private static final NetworkTable nTable = NetworkTableInstance.getDefault().getTable("SmartDashboard/Control Panel");

    private static Drivetrain drivetrain;


    public static void configureBinding(Drivetrain drivetrain, Shooter shooter, Intake intake) {
        drivetrain.setDefaultCommand(new JoystickDrive(drivetrain, controller));
        ControlPanel.drivetrain = drivetrain;
        
        
        //new JoystickButton(controller, 1).whileTrue(intake);

        new JoystickButton(controller, 7).whileTrue(drivetrain.homeCommand()); //Home Command for Swerve
        new JoystickButton(controller, 8).onTrue(new InstantCommand(() -> drivetrain.resetIMU())); //Gyro Reset for Swerve

        }




   
    }
    


