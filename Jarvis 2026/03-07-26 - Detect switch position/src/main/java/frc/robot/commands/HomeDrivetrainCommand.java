package frc.robot.commands;

import java.util.Arrays;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveModule;

/**
 * A command to home the drivetrain by spinning the modules until the magnetic sensors are triggered, then setting the module angles to 0
 */
public class HomeDrivetrainCommand extends Command {
    private final SwerveSubsystem swerveSubsystem;

    public HomeDrivetrainCommand(SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;

        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {
        // Start spinning the modules counter-clockwise slowly
        for (SwerveModule swerveModule : swerveSubsystem.getModules()) {
            swerveModule.getAngleMotor().set(0.1);
        }
    }

    @Override
    public void execute() {
        SwerveModule[] modules = swerveSubsystem.getModules();
        DigitalInput[] moduleMagSensors = swerveSubsystem.getModuleMagSensors();
        // Loop over the modules and their corresponding magnetic sensors
        for(int i = 0; i < modules.length; i++) {
            // For each module, check if the magnetic sensor is triggered. If it is, stop the module and set its angle to 0
            if (moduleMagSensors[i].get()) {
                modules[i].getAngleMotor().set(0);
                modules[i].setAngle(0);
            }
        }
    }

    @Override
    public boolean isFinished() {
        // The command is finished when all the magnetic sensors are triggered, meaning all the modules are homed
        return Arrays.stream(swerveSubsystem.getModuleMagSensors()).allMatch(DigitalInput::get);
    }
}
