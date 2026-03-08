package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Subsystems.ChassisVisionLocalizer;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;
import frc.robot.Util.PIDDisplay;
import frc.robot.Util.ShooterSolver;

public class RobotContainer {
  private static final Drivetrain drivetrain = new Drivetrain();
  private static final ShooterSolver shootersolver = new ShooterSolver(drivetrain);
  private static final Shooter shooter = new Shooter(shootersolver, drivetrain);
  private static final Intake intake = new Intake();
  //private static final Limelight limelight = new Limelight();

  SendableChooser<Command> autoChooser;

  private static boolean isBlue = false;

  public RobotContainer() {
    new PIDDisplay();
    new ChassisVisionLocalizer();

    ControlPanel.configureBinding(drivetrain, shooter, intake);
    configureAuto();

    PIDDisplay.Init();
    
    Ultrasonic.setAutomaticMode(true);
  }

  public static boolean isBlue() {
    return isBlue;
  }

  private void configureAuto() {
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  public Command getAutonomousCommand() {
    return new SequentialCommandGroup(
      drivetrain.homeCommand(),
      AutoBuilder.buildAuto(autoChooser.getSelected().getName())
    );
  }

  public void onTeleopEnabled() {
   
    }
}
