package frc.robot;

import edu.wpi.first.wpilibj.Ultrasonic;
import frc.robot.Subsystems.ChassisVisionLocalizer;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Util.PIDDisplay;

public class RobotContainer {
  private static final Drivetrain drivetrain = new Drivetrain();
  //SendableChooser<Command> autoChooser;

  private static boolean isBlue = false;

  public RobotContainer() {
    new PIDDisplay();
    new ChassisVisionLocalizer();

    ControlPanel.configureBinding(drivetrain, null);
    configureAuto();

    PIDDisplay.Init();
    
    Ultrasonic.setAutomaticMode(true);
  }

  public static boolean isBlue() {
    return isBlue;
  }

  private void configureAuto() {
    //autoChooser = AutoBuilder.buildAutoChooser();
   // SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  // public Command getAutonomousCommand() {
  //   return new SequentialCommandGroup(
  //     drivetrain.homeCommand(),
  //     AutoBuilder.buildAuto(autoChooser.getSelected().getName())
  //   );
  // }

  public void onTeleopEnabled() {
    // TODO Auto-generated method stub
    //throw new UnsupportedOperationException("Unimplemented method 'onTeleopEnabled'");
    }
}
