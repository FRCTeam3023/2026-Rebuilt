package frc.robot;

import com.pathplanner.lib.commands.PathfindingCommand;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.LimelightHelpers;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    CameraServer.startAutomaticCapture();
    /*WebServer.start(5800, Filesystem.getDeployDirectory().getPath());
    ^^^ REMOVING TEMPORARILY TO CONFIGURE LIMELIGHT PORTS*/
    PathfindingCommand.warmupCommand().schedule();

    // (robotIP):5801 will now point to a Limelight3A's (id 0) web interface stream:
    // (robotIP):5800 will now point to a Limelight3A's (id 0) video stream:
    PortForwarder.add(5800, "172.29.0.1", 5800);
    PortForwarder.add(5801, "172.29.0.1", 5801);
    PortForwarder.add(5802, "172.29.0.1", 5802);
    PortForwarder.add(5803, "172.29.0.1", 5803);
    PortForwarder.add(5804, "172.29.0.1", 5804);
    PortForwarder.add(5805, "172.29.0.1", 5805);
    PortForwarder.add(5806, "172.29.0.1", 5806);
    PortForwarder.add(5807, "172.29.0.1", 5807);
    PortForwarder.add(5808, "172.29.0.1", 5808);
    PortForwarder.add(5809, "172.29.0.1", 5809);
    
    // (robotIP):5811 will now point to a Limelight3A's (id 1) web interface stream:
    // (robotIP):5810 will now point to a Limelight3A's (id 1) video stream:
    PortForwarder.add(5811, "172.29.1.1", 5801);
    PortForwarder.add(5812, "172.29.1.1", 5802);
    PortForwarder.add(5813, "172.29.1.1", 5803);
    PortForwarder.add(5814, "172.29.1.1", 5804);
    PortForwarder.add(5815, "172.29.1.1", 5805);
    PortForwarder.add(5816, "172.29.1.1", 5806);
    PortForwarder.add(5817, "172.29.1.1", 5807);
    PortForwarder.add(5818, "172.29.1.1", 5808);
    PortForwarder.add(5819, "172.29.1.1", 5809);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    m_robotContainer.onTeleopEnabled();
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
