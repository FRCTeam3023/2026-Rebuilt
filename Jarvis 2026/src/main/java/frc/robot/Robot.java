package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot
{

  private static Robot   instance;
  private        Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  private Timer disabledTimer;

  public Robot()
  {
    instance = this;
  }

  public static Robot getInstance()
  {
    return instance;
  }

  @Override
  public void robotInit()
  {
    m_robotContainer = new RobotContainer();

    // Port forwarding for Limelights
    PortForwarder.add(5801, "172.29.0.1", 5801);
    PortForwarder.add(5802, "172.29.0.1", 5802);
    PortForwarder.add(5803, "172.29.0.1", 5803);
    PortForwarder.add(5804, "172.29.0.1", 5804);
    PortForwarder.add(5805, "172.29.0.1", 5805);
    PortForwarder.add(5806, "172.29.0.1", 5806);
    PortForwarder.add(5807, "172.29.0.1", 5807);
    PortForwarder.add(5808, "172.29.0.1", 5808);
    PortForwarder.add(5809, "172.29.0.1", 5809);

    PortForwarder.add(5811, "172.29.1.1", 5801);
    PortForwarder.add(5812, "172.29.1.1", 5802);
    PortForwarder.add(5813, "172.29.1.1", 5803);
    PortForwarder.add(5814, "172.29.1.1", 5804);
    PortForwarder.add(5815, "172.29.1.1", 5805);
    PortForwarder.add(5816, "172.29.1.1", 5806);
    PortForwarder.add(5817, "172.29.1.1", 5807);
    PortForwarder.add(5818, "172.29.1.1", 5808);
    PortForwarder.add(5819, "172.29.1.1", 5809);

    disabledTimer = new Timer();

    if (isSimulation())
    {
      DriverStation.silenceJoystickConnectionWarning(true);
    }
  }

 @Override
  public void robotPeriodic()
  {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit()
  {
    m_robotContainer.setMotorBrake(true);
    disabledTimer.reset();
    disabledTimer.start();
  }
  
  @Override
  public void disabledPeriodic()
  {
    if (disabledTimer.hasElapsed(Constants.DrivebaseConstants.WHEEL_LOCK_TIME))
    {
      m_robotContainer.setMotorBrake(false);
      disabledTimer.stop();
      disabledTimer.reset();
    }
  }

  @Override
  public void autonomousInit()
  {
    m_robotContainer.setMotorBrake(true);
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    System.out.println("Auto selected: " + m_autonomousCommand);

    if (m_autonomousCommand != null)
    {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  @Override
  public void autonomousPeriodic()
  {
  }

  @Override
  public void teleopInit()
  {
    if (m_autonomousCommand != null)
    {
      m_autonomousCommand.cancel();
    } else
    {
      CommandScheduler.getInstance().cancelAll();
    }
  }

  @Override
  public void teleopPeriodic()
  {
  }

  @Override
  public void testInit()
  {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic()
  {
  }

  @Override
  public void simulationInit()
  {
  }

  @Override
  public void simulationPeriodic()
  {
  }
}
