package frc.robot.Subsystems;

import java.lang.reflect.Array;
import java.util.Arrays;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Function;
import java.util.stream.Collectors;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.signals.InvertedValue;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.SerialPort.StopBits;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Commands.Notifications;

public class Drivetrain extends SubsystemBase {
    private final Pigeon2 IMU = new Pigeon2(Constants.CAN_DEVICES.PIGEON_2.id);
    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(Constants.ModuleConstants.MODULE_POSITIONS);
    private static SwerveDrivePoseEstimator poseEstimator;
    private final NetworkTable nTable = NetworkTableInstance.getDefault().getTable("SmartDashboard/Drivetrain");

    private final GenericEntry headingEntry = nTable.getTopic("Heading").getGenericEntry();

    public static Field2d field = new Field2d();

    public static boolean alignMode = false;
    private PathPlannerPath currentLineupPath = null;

    /* public Drivetrain() {
        poseEstimator = new SwerveDrivePoseEstimator(
            kinematics, 
            new Rotation2d(), 
            MODULES.collectProperty(SwerveModule::getPosition, SwerveModulePosition.class),
            new Pose2d(0,0, new Rotation2d()),
            MatBuilder.fill(Nat.N3(), Nat.N1(),0.05,0.05,0.05), //Standard deviations for state estimate, (m,m,rad). Increase to trust less
            MatBuilder.fill(Nat.N3(), Nat.N1(), 
                Constants.PhotonConstants.STANDARD_DEVIATION,
                Constants.PhotonConstants.STANDARD_DEVIATION,
                Constants.PhotonConstants.STANDARD_DEVIATION
            )
        );
    } */
    
    enum MODULES {
    FRONT_LEFT(0),
    FRONT_RIGHT(1),
    BACK_LEFT(2),
    BACK_RIGHT(3);

    public SwerveModule base;
    private MODULES(int moduleID) {
      this.base = new SwerveModule(
        moduleID, 
        moduleID + 1, 
        moduleID + 5, 
        ModuleConstants.MODULE_OFFSETS[moduleID], 
        moduleID % 2 == 0 ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive,
        moduleID
      );
    }
//#region Don't touch
    /**
     * Iterates through the list of swerve modules and returns an array containing the result of executing a function on each module
     * @param <T> Property type
     * @param map The function to run on each swerve module
     * @return The resulting list
     */
    @SuppressWarnings("unchecked")
    public static <T> T[] collectProperty(Function<SwerveModule, T> map, Class<T> returnType) {
      List<T> result = List.of(MODULES.values()).stream().map(m -> map.apply(m.base)).collect(Collectors.toList());
      return result.toArray((T[]) Array.newInstance(returnType, result.size()));
    }

    public static void forAll(Consumer<SwerveModule> lambda) {
      for (MODULES modules : MODULES.values()) {
        lambda.accept(modules.base);
      }
    }
//#endregion
  }

  public static void setModuleStates(SwerveModuleState[] moduleStates){
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, ModuleConstants.MAX_SPEED);
    MODULES.forAll(m -> m.setDesiredState(moduleStates[m.moduleID]));
  }

  public void drive(ChassisSpeeds speeds, boolean isFieldRelative){
    if(isFieldRelative)
      speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getPose().getRotation());

    setModuleStates(kinematics.toSwerveModuleStates(speeds));
  }

  public Pose2d getPose(){
    return poseEstimator.getEstimatedPosition();
  }

  public void resetIMU() {
    IMU.reset();
  }

  public Command homeCommand() {
    return new SequentialCommandGroup(
      new InstantCommand(() -> MODULES.forAll(m -> m.setHomed(false))),
      new ParallelRaceGroup(
        new RunCommand(() -> MODULES.forAll(SwerveModule::home), this),
        new WaitUntilCommand(() -> !Arrays.asList(MODULES.collectProperty(m -> m.homed, Boolean.class)).contains(false))
          .andThen(Notifications.SWERVE_HOME_SUCCESS.send()),
        new WaitCommand(3)
          .andThen(Notifications.SWERVE_HOME_FAIL.send())
      )
    );
  }
  
}