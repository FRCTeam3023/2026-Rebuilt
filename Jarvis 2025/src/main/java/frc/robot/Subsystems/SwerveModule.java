// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Util.SparkBaseSetter;
import frc.robot.Util.SparkBaseSetter.SparkConfiguration;
import frc.robot.Util.TalonFXSetter;

public class SwerveModule extends SubsystemBase {
    private TalonFX driveMotor;
    private SparkMax turnMotor;
    private SparkMaxConfig turnConfig;
    private SparkClosedLoopController turnPIDController;
    private RelativeEncoder turnEncoder;
    private double moduleOffset;
    private DigitalInput hallEffectSensor;
    public int moduleID;

    public boolean homed = false;
    private boolean wasHomed;

    private VelocityVoltage desiredVelocity = new VelocityVoltage(0);
    private TalonFXConfiguration driveConfiguration;

    public static final TalonFXSetter driveSetters = new TalonFXSetter();
    public static final SparkBaseSetter turnSetters = new SparkBaseSetter();

    private final NetworkTable nTable = NetworkTableInstance.getDefault().getTable("SmartDashboard/Drivetrain/Swerve/Modules");
    private final GenericEntry homedEntry, switchEntry;

    /**
     * A single swerve module object
     * @param moduleID Module number
     * @param driveID CAN ID number for the drive Falcon500 
     * @param turnID CAN ID number for the turning SparkMax
     * @param moduleOffset radian offset for the zero position in relation to the hall effect sensor
     * @param inverted set true if the drive motor should be inverted. (Preference depending on set module offset)
     * @param sensorID DIO pin number of the hall effect sensor
     */

    public SwerveModule(int moduleID, int driveID, int turnID, double moduleOffset, InvertedValue inverted, int sensorID){
        this.moduleOffset = moduleOffset;
        this.moduleID = moduleID;

        hallEffectSensor = new DigitalInput(sensorID);

        //#region Drive Motor
        driveMotor = new TalonFX(driveID);

        driveConfiguration = new TalonFXConfiguration();
        driveConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        driveConfiguration.MotorOutput.Inverted = inverted;
        driveConfiguration.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.25;

        driveConfiguration.Voltage.PeakForwardVoltage = Constants.GAINS.DRIVE.peakOutput;
        driveConfiguration.Voltage.PeakReverseVoltage = -Constants.GAINS.DRIVE.peakOutput;

        driveConfiguration.Feedback.SensorToMechanismRatio = ModuleConstants.DRIVE_GEARING / (ModuleConstants.WHEEL_DIA * Math.PI);
        driveConfiguration.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;

        driveMotor.getConfigurator().apply(driveConfiguration);
        driveSetters.addConfigurator(driveMotor.getConfigurator());
        desiredVelocity.UpdateFreqHz = 100;
        //#endregion

        //#region Turn Motor
        turnMotor = new SparkMax(turnID, MotorType.kBrushless);
        turnPIDController = turnMotor.getClosedLoopController();
        turnConfig = new SparkMaxConfig();
        turnConfig
            .inverted(true)
            .idleMode(IdleMode.kBrake)
            .voltageCompensation(12);
        turnConfig.encoder
            .positionConversionFactor(2 * Math.PI / ModuleConstants.TURN_GEARING)
            .velocityConversionFactor(2 * Math.PI / ModuleConstants.TURN_GEARING);
        turnConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .positionWrappingEnabled(true)
            .positionWrappingMaxInput(Math.PI)
            .positionWrappingMinInput(-Math.PI)
            .outputRange(-Constants.GAINS.TURN.peakOutput, Constants.GAINS.TURN.peakOutput);

        turnSetters.addConfigurator(new SparkConfiguration(turnMotor, turnConfig));
        turnEncoder = turnMotor.getEncoder();
        //#endregion

        homedEntry = nTable.getTopic("Homed [" + moduleID + "]").getGenericEntry();
        switchEntry = nTable.getTopic("Switch [" + moduleID + "]").getGenericEntry();
    }    

    public void periodicDebug() {
        switchEntry.setBoolean(getSwitch());
        homedEntry.setBoolean(homed);
    }

    /**
     * Set the desired state for the module. Drive speeds of 0 will result in no azimuth movement.
     * @param desiredState the desired state
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        if(Math.abs(desiredState.speedMetersPerSecond) < .001){
            stop();
            return;
        }

        desiredState.optimize(getWrappedAngle());
        desiredVelocity.Velocity = desiredState.speedMetersPerSecond;
        driveMotor.setControl(desiredVelocity);

        turnPIDController.setReference(desiredState.angle.getRadians(), SparkMax.ControlType.kPosition);
    }

    public void home(){
        boolean switchState = getSwitch();
        //only triggers on rising edge of switch, set new home state and target location
        if (switchState != wasHomed && switchState) {
            turnEncoder.setPosition(moduleOffset);
            homed = true;
        }

        //if it is triggered, set to target; else keep rotating
        if (homed)
            turnPIDController.setSetpoint(moduleOffset, SparkMax.ControlType.kPosition);
        else
            turnMotor.set(0.25);

        wasHomed = switchState;
    }

    public boolean getSwitch(){
        return !hallEffectSensor.get();
    }

    /**
     * @return speed of drive motor in meters/second
     */
    public double getSpeed(){
        return driveMotor.getVelocity().getValueAsDouble();
    }

    /**
     * Angle of module. This is the unbound version raw from encoder
     * @return angle of module 
     */
    public Rotation2d getAngle(){
        return new Rotation2d(turnEncoder.getPosition());
    }

    /**
     * Angle of the swerve module constrained to (-PI, PI)
     * @return
     */
    public Rotation2d getWrappedAngle(){
        return new Rotation2d(MathUtil.angleModulus(getAngle().getRadians()));
    }

    public void setHomed(boolean state){
        homed = state;
        wasHomed = getSwitch();
    }

    /**
     * Direct control of motor output percent
     * @param drive drive motor speed
     * @param turn turn motor speed
     */
    public void setMotorSpeeds(double drive, double turn){
        turnMotor.set(turn);
        driveMotor.set(drive);
    }

    public SwerveModuleState getState(){
        return new SwerveModuleState(getSpeed(), getAngle());
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(driveMotor.getPosition().getValueAsDouble(), getAngle());
    }

    public void stop(){
        driveMotor.set(0);
        turnMotor.set(0);
    }

    public void zeroEncoders() {
        driveMotor.setPosition(0);
        turnEncoder.setPosition(0);
    }
}
