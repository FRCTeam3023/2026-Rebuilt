// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Util;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Add your docs here. */

public class PIDDisplay extends SubsystemBase{

    //shuffeboard stuff

    public static final SendableChooser<PIDSetter> PIDList = new SendableChooser<>();
    public static final WPILibSetter defaultPID = new WPILibSetter(List.of(new PIDController(0, 0, 0)));

    private static NetworkTable nTable = NetworkTableInstance.getDefault().getTable("SmartDashboard/PID Display");

    private static GenericEntry PEntry = nTable.getTopic("P").getGenericEntry();
    private static GenericEntry IEntry = nTable.getTopic("I").getGenericEntry();
    private static GenericEntry DEntry = nTable.getTopic("D").getGenericEntry();
    private static GenericEntry SEntry = nTable.getTopic("S").getGenericEntry();
    private static GenericEntry VEntry = nTable.getTopic("V").getGenericEntry();

    PIDSetter selectedPID;
    PIDSetter lastSelected;

    double PValue;
    double IValue;
    double DValue;
    double SValue;
    double VValue;

    public static void Init() {
        SmartDashboard.putData("PID Display/PID Selector", PIDList);
        updatePID(new Gains(0, 0));
        PIDList.setDefaultOption("Default PID", defaultPID);
    }

    private static void updatePID(Gains gains) {
        PEntry.setDouble(gains.P);
        IEntry.setDouble(gains.I);
        DEntry.setDouble(gains.D);
        SEntry.setDouble(gains.S);
        VEntry.setDouble(gains.V);
    }

    @Override
    public void periodic(){
        //Check if the selected PID config is different. If so update the Dashboard with new numbers
        selectedPID = PIDList.getSelected();
        if (selectedPID == null) return;
        Gains codeCurrentPID = selectedPID.getPID();
        if (codeCurrentPID == null) return;
        
        if(selectedPID != lastSelected)
            updatePID(codeCurrentPID);
        lastSelected = selectedPID;
        
        PValue = PEntry.getDouble(codeCurrentPID.P);
        IValue = IEntry.getDouble(codeCurrentPID.I);
        DValue = DEntry.getDouble(codeCurrentPID.D);
        SValue = SEntry.getDouble(codeCurrentPID.S);
        VValue = VEntry.getDouble(codeCurrentPID.V);

        //Check if any of the dashboard values are different than the PID values in code, write updated values
        if(PValue != codeCurrentPID.P || IValue != codeCurrentPID.I || DValue != codeCurrentPID.D || SValue != codeCurrentPID.S || VValue != codeCurrentPID.V){
            selectedPID.setPID(new Gains(PValue, IValue, DValue, SValue, VValue));
        }
    }

    public interface PIDSetter {
        public void setPID(Gains gains);

        public Gains getPID();
    }
}
