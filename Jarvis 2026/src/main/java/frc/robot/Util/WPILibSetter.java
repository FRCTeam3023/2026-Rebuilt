// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Util;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.Util.PIDDisplay.PIDSetter;

/** Add your docs here. */
public class WPILibSetter implements PIDSetter{
    private List<PIDController> controllers;
    private Gains gains;

    public WPILibSetter(List<PIDController> controllers){
        this.controllers = controllers;
    }

    public void addController(PIDController controller) {
        controllers.add(controller);
    }

    @Override
    public void setPID(Gains gains) {
        this.gains = gains;
        controllers.forEach(controller -> controller.setPID(gains.P, gains.I, gains.D));
    }

    @Override
    public Gains getPID() {
        return gains;
    }
}
