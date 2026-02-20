// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Util;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;

import frc.robot.Util.PIDDisplay.PIDSetter;

/** Add your docs here. */
public class TalonFXSetter implements PIDSetter {
    private List<TalonFXConfigurator> configurators = new ArrayList<TalonFXConfigurator>();
    private Gains gains;

    public TalonFXSetter(TalonFXConfigurator... configurators){
        this.configurators.addAll(Arrays.asList(configurators));
    }

    public void addConfigurator(TalonFXConfigurator configurator) {
        configurators.add(configurator);
    }

    @Override
    public void setPID(Gains gains) {
        this.gains = gains;

        Slot0Configs slot0Configs = new Slot0Configs();
        slot0Configs.kP = gains.P;
        slot0Configs.kI = gains.I;
        slot0Configs.kD = gains.D;
        slot0Configs.kS = gains.S;
        slot0Configs.kV = gains.V;

        configurators.forEach(configurator -> configurator.apply(slot0Configs));
    }

    @Override
    public Gains getPID() {
        return gains;
    }
}
