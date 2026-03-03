// IMPORTED FROM JARVIS 2025, USED ONLY FOR HOMING

package frc.robot.util;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfigurator;

import frc.robot.util.PIDDisplay.PIDSetter;

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
