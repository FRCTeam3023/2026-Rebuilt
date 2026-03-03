// IMPORTED FROM JARVIS 2025, USED ONLY FOR HOMING

package frc.robot.util;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkBaseConfig;

import frc.robot.util.PIDDisplay.PIDSetter;

public class SparkBaseSetter implements PIDSetter {
    public static class SparkConfiguration {
        SparkBase motor;
        SparkBaseConfig config;

        /**
         * Creates an object that stores a motor base and its config.
         * This is merely to organize things and is not used for anything else.
         */
        public SparkConfiguration(SparkBase motor, SparkBaseConfig config) {
            this.motor = motor;
            this.config = config;
        }
    }

    private List<SparkConfiguration> configurations = new ArrayList<SparkConfiguration>();
    private Gains gains;

    public SparkBaseSetter(SparkConfiguration... configurations) {
        this.configurations.addAll(Arrays.asList(configurations));
    }

    public void addConfigurator(SparkConfiguration configuration) {
        configurations.add(configuration);
    }

    @Override
    public void setPID(Gains gains) {
        this.gains = gains;
        configurations.forEach(configuration -> {
            configuration.config.closedLoop.pidf(gains.P, gains.I, gains.D, gains.F);
            configuration.motor.configure(configuration.config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
        });
    }

    @Override
    public Gains getPID() {
        return gains;
    }

}
