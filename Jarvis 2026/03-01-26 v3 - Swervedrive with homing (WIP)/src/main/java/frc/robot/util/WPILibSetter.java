// IMPORTED FROM JARVIS 2025, USED ONLY FOR HOMING

package frc.robot.util;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.util.PIDDisplay.PIDSetter;

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
