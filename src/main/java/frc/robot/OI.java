package frc.robot;

import edu.wpi.first.wpilibj.XboxController.*;

//import edu.wpi.first.wpilibj2.command.button.Trigger;

import edu.wpi.first.wpilibj.XboxController;

public class OI {

    public static final XboxController m_driver_controller = new XboxController(0);
    public static final XboxController m_codriver_controller = new XboxController(1);
    public static double intakeOn;
    public static double intakeOff;

    public void triggerPeriodic() {
         intakeOn = triggerThreshold (m_codriver_controller.getLeftTriggerAxis());
         intakeOff = triggerThreshold (m_codriver_controller.getRightTriggerAxis());
    }

    public double triggerThreshold(double tValue) {
        if (Math.abs(tValue) < 0.1) { //change this value to be more specific if wanted
            return 0;
        } else {
            return 1 * tValue;
        }
        //trigger axis is bound by the range [0,1] not [-1,1]
    }
    
    //two total xbox controllers, one for driver, one for co-driver. 4 buttons on each, 2 triggers, 2 bumpers
   
}



