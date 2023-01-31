package frc.robot;

import edu.wpi.first.wpilibj.XboxController.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.XboxController;

public class OI {

    public double intakeOn, intakeOff;

    public static final XboxController m_controller = new XboxController(0);//buttons/co-driver controller
    public static final XboxController m_controller_two = new XboxController(1);//main driver/driving controller

    //public static Button armExtended  = Button.kY;//button to fully extend to 90 degrees
    //public static Button armControl  = Button.kLeftStick;//stick to control arm length (any length from fully retracted to extended)
    public static Button pitchPD = Button.kLeftBumper;//pitch pd for charge station
    public static Button stop = Button.kRightBumper;//stop drive bumper
    //public static Trigger leftTrigger = Trigger.;

    public static Button lowShelf = Button.kX; // low shelf height combo of elevator and arm angle
    public static Button midShelf = Button.kA; // mid shelf
    public static Button highShelf = Button.kB; // high shelf
    public static Button turnNorth = Button.kY;//button to turn to true north


    public static Button outtake = Button.kA; 
    
    public void triggerPeriodic() {
        intakeOn = triggerThreshold (m_controller.getLeftTriggerAxis());
        intakeOff = triggerThreshold (m_controller.getRightTriggerAxis());
    }
    /*two total xbox controllers, one for driver, one for co-driver. 4 buttons on each, 2 joysticks, 2 triggers, 2 bumbers, 2 additional buttons at the back of the contoller, and possibly 4 directional arrow keys. All can be programmed for different settings as desired.
     * buttons/joysticks to be coded:
     * - make joystick for driving robot
     * - outtake button
     */

    public static double triggerThreshold(double tValue) {
        if (Math.abs(tValue) < 0.1) { //change this value to be more specific if wanted
            return 0;
        } else {
            return 1 * tValue;
        }
        //trigger axis is bound by the range [0,1] not [-1,1]
    }
}