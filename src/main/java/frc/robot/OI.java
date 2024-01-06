package frc.robot;

import edu.wpi.first.wpilibj.XboxController.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.XboxController;

public class OI {

    public double intakeOn, intakeOff;

    public static final XboxController m_controller = new XboxController(0);//buttons/co-driver controller
    public static final XboxController m_controller_two = new XboxController(1);//main driver/driving controller

    public static Button stop = Button.kRightBumper;//stop drive bumper

    public static Button turnNorth = Button.kY;//button to turn to true north

    public static Button outtake = Button.kA; 
    

    /*two total xbox controllers, one for driver, one for co-driver. 4 buttons on each, 2 joysticks, 2 triggers, 2 bumbers, 2 additional buttons at the back of the contoller, and possibly 4 directional arrow keys. All can be programmed for different settings as desired.
     * buttons/joysticks to be coded:
     * - make joystick for driving robot
     * - outtake button
     */

    
}

