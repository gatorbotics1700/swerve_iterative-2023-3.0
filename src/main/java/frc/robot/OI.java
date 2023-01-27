package frc.robot;

import edu.wpi.first.wpilibj.XboxController.*;
import edu.wpi.first.wpilibj.XboxController;

public class OI {
    public static final XboxController m_controller = new XboxController(0);
    public static final XboxController m_controller_two = new XboxController(1);

    public static Button stop = Button.kB;
    public static Button turnNorth = Button.kA;//button to turn to true north
    public static Button armExtended  = Button.kY;//button to fully extend to 90 degrees
    public static Button swingThrough  = Button.kX;//button to go through
    public static Button armControl  = Button.kLeftStick;//stick to control arm length (any length from fully retracted to extended)

    public static Button intakeOn = Button.kA; // second xbox controller -- need to figure out how to map this to a certain xbox controller

    public static Button intakeOff = Button.kX; // second xbox controller
}
