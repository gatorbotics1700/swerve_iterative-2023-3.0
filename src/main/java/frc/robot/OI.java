package frc.robot;

import edu.wpi.first.wpilibj.XboxController.*;
import edu.wpi.first.wpilibj.XboxController;

public class OI {
    public static final XboxController m_controller = new XboxController(0);

    public static Button stop = Button.kB;
    public static Button armExtended  = Button.kY;//button to fully extend to 90 degrees
    public static Button armControl  = Button.kLeftStick;//stick to control arm length (any length from fully retracted to extended)
}
