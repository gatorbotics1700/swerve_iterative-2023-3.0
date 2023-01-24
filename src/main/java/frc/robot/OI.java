package frc.robot;

import edu.wpi.first.wpilibj.XboxController.*;
import edu.wpi.first.wpilibj.XboxController;

public class OI {
    public static final XboxController m_controller = new XboxController(0);

    public static Button stop = Button.kB;

    public static Button intakeOn = Button.kA; // second xbox controller -- need to figure out how to map this to a certain xbox controller

    public static Button intakeOff = Button.kX; // second xbox controller
}
