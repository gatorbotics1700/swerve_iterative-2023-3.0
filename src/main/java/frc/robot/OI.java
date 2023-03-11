package frc.robot;

import edu.wpi.first.wpilibj.XboxController.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.XboxController;

public class OI {

    public double intakeOn, intakeOff;

    public static final XboxController m_controller = new XboxController(0);//main driver/driving controller
    public static final XboxController m_controller_two = new XboxController(1);//buttons/co-driver controller

    public static Button pitchPD = Button.kLeftBumper;//pitch pd for charge station
    public static Button stop = Button.kRightBumper;//stop drive bumper

}
