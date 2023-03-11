package frc.robot;

import edu.wpi.first.wpilibj.XboxController.*;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Joystick;

public class OI {

    public double intakeOn, intakeOff; 

    public static final XboxController m_controller = new XboxController(0);//buttons/co-driver controller
    public static final XboxController m_controller_two = new XboxController(1);//main driver/driving controller

    public static Button pitchPD = Button.kLeftBumper;//pitch pd for charge station
    public static Button stop = Button.kRightBumper;//stop drive bumper

    public static final  Joystick joystick = new Joystick(0);
    public static final JoystickButton button0 = new JoystickButton(joystick, 0);
    public static final JoystickButton button1 = new JoystickButton(joystick, 1);
    public static final JoystickButton button2 = new JoystickButton(joystick, 2);
    public static final JoystickButton button3 = new JoystickButton(joystick, 3);
    public static final JoystickButton button4 = new JoystickButton(joystick, 4);
    public static final JoystickButton button5 = new JoystickButton(joystick, 5);
    public static final JoystickButton button6 = new JoystickButton(joystick, 6);
    public static final JoystickButton button7 = new JoystickButton(joystick, 7);
    public static final JoystickButton button8 = new JoystickButton(joystick, 8);

}
