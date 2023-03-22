package frc.robot;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.Joystick;

public class OI {

    public static final XboxController m_controller = new XboxController(0);//main driver/driving controller
    public static final XboxController m_controller_two = new XboxController(1);//buttons/co-driver controller

    public static final  Joystick joystick = new Joystick(2);
    public static final JoystickButton button0 = new JoystickButton(joystick, 0);
    public static final JoystickButton button1 = new JoystickButton(joystick, 1);
    public static final JoystickButton button2 = new JoystickButton(joystick, 2);
    public static final JoystickButton button3 = new JoystickButton(joystick, 3);
    public static final JoystickButton button4 = new JoystickButton(joystick, 4);
    public static final JoystickButton button5 = new JoystickButton(joystick, 5);
    public static final JoystickButton button6 = new JoystickButton(joystick, 6);
    public static final JoystickButton button7 = new JoystickButton(joystick, 7);
    public static final JoystickButton button8 = new JoystickButton(joystick, 8);

    public static double getLeftAxis(){
        return m_controller_two.getLeftY();
    }

    public static double getRightAxis(){
        return m_controller_two.getRightY();
    }
}
