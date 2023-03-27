package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

public class OI {


    public static final XboxController m_controller = new XboxController(0);//main driver/driving controller
    public static final XboxController m_controller_two = new XboxController(1);//buttons/co-driver controller

    public static double getLeftAxis(){
        return m_controller_two.getLeftY();
    }

    public static double getRightAxis(){
        return m_controller_two.getRightY();
    }
}
