package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

public class OI {
    //deleted getters because variables are public
    public static final XboxController m_controller = new XboxController(0);//main driver/driving controller
    public static final XboxController m_controller_two = new XboxController(1);//buttons/co-driver controller
}
