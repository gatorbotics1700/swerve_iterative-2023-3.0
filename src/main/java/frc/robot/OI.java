package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

public class OI {

    public static final XboxController m_controller_elec = new XboxController(0);//buttons/co-driver controller

    public static double getElecAxis(){
        return m_controller_elec.getLeftY();
    }
}
