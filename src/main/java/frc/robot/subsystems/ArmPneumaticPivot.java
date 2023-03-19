package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kForward;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kOff;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kReverse;
import frc.robot.Robot;

public class ArmPneumaticPivot { //actuate = down, retract = up

    // public static enum PneumaticPivotStates{
    //     ACTUATING,
    //     RETRACTING,
    //     OFF;
    // }
    
    // public PneumaticIntakeSubsystem piSystem = Robot.m_pneumaticIntakeSubsystem;

    // public PneumaticPivotStates pneumaticPivotState = PneumaticPivotStates.RETRACTING;

    // public static DoubleSolenoid solenoid = new DoubleSolenoid(10, PneumaticsModuleType.REVPH, 4, 6); 

    // public void init() {
    //     solenoid.set(kOff);
    //     System.out.println("solenoid set to off");
    // }

    // public void periodic(){
    //     if (pneumaticPivotState == PneumaticPivotStates.ACTUATING){
    //         solenoid.set(kForward);
    //     } else if (pneumaticPivotState == PneumaticPivotStates.RETRACTING){
    //         solenoid.set(kReverse);
    //         System.out.println("retracting");
    //     } else {
    //         solenoid.set(kOff);
    //     }
    // }

    // public boolean getPSI(){
    //     return true;
    //     //System.out.println(piSystem.compressor.getCurrent());
    //     //return piSystem.compressor.getPressureSwitchValue();
    // }


    // public void setState(PneumaticPivotStates newPivotState){
    //     pneumaticPivotState = newPivotState;
    // }
}
