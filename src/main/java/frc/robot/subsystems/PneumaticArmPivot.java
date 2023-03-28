package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kForward;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kOff;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kReverse;
import frc.robot.Robot;

public class PneumaticArmPivot { //actuate = down, retract = up

    public static enum PneumaticPivotStates{
        UP,
        DOWN,
        OFF;
    }

    public PneumaticPivotStates pneumaticPivotState = PneumaticPivotStates.OFF;

    public static DoubleSolenoid solenoid = new DoubleSolenoid(10, PneumaticsModuleType.REVPH, 13, 10); 

    public void init() {
        solenoid.set(kOff);
        System.out.println("solenoid set to off");
    }

    public void periodic(){
        if (pneumaticPivotState == PneumaticPivotStates.UP){
            solenoid.set(kForward);
        } else if (pneumaticPivotState == PneumaticPivotStates.DOWN){
            solenoid.set(kReverse);
            System.out.println("retracting");
        } else {
            solenoid.set(kOff);
        }
    }

    public void setState(PneumaticPivotStates newPivotState){
        pneumaticPivotState = newPivotState;
    }
}
