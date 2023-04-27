package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kForward;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kOff;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kReverse;
import frc.robot.Robot;

public class PneumaticArmPivot { //actuate = down, retract = up

    private DoubleSolenoid solenoid; 
    public PneumaticPivotStates pneumaticPivotState;

    public PneumaticArmPivot(){
        solenoid = new DoubleSolenoid(10, PneumaticsModuleType.CTREPCM, 6, 7); //a little worried this being in constructor could break it
        init();
    }

    public static enum PneumaticPivotStates{
        UP,
        DOWN,
        OFF;
    }

    public void init() {
        pneumaticPivotState = PneumaticPivotStates.OFF;
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
