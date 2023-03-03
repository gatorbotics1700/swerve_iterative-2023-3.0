package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kForward;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kOff;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kReverse;

public class ArmPneumaticPivot { //actuate = down, retract = up

    public static enum PneumaticPivotStates{
        ACTUATING,
        RETRACTING,
        OFF;
    }
    public PneumaticPivotStates pneumaticPivotState = PneumaticPivotStates.OFF;

    private Compressor compressor = new Compressor(PneumaticsModuleType.REVPH); 
    private DoubleSolenoid solenoid = new DoubleSolenoid(2, PneumaticsModuleType.REVPH, 8, 9); 

    public void init() {
        solenoid.set(kOff);
    }

    public void periodic(){
        if (pneumaticPivotState == PneumaticPivotStates.ACTUATING){
            solenoid.set(kForward);
        } else if (pneumaticPivotState == PneumaticPivotStates.RETRACTING){
            solenoid.set(kReverse);
        } else {
            solenoid.set(kOff);
        }
    }

    public boolean getPSI(){
        System.out.println(compressor.getCurrent());
        return compressor.getPressureSwitchValue();
    }


    public void setState(PneumaticPivotStates newPivotState){
        pneumaticPivotState = newPivotState;
    }
}