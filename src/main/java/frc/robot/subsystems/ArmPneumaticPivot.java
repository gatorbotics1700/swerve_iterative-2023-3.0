package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public class ArmPneumaticPivot { //actuate = down, retract = up

    public static enum PneumaticPivotStates{
        DOWN,
        UP,
        OFF;
    }

    public PneumaticPivotStates pneumaticPivotState = PneumaticPivotStates.UP;

    public static Solenoid solenoid = new Solenoid(PneumaticsModuleType.REVPH, 4); //TODO: change port number

    public void init() {
        solenoid.set(false);
        System.out.println("solenoid set to off");
    }

    public void periodic(){
        if (pneumaticPivotState == PneumaticPivotStates.DOWN){
            solenoid.set(true);
        } else if (pneumaticPivotState == PneumaticPivotStates.UP){
            solenoid.set(false);
        } else {
            solenoid.set(false);
        }
    }

    public boolean getPSI(){
        return true;
        //System.out.println(piSystem.compressor.getCurrent());
        //return piSystem.compressor.getPressureSwitchValue();
    }


    public void setState(PneumaticPivotStates newPivotState){
        pneumaticPivotState = newPivotState;
    }
}
