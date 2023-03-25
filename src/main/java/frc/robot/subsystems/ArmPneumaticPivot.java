package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid;

public class ArmPneumaticPivot { //actuate = down, retract = up

    public static enum PneumaticPivotStates{
        DOWN,
        UP,
        OFF;
    }

    public PneumaticPivotStates pneumaticPivotState = PneumaticPivotStates.UP;

    public static DoubleSolenoid solenoid = new DoubleSolenoid(10, PneumaticsModuleType.REVPH, 3, 7); //TODO: change port number + check channels

    public void init() {
        solenoid.set(Value.kOff);
        System.out.println("solenoid set to off");
    }

    public void periodic(){
        if (pneumaticPivotState == PneumaticPivotStates.DOWN){
            solenoid.set(Value.kForward);
        } else if (pneumaticPivotState == PneumaticPivotStates.UP){
            solenoid.set(Value.kReverse);
        } else {
            solenoid.set(Value.kOff);
        }
    }

    public void setState(PneumaticPivotStates newPivotState){
        pneumaticPivotState = newPivotState;
    }
}
