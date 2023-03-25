// package frc.robot.subsystems;

// import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
// import edu.wpi.first.wpilibj.PneumaticsModuleType;
// import edu.wpi.first.wpilibj.DoubleSolenoid;

// public class ArmPneumaticPivot { //actuate = down, retract = up

//     public static enum PneumaticPivotStates{
//         DOWN,
//         UP,
//         OFF;
//     }

//     public PneumaticPivotStates pneumaticPivotState;
//     public static DoubleSolenoid solenoid;

//     public ArmPneumaticPivot(){
//         solenoid = new DoubleSolenoid(10, PneumaticsModuleType.REVPH, 3, 7); //TODO: change port number + check channels
//         init();
//     }

//     public void init() {
//         pneumaticPivotState  = PneumaticPivotStates.UP;
//         System.out.println("solenoid set to off");
//     }

//     public void periodic(){
//         if (pneumaticPivotState == PneumaticPivotStates.DOWN){
//             solenoid.set(Value.kForward);
//         } else if (pneumaticPivotState == PneumaticPivotStates.UP){
//             solenoid.set(Value.kReverse);
//         } else {
//             solenoid.set(Value.kOff);
//         }
//     }

//     public void setState(PneumaticPivotStates newPivotState){
//         pneumaticPivotState = newPivotState;
//     }
// }

package frc.robot.subsystems;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kForward;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kOff;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kReverse;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.Constants;

public class ArmPneumaticPivot {

    public static enum PneumaticPivotStates{
        UP, 
        DOWN, 
        OFF;
    }

    public PneumaticPivotStates pneumaticPivotStates;
    public DoubleSolenoid solenoidOne; 
    // Initializes a DigitalInput on DIO 0 (roborio is built in w/ 10 DIOs (digital input-output ports))

    public ArmPneumaticPivot(){ //TODO: add back compressor if it turns out we need it
        solenoidOne = new DoubleSolenoid(10, PneumaticsModuleType.REVPH, 10, 13); //3 & 7
        init();
    }

    public void init(){
        pneumaticPivotStates = PneumaticPivotStates.UP;
    }

    public void periodic(){
       if(pneumaticPivotStates == PneumaticPivotStates.DOWN){
            solenoidOne.set(kReverse); 
        } else if (pneumaticPivotStates == PneumaticPivotStates.UP){
            solenoidOne.set(kForward);
        }else{
            solenoidOne.set(kOff);
        }
    }

    public void setStatePneumaticIntake(PneumaticPivotStates newPivotState){
        pneumaticPivotStates = newPivotState;
    }

 }
