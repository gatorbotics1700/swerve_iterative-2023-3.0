package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kForward;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kOff;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kReverse;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;

public class BuddyClimb{ //up and down state, starts down then up, down 

    private DoubleSolenoid solenoidOne;
    private DoubleSolenoid solenoidTwo; 
    public static PneumaticClimbStates pneumaticClimbState;

    public BuddyClimb(){
        solenoidOne = new DoubleSolenoid(10, PneumaticsModuleType.REVPH, 13, 10); 
        solenoidTwo = new DoubleSolenoid(10, PneumaticsModuleType.REVPH, 13, 10); //check forward and reverse chanel for both solenoid 
        init();
    }

    public static enum PneumaticClimbStates{
        UP,
        DOWN,
        OFF;
    }

    public void init(){
        pneumaticClimbState = PneumaticClimbStates.DOWN;
        solenoidOne.set(kOff);
        solenoidTwo.set(kOff);
        System.out.println("solenoid set to off");
    }

    public void periodic(){
        if(pneumaticClimbState == PneumaticClimbStates.DOWN){
            solenoidOne.set(kForward);
            solenoidTwo.set(kForward);  
        } else if (pneumaticClimbState == PneumaticClimbStates.UP){
            solenoidOne.set(kReverse);
            solenoidTwo.set(kForward); 
        }else{
            solenoidOne.set(kOff);
            solenoidTwo.set(kForward); 
        }
    }

    public void setState(PneumaticClimbStates newClimbState){
        pneumaticClimbState = newClimbState;
    }
}