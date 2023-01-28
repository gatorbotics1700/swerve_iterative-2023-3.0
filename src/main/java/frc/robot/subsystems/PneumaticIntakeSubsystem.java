package frc.robot.subsystems;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Compressor;

public class PneumaticIntakeSubsystem {
    public static enum PneumaticIntakeStates{
        ACTUATING, 
        RETRACTING, 
        OFF;
    }

    public static PneumaticIntakeStates pneumaticIntakeStates = PneumaticIntakeStates.OFF;

    //single solenoid?
    public static DoubleSolenoid solenoidOne = new DoubleSolenoid(2, PneumaticsModuleType.REVPH, 8, 9); 
    //what compressor are we using?
    public static Compressor compressor = new Compressor(PneumaticsModuleType.REVPH); 

    public void init(){
        solenoidOne.set(kOff);
        System.out.println("soleinoid set to off");
    }

    public void setStatePneumaticIntake(PneumaticIntakeStates newState){
        pneumaticIntakeStates = newState;
    }

    public void periodic(){
        if(pneumaticIntakeStates == PneumaticIntakeStates.ACTUATING){
            solenoidOne.set(kForward);
            System.out.println("Solenoid Actuating");
        } else if (pneumaticIntakeStates == PneumaticIntakeStates.RETRACTING){
            solenoidOne.set(kReverse);
            System.out.println("SolenoidRetracting");
        }else{
            solenoidOne.set(kOff);
            System.out.println("Solenoid Off");
        }
    }

    public boolean getPSI(){
        System.out.println(compressor.getCurrent());
        return compressor.getPressureSwitchValue();
    }
}
