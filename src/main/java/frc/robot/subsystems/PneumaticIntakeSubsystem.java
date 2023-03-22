package frc.robot.subsystems;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kForward;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kOff;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kReverse;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.Constants;

public class PneumaticIntakeSubsystem {

    public static enum PneumaticIntakeStates{
        PINCHING, 
        RELEASING, 
        OFF;
    }

    public PneumaticIntakeStates pneumaticIntakeState;
    private DoubleSolenoid solenoidOne; 
    // Initializes a DigitalInput on DIO 0 (roborio is built in w/ 10 DIOs (digital input-output ports))
    private DigitalInput beambreakSensor;

    public PneumaticIntakeSubsystem(){ //TODO: add back compressor if it turns out we need it
        solenoidOne = new DoubleSolenoid(10, PneumaticsModuleType.REVPH, 5, 7);
        beambreakSensor = new DigitalInput(Constants.BEAM_BREAK_RECEIVER); 
        init();
    }

    public void init(){
        pneumaticIntakeState = PneumaticIntakeStates.PINCHING;
    }

    public void periodic(){
       if(pneumaticIntakeState == PneumaticIntakeStates.PINCHING){
            solenoidOne.set(kForward); //TODO: make sure this is the right direction
        } else if (pneumaticIntakeState == PneumaticIntakeStates.RELEASING){
            solenoidOne.set(kReverse);
        }else{
            solenoidOne.set(kOff);
        }
    }

    public void setStatePneumaticIntake(PneumaticIntakeStates newIntakeState){
        pneumaticIntakeState = newIntakeState;
    }

    public boolean isBeamBroken(){
        return beambreakSensor.get(); 
    }
 }
