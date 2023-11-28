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
        // TODO
    }

    public PneumaticIntakeStates pneumaticIntakeState;
    private DoubleSolenoid solenoidOne; 
    // Initializes a DigitalInput on DIO 0 (roborio is built in w/ 10 DIOs (digital input-output ports))
    private DigitalInput beambreakSensor;

    public PneumaticIntakeSubsystem(){
        solenoidOne = new DoubleSolenoid(10, PneumaticsModuleType.CTREPCM, 4, 5); //3 & 7
        beambreakSensor = new DigitalInput(Constants.BEAM_BREAK_RECEIVER); 
        init();
    }

    public void init(){
        // set the state to PINCHING
    }

    public void periodic(){
       // TODO
    }

    public void setStatePneumaticIntake(PneumaticIntakeStates newIntakeState){
        // TODO
    }

    public boolean isBeamBroken(){
        return beambreakSensor.get(); 
    }
 }
