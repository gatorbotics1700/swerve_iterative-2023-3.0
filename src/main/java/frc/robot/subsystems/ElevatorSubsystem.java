package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import frc.robot.Constants;
import frc.robot.Gains;
import edu.wpi.first.wpilibj.DigitalInput;

import com.ctre.phoenix.motorcontrol.ControlMode;

//NOTES ON MEASUREMENTS
//22.5inches on inside of metal frame that chain moves in
//10 in on moving mechanism thing

public class ElevatorSubsystem {

    //these values are to be determined (untested)
    public double _kP = 0.05;
    public double _kI = 0.0;
    public double _kD = 0.0;
    public int _kIzone = 0;
    public double _kPeakOutput = 1.0;

    public double scaleDown = 1.214; //this value was determined using tested values and plotted by lauren mcdonald in loggerpro!

    public static TalonFX elevatorMotor = new TalonFX(Constants.ELEVATOR_CAN_ID);
    public static ElevatorStates elevatorState = ElevatorStates.LOW_ELEVATOR_HEIGHT;

    DigitalInput top_limit_switch = new DigitalInput(Constants.topLimitSwitchPort);
    DigitalInput bottom_limit_switch = new DigitalInput(Constants.bottomLimitSwitchPort);
    
    public Gains elevatorGains = new Gains(_kP, _kI, _kD, _kIzone, _kPeakOutput);

    public static enum ElevatorStates{
        STOP, //goes with limit switch
        LOW_ELEVATOR_HEIGHT,
        SHELF_ELEVATOR_HEIGHT,
        MID_ELEVATOR_HEIGHT,
        HIGH_ELEVATOR_HEIGHT;
    }

    public void init(){
        System.out.println("elevator init!!!!");
        elevatorMotor.setInverted(true);
        elevatorMotor.setNeutralMode(NeutralMode.Brake);

        //configuring deadband
        elevatorMotor.configAllowableClosedloopError(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
		/* Config Position Closed Loop gains in slot0, typically kF stays zero. */
		elevatorMotor.config_kP(Constants.kPIDLoopIdx, elevatorGains.kP, Constants.kTimeoutMs);
		elevatorMotor.config_kI(Constants.kPIDLoopIdx, elevatorGains.kI, Constants.kTimeoutMs);
		elevatorMotor.config_kD(Constants.kPIDLoopIdx, elevatorGains.kD, Constants.kTimeoutMs);
    }

    public void periodic(){//still need to scale down the values or figure out why it is overshooting
        System.out.println("elevator periodic!!!!!!!");
        if (elevatorState == ElevatorStates.STOP){ //emergency stop
            elevatorMotor.set(ControlMode.PercentOutput, 0);
        } else if (elevatorState == ElevatorStates.LOW_ELEVATOR_HEIGHT){
            elevatorMotor.set(ControlMode.Position, 0 * Constants.TICKS_PER_INCH); //official 2/13
        } else if (elevatorState == ElevatorStates.SHELF_ELEVATOR_HEIGHT) {
            elevatorMotor.set(ControlMode.Position, 39 * Constants.TICKS_PER_INCH); //oficial 2/13
        } else if (elevatorState == ElevatorStates.MID_ELEVATOR_HEIGHT){
            elevatorMotor.set(ControlMode.Position, 40 * Constants.TICKS_PER_INCH); //official 2/13
        } else { //high elevator height
            elevatorMotor.set(ControlMode.Position, 48 * Constants.TICKS_PER_INCH); //change value once we know robot dimensions
        }

        if(top_limit_switch.get() || bottom_limit_switch.get()){
            setState(ElevatorStates.STOP);
            System.out.println("Top/bottom limit switch triggered");
        }
    }

    public void setState(ElevatorStates newElevatorState){
        elevatorState = newElevatorState;
    }
}
