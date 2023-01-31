package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import frc.robot.Constants;
import frc.robot.Gains;

import com.ctre.phoenix.motorcontrol.ControlMode;

public class ElevatorSubsystem {

    //these values are to be determined (untested)
    public double _kP = 1.0;
    public double _kI = 0.0;
    public double _kD = 0.0;
    public int _kIzone = 0;
    public double _kPeakOutput = 1.0;

    public static TalonFX elevatorMotor = new TalonFX(Constants.ELEVATOR_CAN_ID);
    public static ElevatorStates elevatorState = ElevatorStates.MID_ELEVATOR_HEIGHT;
    
    public Gains elevatorGains = new Gains(_kP, _kI, _kD, _kIzone, _kPeakOutput);

    public static enum ElevatorStates{
        ZERO,
        LOW_ELEVATOR_HEIGHT,
        MID_ELEVATOR_HEIGHT,
        HIGH_ELEVATOR_HEIGHT;
    }

    public void init(){
        elevatorMotor.setInverted(false);
        elevatorMotor.setNeutralMode(NeutralMode.Brake);

        //configuring deadband
        elevatorMotor.configAllowableClosedloopError(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
		/* Config Position Closed Loop gains in slot0, typically kF stays zero. */
		elevatorMotor.config_kP(Constants.kPIDLoopIdx, elevatorGains.kP, Constants.kTimeoutMs);
		elevatorMotor.config_kI(Constants.kPIDLoopIdx, elevatorGains.kI, Constants.kTimeoutMs);
		elevatorMotor.config_kD(Constants.kPIDLoopIdx, elevatorGains.kD, Constants.kTimeoutMs);
    }

    public void periodic(){
        if (elevatorState == ElevatorStates.ZERO){
            elevatorMotor.set(ControlMode.Position, 0);
        } else if (elevatorState == ElevatorStates.LOW_ELEVATOR_HEIGHT){
            elevatorMotor.set(ControlMode.Position, 100); //change value once we know robot dimensions
        } else if (elevatorState == ElevatorStates.MID_ELEVATOR_HEIGHT){
            elevatorMotor.set(ControlMode.Position, 24.5); //change value once we know robot dimensions
        } else {
            elevatorMotor.set(ControlMode.Position, 300); //change value once we know robot dimensions
        }
    }

    public void setState(ElevatorStates newElevatorState){
        elevatorState = newElevatorState;
    }
}
