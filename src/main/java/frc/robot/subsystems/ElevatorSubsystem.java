package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import frc.robot.Constants;

public class ElevatorSubsystem {

    public ElevatorSubsystem(){}

    public static TalonFX elevatorMotor = new TalonFX(Constants.ELEVATOR_CAN_ID);
    public static ElevatorStates elevatorState = ElevatorStates.MID_ELEVATOR_HEIGHT;
    
    public static enum ElevatorStates{
        ZERO, 
        LOW_ELEVATOR_HEIGHT,
        MID_ELEVATOR_HEIGHT,
        HIGH_ELEVATOR_HEIGHT;
    }

    public void init(){
        elevatorMotor.setInverted(false);
        elevatorMotor.setNeutralMode(NeutralMode.Brake);
    }

    public void periodic(){
        if (elevatorState == ElevatorStates.ZERO){

        } else if (elevatorState == ElevatorStates.LOW_ELEVATOR_HEIGHT){

        } else if (elevatorState == ElevatorStates.MID_ELEVATOR_HEIGHT){

        } else {
            
        }
    }

    public void setState(ElevatorStates newElevatorState){
        elevatorState = newElevatorState;
    }
}
