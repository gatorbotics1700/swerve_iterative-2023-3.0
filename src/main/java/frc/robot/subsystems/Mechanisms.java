package frc.robot.subsystems;

import frc.robot.subsystems.ArmTelescopingSubsystem;
import frc.robot.subsystems.ArmTelescopingSubsystem.TelescopingStates;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorStates;
import frc.robot.Constants;

public class Mechanisms {
    
    private static ArmTelescopingSubsystem armTelescopingSubsystem = new ArmTelescopingSubsystem();
    private static ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
    

    public void init(){
        //elevator
        elevatorSubsystem.init();

        //telescope
        armTelescopingSubsystem.setTState(TelescopingStates.SHELF_ARM_LENGTH); //moved from auto periodic to init
        armTelescopingSubsystem.init();
        armTelescopingSubsystem.telescopingMotor.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs); //VERY VERY IMPORTANT
    
        mechState = MechanismStates.HOLDING; 
    }

    public static enum MechanismStates{
        LOW_NODE,
        MID_NODE,
        HIGH_NODE,
        SHELF,
        HOLDING;
    }

    MechanismStates mechState = MechanismStates.HOLDING;

    public void periodic(){
        armTelescopingSubsystem.periodic();
        elevatorSubsystem.periodic();

        if (mechState == MechanismStates.LOW_NODE){
            elevatorSubsystem.setState(ElevatorStates.LOW_ELEVATOR_HEIGHT);
            armTelescopingSubsystem.setTState(TelescopingStates.LOW_ARM_LENGTH);
        } else if (mechState == MechanismStates.MID_NODE){
            elevatorSubsystem.setState(ElevatorStates.MID_ELEVATOR_HEIGHT);
            armTelescopingSubsystem.setTState(TelescopingStates.MID_ARM_LENGTH);
        } else if (mechState == MechanismStates.HIGH_NODE) {
            elevatorSubsystem.setState(ElevatorStates.HIGH_ELEVATOR_HEIGHT);
            armTelescopingSubsystem.setTState(TelescopingStates.HIGH_ARM_LENGTH);
        } else if (mechState == MechanismStates.SHELF){
            elevatorSubsystem.setState(ElevatorStates.SHELF_ELEVATOR_HEIGHT);
            armTelescopingSubsystem.setTState(TelescopingStates.SHELF_ARM_LENGTH);
        } else { 
            elevatorSubsystem.setState(ElevatorStates.ZERO);
            armTelescopingSubsystem.setTState(TelescopingStates.RETRACTED);

        }
    }

    public void setState(MechanismStates mechState){
        this.mechState = mechState; 
    }

    public boolean isDoneHigh(){
       if(elevatorSubsystem.isAtHigh() && armTelescopingSubsystem.isAtHigh()){
            return true;
       }
       return false; 
    }

}
