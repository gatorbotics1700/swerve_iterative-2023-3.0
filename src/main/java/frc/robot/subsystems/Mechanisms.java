package frc.robot.subsystems;

import frc.robot.subsystems.ArmTelescopingSubsystem;
import frc.robot.subsystems.ArmTelescopingSubsystem.TelescopingStates;
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

        } else if (mechState == MechanismStates.MID_NODE){

        } else if (mechState == MechanismStates.HIGH_NODE) {

        } else if (mechState == MechanismStates.SHELF){

        } else { //holding

        }
    }

}
