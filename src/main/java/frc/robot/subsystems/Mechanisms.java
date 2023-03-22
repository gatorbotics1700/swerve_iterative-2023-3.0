package frc.robot.subsystems;

import frc.robot.subsystems.ArmTelescopingSubsystem;
import frc.robot.subsystems.ArmTelescopingSubsystem.TelescopingStates;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorStates;
import com.ctre.phoenix.motorcontrol.ControlMode;
import frc.robot.Constants;

public class Mechanisms {
    
    public static ArmTelescopingSubsystem armTelescopingSubsystem;
    public static ElevatorSubsystem elevatorSubsystem;

    public Mechanisms(){
        armTelescopingSubsystem = new ArmTelescopingSubsystem();
        elevatorSubsystem = new ElevatorSubsystem();
        init();
    }

    public void init(){
        elevatorSubsystem.init();
        armTelescopingSubsystem.init();
    
        mechState = MechanismStates.HOLDING; 
    }

    public static enum MechanismStates{
        LOW_NODE,
        MID_NODE,
        HIGH_NODE,
        SHELF,
        GROUNDPICKUP,
        MANUAL_ELEVATOR,
        MANUAL_TELESCOPE,
        HOLDING;
    }

    MechanismStates mechState = MechanismStates.HOLDING;

    public void periodic(){
        if (mechState == MechanismStates.LOW_NODE){
            elevatorSubsystem.setState(ElevatorStates.LOW_ELEVATOR_HEIGHT);
            if(elevatorSubsystem.isAtLow()){
                System.out.println("elevator is at low");
                armTelescopingSubsystem.setTState(TelescopingStates.LOW_ARM_LENGTH);
            }
        } else if (mechState == MechanismStates.MID_NODE){
            elevatorSubsystem.setState(ElevatorStates.MID_ELEVATOR_HEIGHT);
            if(elevatorSubsystem.isAtMid()){
                System.out.println("elevator is at mid");
                armTelescopingSubsystem.setTState(TelescopingStates.MID_ARM_LENGTH);
            }
        } else if (mechState == MechanismStates.SHELF){
            elevatorSubsystem.setState(ElevatorStates.SHELF_ELEVATOR_HEIGHT);
            if(elevatorSubsystem.isAtShelf()){
              armTelescopingSubsystem.setTState(TelescopingStates.SHELF_ARM_LENGTH);
            }
        } else if(mechState == MechanismStates.GROUNDPICKUP){
            armTelescopingSubsystem.setTState(TelescopingStates.RETRACTED);
            if(armTelescopingSubsystem.isAtRetracted()){
             elevatorSubsystem.setState(ElevatorStates.ZERO);
            }
        } else if (mechState == MechanismStates.MANUAL_ELEVATOR){
            elevatorSubsystem.setState(ElevatorStates.MANUAL);
            System.out.println("Manual elevator");
        } else if (mechState == MechanismStates.MANUAL_TELESCOPE){
            armTelescopingSubsystem.setTState(TelescopingStates.MANUAL);
            System.out.println("manual telescope");
        } else { 
            elevatorSubsystem.setState(ElevatorStates.ZERO);
            armTelescopingSubsystem.setTState(TelescopingStates.RETRACTED);
        }

        armTelescopingSubsystem.periodic();
        elevatorSubsystem.periodic();
    }

    public void setState(MechanismStates mechState){
        this.mechState = mechState; 
    }

    public boolean isDoneMid(){
        if(elevatorSubsystem.isAtMid() && armTelescopingSubsystem.isAtMid()){
             return true;
        }
        return false; 
     }

     public boolean isDoneLow(){
        if(elevatorSubsystem.isAtLow() && armTelescopingSubsystem.isAtLow()){
             return true;
        }
        return false; 
     }

     public boolean isDoneShelf(){
        if(elevatorSubsystem.isAtShelf() && armTelescopingSubsystem.isAtShelf()){
            return true;
        }
        return false;
     }

     public void retractArmManually(){
        armTelescopingSubsystem.setTState(TelescopingStates.MANUAL);
    }


}
