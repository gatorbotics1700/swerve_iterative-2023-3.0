package frc.robot.subsystems;

import frc.robot.subsystems.ArmTelescopingSubsystem;
import frc.robot.subsystems.ArmPneumaticPivot.PneumaticPivotStates;
import frc.robot.subsystems.ArmTelescopingSubsystem.TelescopingStates;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorStates;

import com.ctre.phoenix.motorcontrol.ControlMode;

import frc.robot.Constants;

public class Mechanisms {
    
    public static ArmTelescopingSubsystem armTelescopingSubsystem = new ArmTelescopingSubsystem();
    public static ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
    public static ArmPneumaticPivot armPneumaticPivot = new ArmPneumaticPivot();

    public void init(){
        //elevator
        elevatorSubsystem.init();

        //telescope
        armTelescopingSubsystem.setTState(TelescopingStates.RETRACTED); //moved from auto periodic to init
        armTelescopingSubsystem.init();
        //armTelescopingSubsystem.telescopingMotor.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs); //VERY VERY IMPORTANT
    
        armPneumaticPivot.init();
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
        armTelescopingSubsystem.periodic();
        elevatorSubsystem.periodic();
        armPneumaticPivot.periodic();

        if (mechState == MechanismStates.LOW_NODE){
            elevatorSubsystem.setState(ElevatorStates.LOW_ELEVATOR_HEIGHT);
            armPneumaticPivot.setState(PneumaticPivotStates.RETRACTING);
            if(elevatorSubsystem.isAtLow()){
                System.out.println("elevator is at low");
                armTelescopingSubsystem.setTState(TelescopingStates.LOW_ARM_LENGTH);
            }
        } else if (mechState == MechanismStates.MID_NODE){
            elevatorSubsystem.setState(ElevatorStates.MID_ELEVATOR_HEIGHT);
            armPneumaticPivot.setState(PneumaticPivotStates.RETRACTING);
            if(elevatorSubsystem.isAtMid()){
                System.out.println("elevator is at mid");
                armTelescopingSubsystem.setTState(TelescopingStates.MID_ARM_LENGTH);
            }
        } else if (mechState == MechanismStates.HIGH_NODE) {
            elevatorSubsystem.setState(ElevatorStates.HIGH_ELEVATOR_HEIGHT);
            armPneumaticPivot.setState(PneumaticPivotStates.RETRACTING);
            if(elevatorSubsystem.isAtHigh()){
                System.out.println("elevator is at high");
                armTelescopingSubsystem.setTState(TelescopingStates.HIGH_ARM_LENGTH);
            }
        } else if (mechState == MechanismStates.SHELF){
            elevatorSubsystem.setState(ElevatorStates.SHELF_ELEVATOR_HEIGHT);
            armPneumaticPivot.setState(PneumaticPivotStates.RETRACTING);
            if(elevatorSubsystem.isAtShelf()){
              armTelescopingSubsystem.setTState(TelescopingStates.SHELF_ARM_LENGTH);
            }
        } else if(mechState == MechanismStates.GROUNDPICKUP){
            armTelescopingSubsystem.setTState(TelescopingStates.RETRACTED);
            if(armTelescopingSubsystem.isAtZero()){
             elevatorSubsystem.setState(ElevatorStates.ZERO);
            }
        } else if (mechState == MechanismStates.MANUAL_ELEVATOR){
            elevatorSubsystem.setState(ElevatorStates.MANUAL);
        } else if (mechState == MechanismStates.MANUAL_TELESCOPE){
            armTelescopingSubsystem.setTState(TelescopingStates.MANUAL);
        } else { 
            armPneumaticPivot.setState(PneumaticPivotStates.ACTUATING);
            elevatorSubsystem.setState(ElevatorStates.STOPPED);
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

     public void retract(){
        //armTelescopingSubsystem.setTState(TelescopingStates.RETRACTED);
        armTelescopingSubsystem.telescopingMotor.set(ControlMode.PercentOutput, 0.2);
        System.out.println("retracting + position: " + armTelescopingSubsystem.telescopingMotor.getSelectedSensorPosition());
    }


}
