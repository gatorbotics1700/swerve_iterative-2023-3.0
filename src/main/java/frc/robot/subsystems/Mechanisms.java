package frc.robot.subsystems;

import frc.robot.subsystems.PneumaticArmPivot.PneumaticPivotStates;
import frc.robot.subsystems.ArmTelescopingSubsystem.TelescopingStates;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorStates;
import frc.robot.subsystems.PneumaticIntakeSubsystem.PneumaticIntakeStates;

public class Mechanisms {
    
    public ArmTelescopingSubsystem armTelescopingSubsystem;
    public ElevatorSubsystem elevatorSubsystem;
    public PneumaticIntakeSubsystem pneumaticIntakeSubsystem;
    public PneumaticArmPivot armPneumaticPivot;

    public static enum MechanismStates{
        LOW_NODE,
        MID_NODE,
        SHELF,
        SUB,
        GROUNDPICKUP,
        MANUAL_ELEVATOR,
        MANUAL_TELESCOPE,
        HOLDING,
        STOP;
    }

    private MechanismStates mechState;

    public Mechanisms(){
        armTelescopingSubsystem = new ArmTelescopingSubsystem();
        elevatorSubsystem = new ElevatorSubsystem();
        pneumaticIntakeSubsystem = new PneumaticIntakeSubsystem();
        armPneumaticPivot = new PneumaticArmPivot();
        init();
    }

    public void init(){
        elevatorSubsystem.init();
        armTelescopingSubsystem.init();
        pneumaticIntakeSubsystem.init();
        armPneumaticPivot.init();
    
        mechState = MechanismStates.HOLDING; 
    }

    public void periodic(){
        if (mechState == MechanismStates.LOW_NODE){ 
            armTelescopingSubsystem.setTState(TelescopingStates.LOW_ARM_LENGTH);
            if(armTelescopingSubsystem.isAtLow()){
                elevatorSubsystem.setState(ElevatorStates.LOW_ELEVATOR_HEIGHT);
                armPneumaticPivot.setState(PneumaticPivotStates.DOWN);
            }
        } else if (mechState == MechanismStates.MID_NODE){
            elevatorSubsystem.setState(ElevatorStates.MID_ELEVATOR_HEIGHT);
            if(elevatorSubsystem.isAtMid()){
                //System.out.println("elevator is at mid");
                armTelescopingSubsystem.setTState(TelescopingStates.MID_ARM_LENGTH);
                armPneumaticPivot.setState(PneumaticPivotStates.DOWN);
            }
        } else if (mechState == MechanismStates.SHELF){
            elevatorSubsystem.setState(ElevatorStates.SHELF_ELEVATOR_HEIGHT);
            if(elevatorSubsystem.isAtShelf()){
              armTelescopingSubsystem.setTState(TelescopingStates.SHELF_ARM_LENGTH);
              armPneumaticPivot.setState(PneumaticPivotStates.DOWN);
            }
        } else if(mechState == MechanismStates.SUB){
            armTelescopingSubsystem.setTState(TelescopingStates.SINGLE_SUBSTATION);
            armPneumaticPivot.setState(PneumaticPivotStates.UP); 
            if (armTelescopingSubsystem.isAtSub()){
                elevatorSubsystem.setState(ElevatorStates.ZERO);
            }
        /*else if(mechState == MechanismStates.GROUNDPICKUP){
            armTelescopingSubsystem.setTState(TelescopingStates.RETRACTED);
            if(armTelescopingSubsystem.isAtRetracted()){
             elevatorSubsystem.setState(ElevatorStates.ZERO);
             armPneumaticPivot.setState(PneumaticPivotStates.DOWN);
            }
        }*/
        }else if (mechState == MechanismStates.MANUAL_ELEVATOR){
            elevatorSubsystem.setState(ElevatorStates.MANUAL);
            System.out.println("Manual elevator");
        } else if (mechState == MechanismStates.MANUAL_TELESCOPE){
            armTelescopingSubsystem.setTState(TelescopingStates.MANUAL);
            System.out.println("manual telescope");
        } else if (mechState == MechanismStates.STOP){
            elevatorSubsystem.setState(ElevatorStates.STOPPED);
            armTelescopingSubsystem.setTState(TelescopingStates.STOP);
            armPneumaticPivot.setState(PneumaticPivotStates.UP);
        } else { //holding
            elevatorSubsystem.setState(ElevatorStates.ZERO); 
            armTelescopingSubsystem.setTState(TelescopingStates.RETRACTED);
            armPneumaticPivot.setState(PneumaticPivotStates.UP);
        }
        armTelescopingSubsystem.periodic();
        elevatorSubsystem.periodic();
        pneumaticIntakeSubsystem.periodic();
        armPneumaticPivot.periodic();
    }

    public void setState(MechanismStates mechState){
        this.mechState = mechState; 
    }

    public boolean isDoneMid(){
        return elevatorSubsystem.isAtMid() && armTelescopingSubsystem.isAtMid();
     }

     public boolean isDoneLow(){
        return elevatorSubsystem.isAtLow() && armTelescopingSubsystem.isAtLow();
     }

     public boolean isDoneShelf(){
        return elevatorSubsystem.isAtShelf() && armTelescopingSubsystem.isAtShelf();
     }
}
