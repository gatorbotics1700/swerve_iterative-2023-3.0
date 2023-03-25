package frc.robot.subsystems;

import frc.robot.subsystems.ArmPneumaticPivot.PneumaticPivotStates;
import frc.robot.subsystems.ArmTelescopingSubsystem.TelescopingStates;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorStates;
import frc.robot.subsystems.PneumaticIntakeSubsystem.PneumaticIntakeStates;

public class Mechanisms {
    
    public ArmTelescopingSubsystem armTelescopingSubsystem;
    public ElevatorSubsystem elevatorSubsystem;
    public PneumaticIntakeSubsystem pneumaticIntakeSubsystem;
    public ArmPneumaticPivot armPneumaticPivot;

    public static enum MechanismStates{
        LOW_NODE,
        MID_NODE,
        SHELF,
        GROUNDPICKUP,
        MANUAL_ELEVATOR,
        MANUAL_TELESCOPE,
        HOLDING;
    }

    private MechanismStates mechState;

    public Mechanisms(){
        armTelescopingSubsystem = new ArmTelescopingSubsystem();
        elevatorSubsystem = new ElevatorSubsystem();
        pneumaticIntakeSubsystem = new PneumaticIntakeSubsystem();
        armPneumaticPivot = new ArmPneumaticPivot();
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
            if(elevatorSubsystem.isAboveLow()){ //if going down, we need to retract first
                armTelescopingSubsystem.setTState(TelescopingStates.LOW_ARM_LENGTH);
                if(armTelescopingSubsystem.isAtLow()){
                    elevatorSubsystem.setState(ElevatorStates.LOW_ELEVATOR_HEIGHT);
                }
            } else { //if going up, we need to go up before extending out
                elevatorSubsystem.setState(ElevatorStates.LOW_ELEVATOR_HEIGHT);
                if(elevatorSubsystem.isAtLow()){
                    System.out.println("elevator is at low");
                    armTelescopingSubsystem.setTState(TelescopingStates.LOW_ARM_LENGTH);
                }
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
        } else { //holding
            elevatorSubsystem.setState(ElevatorStates.ZERO); 
            armTelescopingSubsystem.setTState(TelescopingStates.RETRACTED);
            // armPneumaticPivot.setState(PneumaticPivotStates.RELEASING);
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
