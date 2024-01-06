package frc.robot.subsystems;

import frc.robot.subsystems.PneumaticArmPivot.PneumaticPivotStates;
import frc.robot.subsystems.ArmTelescopingSubsystem.TelescopingStates;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorStates;
import frc.robot.subsystems.PneumaticIntakeSubsystem.PneumaticIntakeStates;
import frc.robot.subsystems.LauncherSubsystem;

public class Mechanisms {
    
    public ArmTelescopingSubsystem armTelescopingSubsystem;
    public ElevatorSubsystem elevatorSubsystem;
    public PneumaticIntakeSubsystem pneumaticIntakeSubsystem;
    public PneumaticArmPivot armPneumaticPivot;
    public LauncherSubsystem launcherSubsystem;

    public static enum MechanismStates{
        LOW_NODE,
        MID_NODE,
        SHELF,
        SUB,
        GROUNDPICKUP,
        AUTO_STARTING,
        MANUAL_ELEVATOR,
        MANUAL_TELESCOPE,
        HOLDING;
    }

    private MechanismStates mechState;

    public Mechanisms(){
        armTelescopingSubsystem = new ArmTelescopingSubsystem();
        elevatorSubsystem = new ElevatorSubsystem();
        pneumaticIntakeSubsystem = new PneumaticIntakeSubsystem();
        armPneumaticPivot = new PneumaticArmPivot();
        launcherSubsystem = new LauncherSubsystem();
        init();
    }

    public void init(){       
        launcherSubsystem.init();
    }

    public void periodic(){
        launcherSubsystem.periodic();
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
