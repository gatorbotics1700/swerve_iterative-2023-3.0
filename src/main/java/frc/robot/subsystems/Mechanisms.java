package frc.robot.subsystems;

import frc.robot.subsystems.LauncherSubsystem;

public class Mechanisms {
    
    public LauncherSubsystem launcherSubsystem;

    public static enum MechanismStates{
    }

    private MechanismStates mechState;

    public Mechanisms(){
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
}
