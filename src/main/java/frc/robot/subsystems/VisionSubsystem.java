package frc.robot.subsystems;

public class VisionSubsystem {
    public enum VisionStates{
        DETECTAPRILTAG,
        DETECTTAPE,
        OFF;
    }

    public VisionStates visionState = VisionStates.OFF; 
    public void setState(VisionStates state){
        visionState = state;
    }

    public void init(){
        
    }
    
    public void periodic(){
        if(visionState == VisionStates.DETECTAPRILTAG){
            
        }
    }

}
