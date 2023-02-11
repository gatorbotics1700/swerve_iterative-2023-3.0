package frc.robot.subsystems;
import frc.robot.subsystems.*;

public class VisionSubsystem {
    public enum VisionStates{
        DETECTAPRILTAG,
        DETECTTAPE,
        OFF;
    }

    public VisionStates visionState = VisionStates.OFF; 

    public LimeLightSubsystem limeLightSubsystem = new LimeLightSubsystem(); 
    public AprilTagSubsystem aprilTagSubsystem = new AprilTagSubsystem();
    
    public void setState(VisionStates state){
        visionState = state;
    }

    public void init(){
        
    }
    
    public void periodic(){
        if(visionState == VisionStates.DETECTAPRILTAG){
            limeLightSubsystem.setPipeline(0);
            aprilTagSubsystem.setState(AprilTagSequence.DETECT);
            setState(OFF);
        }

        if(visionState == VisionStates.DETECTTAPE){
            limeLightSubsystem.setPipeline(1);
        }
    }

}
