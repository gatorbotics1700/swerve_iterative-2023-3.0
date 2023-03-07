package frc.robot.subsystems.Vision;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Vision.LimeLightSubsystem.LimelightStates;

public class VisionSubsystem {
    public static enum VisionStates{
        DETECTAPRILTAG,
        DETECTTAPE,
        OFF;
    }

    public VisionStates visionState = VisionStates.OFF; 

    public LimeLightSubsystem limeLightSubsystem = new LimeLightSubsystem(); 
    public AprilTagSubsystem aprilTagSubsystem = new AprilTagSubsystem();
    public static int level;
    
    public void setState(VisionStates state){
        visionState = state;
    }

    public void init(){
    }
    
    public void periodic(){
        if(visionState == VisionStates.DETECTAPRILTAG){
            limeLightSubsystem.setPipeline(0.0);
            if(LimeLightSubsystem.tv == 0){
                return;
            }
            aprilTagSubsystem.setState(AprilTagSubsystem.AprilTagSequence.DETECT);
            setState(VisionStates.OFF);
        }

        if(visionState == VisionStates.DETECTTAPE){
            limeLightSubsystem.setPipeline(2.0);
            if(LimeLightSubsystem.tv == 0){
                limeLightSubsystem.setPipeline(1.0);
                if(LimeLightSubsystem.tv == 0){
                    return;
                }
            }
            limeLightSubsystem.setState(LimelightStates.SCANTAPE);
        }
    }

}
