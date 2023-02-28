package frc.robot.subsystems;
import frc.robot.subsystems.*;
import frc.robot.subsystems.LimeLightSubsystem.LimelightStates;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.AprilTagSubsystem;
import frc.robot.*;

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
