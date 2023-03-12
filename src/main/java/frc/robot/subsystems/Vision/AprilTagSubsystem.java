package frc.robot.subsystems.Vision;

import javax.swing.plaf.nimbus.State;

import edu.wpi.first.apriltag.AprilTagDetection;
import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

//import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
//import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.autonomous.AutonomousBasePD;
import frc.robot.autonomous.StateWithCoordinate;
import frc.robot.autonomous.StateWithCoordinate.AutoStates;
import frc.robot.subsystems.*;
import frc.robot.Buttons;

public class AprilTagSubsystem {
    public static enum AprilTagSequence{
        DETECT,
        CORRECTPOSITION,
        OFF;
    }

    private static AprilTagSequence states = AprilTagSequence.DETECT; 
    private static DrivetrainSubsystem drivetrainSubsystem = Robot.m_drivetrainSubsystem;

    public void setState(AprilTagSequence newState){
        states = newState;
    }
    
    private static LimeLightSubsystem limeLightSubsystem = new LimeLightSubsystem();
    private static AutonomousBasePD autonomousBasePD = new AutonomousBasePD();

    public void init(){
        limeLightSubsystem.setPipeline(1.0);
        autonomousBasePD.resetControllers();
        Robot.m_drivetrainSubsystem.resetOdometry(new Pose2d(AprilTagLocation.scoringPoses[4].getX() -36.5*Constants.METERS_PER_INCH, AprilTagLocation.scoringPoses[4].getY() - 12.0*Constants.METERS_PER_INCH, new Rotation2d(Math.toRadians(0.0))));
        System.out.println("resetted odometry in INIT to: " + DrivetrainSubsystem.m_pose);
    }
    
    public void periodic(){
        Robot.m_drivetrainSubsystem.drive();
        if(states == AprilTagSequence.DETECT){
            limeLightSubsystem.reset();
            if(LimeLightSubsystem.tv!=0){ //made private in limelightss but changed due to pull request- ask katherine!
                System.out.println("APRIL TAG DETECTED!!!!!!");
                setState(AprilTagSequence.CORRECTPOSITION);
                autonomousBasePD.resetControllers();
                AutonomousBasePD visionPID = new AutonomousBasePD(DrivetrainSubsystem.m_pose, new StateWithCoordinate[]{                    
                    new StateWithCoordinate(AutoStates.FIRST),
                    new StateWithCoordinate(AutoStates.DRIVE, AprilTagLocation.scoringPoses[0]),
                    new StateWithCoordinate(Buttons.level)
                });

                System.out.println("Reset odometry to this m_pose: " + DrivetrainSubsystem.m_pose);
                }
        } else if(states == AprilTagSequence.CORRECTPOSITION){
            System.out.println("Limelight tv: " + limeLightSubsystem.getTv());
        
            if(limeLightSubsystem.getTv() != 0.0){
                double[] tempArray = limeLightSubsystem.networkTable.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
                drivetrainSubsystem.resetOdometry(new Pose2d(tempArray[0], tempArray[1], new Rotation2d (Math.toRadians(tempArray[5]))));
                autonomousBasePD.periodic();

                if(autonomousBasePD.xAtSetpoint() && autonomousBasePD.yAtSetpoint() && autonomousBasePD.turnAtSetpoint()){
                    setState(AprilTagSequence.OFF);
                    Robot.m_drivetrainSubsystem.setSpeed(ChassisSpeeds.fromFieldRelativeSpeeds(0.0, 0.0, 0.0, Robot.m_drivetrainSubsystem.getPoseRotation()));
                    System.out.println("finished correcting position!!!!!");
                }     
            }else{
                System.out.println("don't see april tag");
                setState(AprilTagSequence.DETECT);
            }
            
        } else if(states == AprilTagSequence.OFF){
            System.out.println("in off");

        }       
    }
}