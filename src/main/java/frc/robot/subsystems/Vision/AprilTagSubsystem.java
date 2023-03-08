package frc.robot.subsystems.Vision;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.apriltag.AprilTagDetection;
import edu.wpi.first.apriltag.AprilTagDetector;

//import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
//import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.autonomous.AutonomousBasePD;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.subsystems.*;

//import frc.robot.subsystems.AprilTagFieldEnum;

public class AprilTagSubsystem {
    private String family = "tag16h5";
    AprilTagDetection[] detectedAprilTagsArray ={};
    AprilTagDetection detectedAprilTag;
    double drivetrainXPosition;
    private static SwerveDrivePoseEstimator swerveDrivePoseEstimator = new SwerveDrivePoseEstimator(Robot.m_drivetrainSubsystem.m_kinematics, Robot.m_drivetrainSubsystem.getGyroscopeRotation(), new SwerveModulePosition[] {Robot.m_drivetrainSubsystem.m_frontLeftModule.getSwerveModulePosition(),Robot.m_drivetrainSubsystem.m_frontRightModule.getSwerveModulePosition(), Robot.m_drivetrainSubsystem.m_backRightModule.getSwerveModulePosition(), Robot.m_drivetrainSubsystem.m_backLeftModule.getSwerveModulePosition()}, new Pose2d(0.0, 0.0,Robot.m_drivetrainSubsystem.getGyroscopeRotation())); //what arguments go here?
    private static Pose2d prePose;

    public static enum AprilTagSequence{
        DETECT,
        CORRECTPOSITION,
        OFF;
    }

    private static AprilTagSequence states = AprilTagSequence.DETECT; 

    public void setState(AprilTagSequence newState){
        states = newState;
    }
    
    private static AprilTagDetector aprilTagDetector = new AprilTagDetector();
    private static LimeLightSubsystem limeLightSubsystem = new LimeLightSubsystem();
    private static AutonomousBasePD autonomousBasePD = new AutonomousBasePD();

  
    
    public void init(){
        limeLightSubsystem.setPipeline(1.0);
        Robot.m_drivetrainSubsystem.resetOdometry(new Pose2d(AprilTagLocation.scoringPoses[4].getX() -36.5*Constants.METERS_PER_INCH, AprilTagLocation.scoringPoses[4].getY() - 12.0*Constants.METERS_PER_INCH, new Rotation2d(Math.toRadians(0.0))));
        System.out.println("resetted odometry in INIT to: " + DrivetrainSubsystem.m_pose);
        System.loadLibrary("opencv_java460");
        aprilTagDetector.addFamily(family, 0); //added 0
    }
    
    public void periodic(){
        if(states == AprilTagSequence.DETECT){
            limeLightSubsystem.reset();
            if(LimeLightSubsystem.tv!=0){ //made private in limelightss but changed due to pull request- ask katherine!
                System.out.println("APRIL TAG DETECTED!!!!!!");
                setState(AprilTagSequence.CORRECTPOSITION);
                //Robot.m_drivetrainSubsystem.resetOdometry(new Pose2d(35, 72, new Rotation2d(0))); //DELETE THIS WHEN DONE WITH TESTING
                System.out.println("Reset odometry to this m_pose: " + DrivetrainSubsystem.m_pose);
                }
        } else if(states == AprilTagSequence.CORRECTPOSITION){
            double[] tempArray = limeLightSubsystem.networkTable.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
            DrivetrainSubsystem.m_pose = new Pose2d (tempArray[0], tempArray[1], new Rotation2d(Math.toRadians(tempArray[5])));
            correctPosition();
            if(autonomousBasePD.xAtSetpoint() && autonomousBasePD.yAtSetpoint() && autonomousBasePD.turnAtSetpoint()){
                setState(AprilTagSequence.OFF);
                System.out.println("finished correcting position!!!!!");
            }
        }        
    }

    private void correctPosition(){
        //System.out.println("I am correcting position!!!");
        autonomousBasePD.driveDesiredDistance(AprilTagLocation.scoringPoses[1]);
        Robot.m_drivetrainSubsystem.drive();
    }
}
