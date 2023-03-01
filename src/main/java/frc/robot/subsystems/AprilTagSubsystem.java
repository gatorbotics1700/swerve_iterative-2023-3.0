package frc.robot.subsystems;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.apriltag.AprilTagDetection;
import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.apriltag.AprilTagPoseEstimator;

import edu.wpi.first.apriltag.AprilTagPoseEstimator.Config;
//import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
//import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;
import frc.robot.autonomous.AutonomousBasePD;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.subsystems.*;

//import frc.robot.subsystems.AprilTagFieldEnum;

public class AprilTagSubsystem {


    //System.setProperty("java.library.path","C:\\Users\\Gatorbotics\\Downloads\\opencv\\build\\java\\x64")
    //System.load("C:\\Users\\Gatorbotics\\Downloads\\opencv\\build\\java\\x64");
    
    /*public enum AprilTagFields { 
        k2023ChargedUp("2023-chargedup.json");
    }*/

    //public static UsbCamera camera0;
    //public static UsbCamera camera1;
    //public static VideoSink sink;
    private String family = "tag16h5";
    Mat source;
    Mat grayMat;
    //CvSink cvSink;
    AprilTagDetection[] detectedAprilTagsArray ={};
    AprilTagDetection detectedAprilTag;
    //VideoSink server;
    /*int aprilTagIds[];
    
    float decisionMargin[];
    double centerX[];
    double centerY[];*/
    double drivetrainXPosition;
    //Path path = Filesystem.getDefault().getPath("2023-chargedup.json");
    //public final String m_resourceFile;

    //CvSource outputStream;

    private static double tagSize = 4.0; //change to actual size(inches)
    private static double fx;//need to add value
    private static double fy;//need to add value
    private static double cx;//need to add value
    private static double cy;//need to add value
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
    private LimeLightSubsystem limeLightSubsystem = new LimeLightSubsystem();
    private static AutonomousBasePD autonomousBasePD = new AutonomousBasePD();

    public final AprilTagPoseEstimator.Config aprilTagPoseEstimatorConfig = new Config(tagSize, fx, fy, cx, cy);
    public AprilTagPoseEstimator aprilTagPoseEstimator = new AprilTagPoseEstimator(aprilTagPoseEstimatorConfig);
    //AprilTagFieldLayout aprilTagFieldLayout = new AprilTagFieldLayout("src/main/deploy/2023-chargedup.json");
    //AprilTagFieldLayout secondAprilTagFieldLayout = new AprilTagFieldLayout(AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile));
    //AprilTagFieldLayout thirdAprilTagFieldLayout = new AprilTagFieldLayout("/edu/wpi/first/apriltag/2023-chargedup.json");
    //Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectory);
    
    public void init(){
        Robot.m_drivetrainSubsystem.resetOdometry(new Pose2d(AprilTagLocation.scoringPoses[13].getX() + 12.0, AprilTagLocation.scoringPoses[13].getY() + 12.0, Robot.m_drivetrainSubsystem.getGyroscopeRotation()));
        System.loadLibrary("opencv_java460");
        //camera0 = new UsbCamera("USB Camera 0", 0);
        aprilTagDetector.addFamily(family, 0); //added 0
        //camera0 = CameraServer.startAutomaticCapture(); //deleted 0
        //camera1 = CameraServer.startAutomaticCapture();
        //server = CameraServer.getServer();
        //sink = CameraServer.getServer();
        source = new Mat();
        grayMat = new Mat();
        

        //System.out.println("source from init: " + source);
        //source.release();
        //cvSink = CameraServer.getVideo(camera0);
        //System.out.println("cvSink value before if statement: " + cvSink);
        /*if(cvSink == null){
            System.out.println("CvSink is null!!!!!!!!!!");
            return;
        }*/
        //System.out.println("cvSink value: " + cvSink);
        //outputStream = CameraServer.putVideo("camera stream", 320, 240);
    }
    

    //kaylin wrote this but probably did it wrong :)
    public void addVisionToOdometry(){
        Transform3d aprilTagError = aprilTagPoseEstimator.estimate(detectedAprilTag);//april tag pose estimator in Transform 3d
        Pose2d aprilTagPose2D = AprilTagLocation.aprilTagPoses[detectedAprilTag.getId()-1].toPose2d();//pose 2d of the actual april tag
        Rotation2d robotSubtractedAngle =  Rotation2d.fromDegrees(aprilTagPose2D.getRotation().getDegrees()-aprilTagError.getRotation().toRotation2d().getDegrees());//angle needed to create pose 2d of robot position, don't know if toRotatation2D converts Rotation3D properly
        Pose2d robotPose2DAprilTag = new Pose2d(aprilTagPose2D.getX()-aprilTagError.getX(), aprilTagPose2D.getY()-aprilTagError.getY(), robotSubtractedAngle);
        swerveDrivePoseEstimator.addVisionMeasurement(robotPose2DAprilTag, Timer.getFPGATimestamp());
    }
    
    public void periodic(){
        if(states == AprilTagSequence.DETECT){
            detectTag();
            if(detectedAprilTagsArray.length!=0){
                System.out.println("APRIL TAG DETECTED!!!!!!");
                setState(AprilTagSequence.CORRECTPOSITION);
                Robot.m_drivetrainSubsystem.resetOdometry(new Pose2d(35, 72, new Rotation2d(0))); //DELETE THIS WHEN DONE WITH TESTING
                System.out.println("Reset odometry to this m_pose: " + DrivetrainSubsystem.m_pose);
                prePose = autonomousBasePD.preDDD(DrivetrainSubsystem.m_pose, AprilTagLocation.scoringPoses[14]);
                }
        } else if(states == AprilTagSequence.CORRECTPOSITION){
            //addVisionToOdometry();
            double[] tempArray = limeLightSubsystem.networkTable.getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
            DrivetrainSubsystem.m_pose = new Pose2d (tempArray[0], tempArray[1], new Rotation2d(tempArray[5]));
            correctPosition();
            if(autonomousBasePD.getDistanceController().atSetpoint()){
                setState(AprilTagSequence.OFF);
            }
        }        
    }

    public void detectTag(){
        //long time = cvSink.grabFrame(source);
        // if(time ==0){
        //     System.out.println("failed to grab a frame");
        //     return;
        // }
        //Imgproc.cvtColor(source, grayMat,Imgproc.COLOR_BGR2GRAY);
        //outputStream.putFrame(source);
        detectedAprilTagsArray = aprilTagDetector.detect(grayMat);
        if(detectedAprilTagsArray.length == 0){
            return;
        } else {
            detectedAprilTag = detectedAprilTagsArray[0];
        }
        
        

        /*for(int i = 0; i < detectedAprilTags.length; i++){
            aprilTagIds[i] = detectedAprilTags[i].getId();
            decisionMargin[i] = detectedAprilTags[i].getDecisionMargin();
            centerX[i] = detectedAprilTags[i].getCenterX();
            centerY[i] = detectedAprilTags[i].getCenterY();
        }*/

        

        System.out.println("Detected Apriltag: " + detectedAprilTag.getId());

    }


    private void correctPosition(){
        System.out.println("I am correcting position!!!");
        autonomousBasePD.driveDesiredDistance(prePose);
        Robot.m_drivetrainSubsystem.drive();
    }
    

    /*private void getError(){
        drivetrainXPosition = Robot.m_drivetrainSubsystem.m_pose.getX();
        for(int i = 0; i < aprilTagIds.length; i++){
            XError[i] = AprilTagLocation.aprilTagPoses[aprilTagIds[i]].getX() - drivetrainXPosition;
            
            //AprilTagLocation
        }
    }*/
}
