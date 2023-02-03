package frc.robot.subsystems;
import org.opencv.imgproc.Imgproc;
import java.nio.file.Path;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.math.geometry.Pose2d;

import org.opencv.core.Mat;
import edu.wpi.first.apriltag.*;
import java.util.Arrays;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.apriltag.AprilTagFields;
import frc.robot.subsystems.*;
import frc.robot.Robot;
import frc.robot.autonomous.*;

//import frc.robot.subsystems.AprilTagFieldEnum;

public class AprilTagSubsystem {


    //System.setProperty("java.library.path","C:\\Users\\Gatorbotics\\Downloads\\opencv\\build\\java\\x64")
    //System.load("C:\\Users\\Gatorbotics\\Downloads\\opencv\\build\\java\\x64");
    
    /*public enum AprilTagFields { 
        k2023ChargedUp("2023-chargedup.json");
    }*/

    public static UsbCamera camera0;
    public static UsbCamera camera1;
    //public static VideoSink sink;
    private String family = "tag16h5";
    Mat source;
    Mat grayMat;
    CvSink cvSink;
    AprilTagDetection[] detectedAprilTagsArray;
    AprilTagDetection detectedAprilTag;
    VideoSink server;
    /*int aprilTagIds[];
    
    float decisionMargin[];
    double centerX[];
    double centerY[];*/
    double drivetrainXPosition;
    //Path path = Filesystem.getDefault().getPath("2023-chargedup.json");
    //public final String m_resourceFile;

    //CvSource outputStream;



    public static enum AprilTagSequence{
        DETECT,
        CORRECTPOSITION;
    }

    private static AprilTagSequence states = AprilTagSequence.DETECT; 

    public void setState(AprilTagSequence newState){
        states = newState;
    }
    
    private static AprilTagDetector aprilTagDetector = new AprilTagDetector();

    private static AutonomousBasePD autonomousBasePD = new AutonomousBasePD();
    //AprilTagFieldLayout aprilTagFieldLayout = new AprilTagFieldLayout("src/main/deploy/2023-chargedup.json");
    //AprilTagFieldLayout secondAprilTagFieldLayout = new AprilTagFieldLayout(AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile));
    //AprilTagFieldLayout thirdAprilTagFieldLayout = new AprilTagFieldLayout("/edu/wpi/first/apriltag/2023-chargedup.json");
    //Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectory);
    
    public void init(){
        System.loadLibrary("opencv_java460");
        //camera0 = new UsbCamera("USB Camera 0", 0);
        aprilTagDetector.addFamily(family, 0); //added 0
        camera0 = CameraServer.startAutomaticCapture(); //deleted 0
        camera1 = CameraServer.startAutomaticCapture();
        server = CameraServer.getServer();
        //sink = CameraServer.getServer();
        source = new Mat();
        grayMat = new Mat();
        

        System.out.println("source from init: " + source);
        //source.release();
        cvSink = CameraServer.getVideo();
        //outputStream = CameraServer.putVideo("camera stream", 320, 240);
    }
    
    public void periodic(){
        if(states == AprilTagSequence.DETECT){
            detectTag();
            setState(AprilTagSequence.CORRECTPOSITION);
        }else if(states == AprilTagSequence.CORRECTPOSITION){
            correctPosition();
        }
    }

    public void detectTag(){
        long time = cvSink.grabFrame(source);
        if(time ==0){
            System.out.println("failed to grab a frame");
            return;
        }
        Imgproc.cvtColor(source, grayMat,Imgproc.COLOR_BGR2GRAY);
        System.out.println("value of source in periodic: " + source);
        System.out.println(grayMat.empty());
        System.out.println(grayMat.width());
        System.out.println(grayMat.height());
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
        Pose2d prePose = autonomousBasePD.preDDD(DrivetrainSubsystem.m_pose, AprilTagLocation.aprilTagPoses[detectedAprilTag.getId()].toPose2d()); 
        autonomousBasePD.driveDesiredDistance(prePose);
    }
    

    /*private void getError(){
        drivetrainXPosition = Robot.m_drivetrainSubsystem.m_pose.getX();
        for(int i = 0; i < aprilTagIds.length; i++){
            XError[i] = AprilTagLocation.aprilTagPoses[aprilTagIds[i]].getX() - drivetrainXPosition;
            
            //AprilTagLocation
        }
    }*/
}
