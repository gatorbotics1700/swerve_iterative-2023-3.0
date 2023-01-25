package frc.robot.subsystems;
import org.opencv.imgproc.Imgproc;

import java.io.IOException;
import java.nio.file.Path;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSink;
import org.opencv.core.Mat;
import edu.wpi.first.apriltag.*;
import java.util.Arrays;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.apriltag.AprilTagFields;


//import frc.robot.subsystems.AprilTagFieldEnum;

public class AprilTagSubsystem {
    static{
        //Loader.load(opencv_java.class);
        // System.loadLibrary("opencv_java470");
        //System.load("C:\\Users\\Gatorbotics\\Downloads\\opencv\\build\\java\\x64\\opencv_java470.dll");
    }
    //System.setProperty("java.library.path","C:\\Users\\Gatorbotics\\Downloads\\opencv\\build\\java\\x64")
    //System.load("C:\\Users\\Gatorbotics\\Downloads\\opencv\\build\\java\\x64");
    
    /*public enum AprilTagFields { 
        k2023ChargedUp("2023-chargedup.json");
    }*/

    public static UsbCamera camera0;
    //public static VideoSink sink;
    private String family = "tag16h5";
    Mat source;
    Mat grayMat;
    CvSink cvSink;
    AprilTagDetection[] detectedAprilTags; 
    VideoSink server;
    int aprilTagIds[];
    float decisionMargin[];
    double centerX[];
    double centerY[];
    //Path path = Filesystem.getDefault().getPath("2023-chargedup.json");
    //public final String m_resourceFile;

    //CvSource outputStream;

    public static enum IDS{
        IDONE,
        IDTWO,
        IDTHREE,
        IDFOUR,
        IDFIVE,
        IDSIX,
        IDSEVEN,
        IDEIGHT
    }

    private static IDS iD;
    private static AprilTagDetector aprilTagDetector = new AprilTagDetector();
    //AprilTagFieldLayout aprilTagFieldLayout = new AprilTagFieldLayout("src/main/deploy/2023-chargedup.json");
    //AprilTagFieldLayout secondAprilTagFieldLayout = new AprilTagFieldLayout(AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile));
    

    public void init(){
        Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve("2023-chargedup.json");
        AprilTagFieldLayout thirdAprilTagFieldLayout;
        try{
            thirdAprilTagFieldLayout = new AprilTagFieldLayout(trajectoryPath);
        } catch(IOException e){
            System.out.println("Couldn't fine file");
        }
        System.loadLibrary("opencv_java460");
        //camera0 = new UsbCamera("USB Camera 0", 0);
        aprilTagDetector.addFamily(family, 0); //added 0
        camera0 = CameraServer.startAutomaticCapture(); //deleted 0
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
        
        detectedAprilTags = aprilTagDetector.detect(grayMat);

        

        for(int i = 0; i < detectedAprilTags.length; i++){
            aprilTagIds[i] = detectedAprilTags[i].getId();
            decisionMargin[i] = detectedAprilTags[i].getDecisionMargin();
            centerX[i] = detectedAprilTags[i].getCenterX();
            centerY[i] = detectedAprilTags[i].getCenterY();
        }

    

        System.out.println("Detected Apriltags: " + detectedAprilTags);


        if(iD == IDS.IDONE){

        }

        if(iD == IDS.IDTWO){

        }

        if(iD == IDS.IDTHREE){

        }

        if(iD == IDS.IDFOUR){

        }

        if(iD == IDS.IDFIVE){

        }

        if(iD == IDS.IDSIX){

        }

        if(iD ==IDS.IDSEVEN){

        }

        if(iD == IDS.IDEIGHT){

        }
    }

}
