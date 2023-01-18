package frc.robot.subsystems;

import org.opencv.imgproc.Imgproc;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSink;
import org.opencv.core.Mat;
import edu.wpi.first.apriltag.*;
import java.util.Arrays;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;

public class AprilTagSubsystem {
    static{
        //Loader.load(opencv_java.class);
        // System.loadLibrary("opencv_java470");
        //System.load("C:\\Users\\Gatorbotics\\Downloads\\opencv\\build\\java\\x64\\opencv_java470.dll");
    }
    //System.setProperty("java.library.path","C:\\Users\\Gatorbotics\\Downloads\\opencv\\build\\java\\x64")
    //System.load("C:\\Users\\Gatorbotics\\Downloads\\opencv\\build\\java\\x64");
    
    public static UsbCamera camera0;
    //public static VideoSink sink;
    private String family = "tag16h5";
    Mat source;
    Mat grayMat;
    CvSink cvSink;
    AprilTagDetection[] detectedAprilTags; 
    VideoSink server;
    //CvSource outputStream;

    private static AprilTagDetector aprilTagDetector = new AprilTagDetector();
    
    public void init(){
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

        
        System.out.println("Detected Apriltags: " + detectedAprilTags);

    }

}
