// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.subsystems.*;
import frc.robot.OI;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.autonomous.*;
import frc.robot.Constants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.opencv.core.Core;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSink;
import org.opencv.core.Mat;
import edu.wpi.first.apriltag.*;
import java.util.Arrays;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
  //private final AutonomousBase autonomousBasePD = new AutonomousBasePD(20, -2, -3, 240);
  public static final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  public static final AprilTagSubsystem m_AprilTagSubsystem = new AprilTagSubsystem(); 

  static {System.loadLibrary(Core.NATIVE_LIBRARY_NAME);}
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  
  

  //Thread m_visionThread;
  @Override

  

  
  public void robotInit() {
    System.out.println("native library name" + Core.NATIVE_LIBRARY_NAME);
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    m_drivetrainSubsystem.zeroGyroscope();
    m_drivetrainSubsystem.zeroDriveEncoder();
    /*m_visionThread = 
      new Thread( 
        () -> {
          while(!Thread.interrupted)){
            //grab image from camera
            long time = cvSink.getFrame(mat);
            if(time == 0){
              continue; //error getting image
            }

            //convert image to grayscale
            Imgproc.cvtColor(mat, graymat, Imgproc.COLOR_BGR2GRAY);

            //run detection
            for (AprilTagDectection detection : detector.detect(graymat)){
              //filter by property

              //run pose estimator
              Transform3d pose = poseEstimator.estimate(detection);
            }
          }
        }
      )*/
    System.out.println("finished april tag init");
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
  }

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
   // autonomousBasePD.init();
    m_drivetrainSubsystem.zeroGyroscope();
    m_drivetrainSubsystem.zeroDriveEncoder();
    m_autoSelected = m_chooser.getSelected();
    m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    //System.out.println("Auto selected: " + m_autoSelected);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
    
    //autonomousBasePD.periodic();
    m_drivetrainSubsystem.drive();
  }



  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    //System.out.println("in teleop init");
    m_AprilTagSubsystem.init();
    
    // m_drivetrainSubsystem.zeroGyroscope();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    System.out.println("i am in teleop");
    m_AprilTagSubsystem.periodic();
    //m_AprilTagSubsystem.detectTag();

    
    // m_drivetrainSubsystem.driveTeleop();

    // if (OI.m_controller.getBButton()){
    //   m_drivetrainSubsystem.stopDrive();
    // }
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {
    //m_realAprilTagDetection.init();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    //m_realAprilTagDetection.periodic();
    System.out.println("i am in test");
    /*m_drivetrainSubsystem.setSpeed(ChassisSpeeds.fromFieldRelativeSpeeds(0.2, 0.3, 0.0, m_drivetrainSubsystem.getGyroscopeRotation()));
    m_drivetrainSubsystem.drive();
    System.out.println("distance: " + m_drivetrainSubsystem.getDistance());*/
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}

  


}
