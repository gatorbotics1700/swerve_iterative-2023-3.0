// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.subsystems.*;
import frc.robot.OI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.autonomous.*;
import frc.robot.Constants;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTable.*;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.ControlMode;
//Sam's Imports Below This
/**import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
//SAM below this
import com.revrobotics.ColorSensorV3; */

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */


//private AutonomousBasePD mScore = new AutonomousBasePD(new Translation2d(222.037, 0), new Translation2d(135.091, -41.307), new Translation2d(0, -44.163), new Translation2d(222.894, -50.377), new Translation2d(0, -65.388), new Translation2d(0, -65.388));

public class Robot extends TimedRobot {
  private AutonomousBase m_autoSelected;
  private final SendableChooser<AutonomousBase> m_chooser = new SendableChooser<AutonomousBase>();
  public static final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem(); //if anything breaks in the future it might be this
  private final Field2d m_field = new Field2d();
  double t= 0.0;
  ChassisSpeeds m_ChassisSpeeds;
  double mpi = Constants.METERS_PER_INCH;

  /*private AutonomousBasePD noGo = new AutonomousBasePD(new Pose2d(0, 0, new Rotation2d(0)), new Pose2d(0, 0, new Rotation2d(0)), new Pose2d(0, 0, new Rotation2d(0)), new Pose2d(0,0, new Rotation2d(0)), new Pose2d(0, 0, new Rotation2d(0)), new Pose2d(0,0, new Rotation2d(0)), new Pose2d(0,0, new Rotation2d(0)));
  private AutonomousBasePD placeNLeave = new AutonomousBasePD(new Pose2d(0, 0, new Rotation2d(0)), new Pose2d(160.0, 0, new Rotation2d(0)), new Pose2d(160.0, 0, new Rotation2d(0)), new Pose2d(160.0, 0, new Rotation2d(0)), new Pose2d(160.0, 0, new Rotation2d(0)), new Pose2d(160.0, 0, new Rotation2d(0)), new Pose2d(160.0, 0, new Rotation2d(0)));
  private AutonomousBasePD antiCharge = new AutonomousBasePD(new Pose2d(86.840, -45.282, new Rotation2d(0)), new Pose2d(221.978, 19.463, new Rotation2d(0)), new Pose2d(135.091, -19.421, new Rotation2d(0)), new Pose2d(0, -22.277, new Rotation2d(0)), new Pose2d(222.491, -28.492, new Rotation2d(0)), new Pose2d(0, -43.502, new Rotation2d(0)), new Pose2d(0, -43.502, new Rotation2d(0)));
  private AutonomousBasePD antiChargeOpposite = new AutonomousBasePD(new Pose2d(86.840, 45.282, new Rotation2d(0)), new Pose2d(221.978, -19.463, new Rotation2d(0)), new Pose2d(135.091, 19.421, new Rotation2d(0)), new Pose2d(0, 22.277, new Rotation2d(0)), new Pose2d(222.491, 28.492, new Rotation2d(0)), new Pose2d(0, 43.502, new Rotation2d(0)), new Pose2d(0, 43.502, new Rotation2d(0)));
  private AutonomousBasePD engageCharge = new AutonomousBasePD(new Pose2d(97.759, 0, new Rotation2d(0)), new Pose2d(97.759, 0, new Rotation2d(0)), new Pose2d(97.759, 0, new Rotation2d(0)), new Pose2d(97.759, 0, new Rotation2d(0)), new Pose2d(97.759, 0, new Rotation2d(0)), new Pose2d(97.759, 0, new Rotation2d(0)), new Pose2d(97.759, 0, new Rotation2d(0)));
  private AutonomousBasePD placeTwoEngage = new AutonomousBasePD(new Pose2d(223.014, 16.468, new Rotation2d(0)), new Pose2d(0, 22.683, new Rotation2d(0)), new Pose2d(135.615, 25.539, new Rotation2d(0)), new Pose2d(222.191, 64.230, new Rotation2d(0)), new Pose2d(97.711, 64.230, new Rotation2d(0)), new Pose2d(97.711, 64.230, new Rotation2d(0)), new Pose2d(97.711, 64.230, new Rotation2d(0))); */

  // whole field: 651.683 (inches)
  // center : 325.8415 (inches)
  // private AutonomousBasePD noGo = new AutonomousBasePD(new Pose2d(0, 0, new Rotation2d(0)), new Pose2d(0, 0, new Rotation2d(0)), new Pose2d(0, 0, new Rotation2d(0)), new Pose2d(0,0, new Rotation2d(0)), new Pose2d(0, 0, new Rotation2d(0)), new Pose2d(0,0, new Rotation2d(0)), new Pose2d(0,0, new Rotation2d(0)));
  private AutonomousBaseTimed timedPath = new AutonomousBaseTimed();
  private AutonomousBasePD testPath = new AutonomousBasePD(new Pose2d(0, 0, new Rotation2d(0)), new Pose2d(20 * mpi, 40 * mpi, new Rotation2d(Math.toRadians(90))), new Pose2d(40 * mpi, 0, new Rotation2d(0)), new Pose2d(0, 30 * mpi, new Rotation2d(0)), new Pose2d(40 * mpi, 30 * mpi, new Rotation2d(0)), new Pose2d(0, 0, new Rotation2d(0)), new Pose2d(20 * mpi, 20 * mpi, new Rotation2d(0)), 180.0); 
  
  // blue alliance 
  // private AutonomousBasePD HDplaceTwoEngageB = new AutonomousBasePD(new Pose2d(56.069 * mpi, 20.19 * mpi,  new Rotation2d(0)), new Pose2d(278.999 * mpi, 36.19 * mpi, new Rotation2d(0)), new Pose2d(257.650 * mpi, 64.004 * mpi, new Rotation2d(0)), new Pose2d(156.859 * mpi, 83.368 * mpi, new Rotation2d(0)), new Pose2d(156.859 * mpi, 83.368 * mpi, new Rotation2d(0)), new Pose2d(156.859 * mpi, 83.368 * mpi, new Rotation2d(0)), new Pose2d(156.859 * mpi, 83.368 * mpi, new Rotation2d(0)));
  // private AutonomousBasePD HBLeaveB = new AutonomousBasePD(new Pose2d(56.069 * mpi, 200.046 * mpi, new Rotation2d(0)), new Pose2d(219.915 * mpi, 200.046 * mpi, new Rotation2d(0)), new Pose2d(219.915 * mpi, 200.046 * mpi, new Rotation2d(0)), new Pose2d(219.915 * mpi, 200.046 * mpi, new Rotation2d(0)), new Pose2d(219.915 * mpi, 200.046 * mpi, new Rotation2d(0)), new Pose2d(219.915 * mpi, 200.046 * mpi, new Rotation2d(0)), new Pose2d(219.915 * mpi, 200.046 * mpi, new Rotation2d(0)));
  // private AutonomousBasePD HDLeaveB = new AutonomousBasePD(new Pose2d(56.069 * mpi, 20.19 * mpi, new Rotation2d(0)), new Pose2d(219.91 * mpi, 20.19 * mpi, new Rotation2d(0)), new Pose2d(219.915 * mpi, 20.19 * mpi, new Rotation2d(0)), new Pose2d(219.915 * mpi, 20.19 * mpi, new Rotation2d(0)), new Pose2d(219.915 * mpi, 20.19 * mpi, new Rotation2d(0)), new Pose2d(219.915 * mpi, 20.19 * mpi, new Rotation2d(0)), new Pose2d(219.915 * mpi, 20.19 * mpi, new Rotation2d(0)));
  // private AutonomousBasePD HDThreeScoreB = new AutonomousBasePD(new Pose2d(56.069 * mpi, 20.19 * mpi, new Rotation2d(0)), new Pose2d(278.999 * mpi, 37.193 * mpi, new Rotation2d(0)), new Pose2d(56.222 * mpi, 43.068 * mpi, new Rotation2d(0)), new Pose2d(197.484 * mpi,45.934 * mpi, new Rotation2d(0)), new Pose2d(279.077 * mpi, 85.622 * mpi, new Rotation2d(0)), new Pose2d(197.484 * mpi,40.000 * mpi, new Rotation2d(0)), new Pose2d(56.154 * mpi,66.117 * mpi, new Rotation2d(0)));
  // private AutonomousBasePD HBThreeScoreB = new AutonomousBasePD(new Pose2d(56.069 * mpi, 200.046 * mpi, new Rotation2d(0)), new Pose2d(278.999 * mpi, 180.683 * mpi, new Rotation2d(0)), new Pose2d(56.069 * mpi, 174.725 * mpi, new Rotation2d(0)), new Pose2d(207.006 * mpi, 174.725 * mpi, new Rotation2d(0)), new Pose2d(278.006 * mpi, 133.515 * mpi, new Rotation2d(0)), new Pose2d(200.552 * mpi, 185.151 * mpi, new Rotation2d(0)), new Pose2d(57.062 * mpi, 154.368 * mpi, new Rotation2d(0)));
  // private AutonomousBasePD engageChargeB = new AutonomousBasePD(new Pose2d(56.069 * mpi, 108.015 * mpi, new Rotation2d(0)), new Pose2d(152.812 * mpi, 108.015 * mpi, new Rotation2d(0)), new Pose2d(152.812 * mpi, 108.015 * mpi, new Rotation2d(0)), new Pose2d(152.812 * mpi, 108.015 * mpi, new Rotation2d(0)), new Pose2d(152.812 * mpi, 108.015 * mpi, new Rotation2d(0)), new Pose2d(152.812 * mpi, 108.015 * mpi, new Rotation2d(0)), new Pose2d(152.812 * mpi, 108.015 * mpi, new Rotation2d(0)));
  //these paths score 3 balls without touching the charge station, requires 7 Pose2ds!
  private AutonomousBasePD threeUnderChargeStation = new AutonomousBasePD(new Pose2d(56.069, 17.332, new Rotation2d(0)), new Pose2d(278.999, 37.193, new Rotation2d(0)), new Pose2d(56.222, 43.068, new Rotation2d(0)), new Pose2d(197.484,45.934, new Rotation2d(0)), new Pose2d(279.077, 85.622, new Rotation2d(0)), new Pose2d(197.484,40.000, new Rotation2d(0)), new Pose2d(56.154,66.117, new Rotation2d(0)), 0);
  private AutonomousBasePD threeAboveChargeStation = new AutonomousBasePD(new Pose2d(56.069, 200.046, new Rotation2d(0)), new Pose2d(278.999, 180.683, new Rotation2d(0)), new Pose2d(56.069, 174.725, new Rotation2d(0)), new Pose2d(207.006, 174.725, new Rotation2d(0)), new Pose2d(278.006, 133.515, new Rotation2d(0)), new Pose2d(200.552, 185.151, new Rotation2d(0)), new Pose2d(57.062, 154.368, new Rotation2d(0)),0);

// red alliance  
// half the field (325.8415) - blue x value + half the field (325.8415) = red x value
  
  private AutonomousBasePD HDplaceTwoEngageR = new AutonomousBasePD(new Pose2d(595.614 * mpi, 20.19 * mpi, new Rotation2d(0)), new Pose2d(372.684 * mpi, 36.19 * mpi, new Rotation2d(0)), new Pose2d(394.033 * mpi, 64.004 * mpi, new Rotation2d(0)), new Pose2d(494.824 * mpi, 83.368 * mpi, new Rotation2d(0)), new Pose2d(494.824 * mpi, 83.368 * mpi, new Rotation2d(0)), new Pose2d(494.824 * mpi, 83.368 * mpi, new Rotation2d(0)), new Pose2d(494.824 * mpi, 83.368 * mpi, new Rotation2d(0)), 0);
   //elise added a goal angle to make the code happy
  // private AutonomousBasePD HBLeaveR = new AutonomousBasePD(new Pose2d(595.614 * mpi, 200.046 * mpi, new Rotation2d(0)), new Pose2d(431.768 * mpi, 200.04 * mpi, new Rotation2d(0)), new Pose2d(431.768 * mpi, 200.046 * mpi, new Rotation2d(0)), new Pose2d(431.768 * mpi, 200.046 * mpi, new Rotation2d(0)), new Pose2d(431.768 * mpi, 200.046 * mpi, new Rotation2d(0)), new Pose2d(431.768 * mpi, 200.046 * mpi, new Rotation2d(0)), new Pose2d(431.768 * mpi, 200.046 * mpi, new Rotation2d(0)));
  // private AutonomousBasePD HDLeaveR = new AutonomousBasePD(new Pose2d(595.614 * mpi, 20.19 * mpi, new Rotation2d(0)), new Pose2d(431.768 * mpi, 20.19 * mpi, new Rotation2d(0)), new Pose2d(431.768 * mpi, 20.19 * mpi, new Rotation2d(0)), new Pose2d(431.768 * mpi, 20.19 * mpi, new Rotation2d(0)), new Pose2d(431.768 * mpi, 20.19 * mpi, new Rotation2d(0)), new Pose2d(431.768 * mpi, 20.19 * mpi, new Rotation2d(0)), new Pose2d(431.768 * mpi, 20.19 * mpi, new Rotation2d(0)));
  // private AutonomousBasePD HDThreeScoreR = new AutonomousBasePD(new Pose2d(595.614 * mpi, 20.19 * mpi, new Rotation2d(0)), new Pose2d(372.684 * mpi, 37.193 * mpi, new Rotation2d(0)), new Pose2d(595.461 * mpi, 43.068 * mpi, new Rotation2d(0)), new Pose2d(454.199 * mpi,45.934 * mpi, new Rotation2d(0)), new Pose2d(372.606, 85.622 * mpi, new Rotation2d(0)), new Pose2d(454.199 * mpi,40.000 * mpi, new Rotation2d(0)), new Pose2d(595.529 * mpi,66.117 * mpi, new Rotation2d(0)));
  // private AutonomousBasePD HBThreeScoreR = new AutonomousBasePD(new Pose2d(595.614 * mpi, 200.046 * mpi, new Rotation2d(0)), new Pose2d(372.684 * mpi, 180.683 * mpi, new Rotation2d(0)), new Pose2d(595.614 * mpi, 174.725 * mpi, new Rotation2d(0)), new Pose2d(444.677 * mpi, 174.725 * mpi, new Rotation2d(0)), new Pose2d(373.677 * mpi, 133.515 * mpi, new Rotation2d(0)), new Pose2d(451.131 * mpi, 185.151 * mpi, new Rotation2d(0)), new Pose2d(594.621 * mpi, 154.368 * mpi, new Rotation2d(0)));
  // private AutonomousBasePD engageChargeR = new AutonomousBasePD(new Pose2d(553.924 * mpi, 108.015 * mpi, new Rotation2d(0)), new Pose2d(553.924 * mpi, 108.015 * mpi, new Rotation2d(0)), new Pose2d(553.924 * mpi, 108.015 * mpi, new Rotation2d(0)), new Pose2d(553.924 * mpi, 108.015 * mpi, new Rotation2d(0)), new Pose2d(553.924 * mpi, 108.015 * mpi, new Rotation2d(0)), new Pose2d(553.924 * mpi, 108.015 * mpi, new Rotation2d(0)), new Pose2d(553.924 * mpi, 108.015 * mpi, new Rotation2d(0)));
  private AutonomousBasePD CurrentPath = HDplaceTwoEngageR;


  PneumaticIntakeSubsystem pneumaticIntakeSubsystem = new PneumaticIntakeSubsystem();

 public static ShuffleboardTab tab = DrivetrainSubsystem.tab;
    public static GenericEntry test =
    tab.add("test", 10)
      .getEntry();
    public static GenericEntry kP =
        tab.add("Auto kP", 0.1)
          .getEntry();
    public static GenericEntry kI =
    tab.add("Auto kI", 0.0)
      .getEntry();
    public static GenericEntry kD =
        tab.add("Auto kD", 0.0)
          .getEntry();
  
  
  /**
  * This function is run when the robot is first started up and should be used for any
  * initialization code.
  */
  @Override
  public void robotInit() { //creates options for different autopaths, names are placeholders

    System.out.println("#I'm Awake");
    
    m_chooser.setDefaultOption("testPath", testPath);
    //m_chooser.setDefaultOption("current", CurrentPath);
    // m_chooser.addOption("noGo!", noGo);
    // m_chooser.addOption("HDLeaveB", HDLeaveB);
    // m_chooser.addOption("HBLeaveB", HBLeaveB); 
    // m_chooser.addOption("timed", timedPath);
    // m_chooser.addOption("HDThreeScoreB", HDThreeScoreB);
    // m_chooser.addOption("HBThreeScoreB", HBThreeScoreB);
    // m_chooser.addOption("HDplaceTwoEngageB", HDplaceTwoEngageB);
    // m_chooser.addOption("engageChargeB", engageChargeB);
    // m_chooser.addOption("HDplaceTwoEngageR", HDplaceTwoEngageR);
    // m_chooser.addOption("HBLeaveR", HBLeaveR);
    // m_chooser.addOption("HDLeaveR", HDLeaveR);
    // m_chooser.addOption("HDThreeScoreR", HDThreeScoreR);
    // m_chooser.addOption("HBThreeScoreR", HBThreeScoreR);
    // m_chooser.addOption("engageChargeR", engageChargeR);
    // //m_chooser.addOption("Motion profiling tester path", motionProfiling);
    // SmartDashboard.putData("Auto choices", m_chooser);
   
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
    SmartDashboard.putNumber("x odometry",DrivetrainSubsystem.m_pose.getX());
    SmartDashboard.putNumber("y odometry",DrivetrainSubsystem.m_pose.getY());
    
    m_field.setRobotPose(DrivetrainSubsystem.m_odometry.getEstimatedPosition());
    
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

    //m_drivetrainSubsystem.m_frontLeftModule.getCANCoder().getPosition();
    // System.out.println("Error code" + m_drivetrainSubsystem.m_frontLeftModule.getCANCoder().getLastError());
    System.out.println("current pose: " + DrivetrainSubsystem.m_pose.getX() + " , " + DrivetrainSubsystem.m_pose.getY());
    m_autoSelected = m_chooser.getSelected();
    m_autoSelected.init();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
     m_autoSelected.periodic();
     //System.out.println("Odometry: "+ DrivetrainSubsystem.m_odometry.getPoseMeters());

     //m_autoSelected.periodic();
     //m_drivetrainSubsystem.drive();
    
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    //m_drivetrainSubsystem.m_frontLeftModule.getCANCoder().getPosition();
   // System.out.println("Error code" + m_drivetrainSubsystem.m_frontLeftModule.getCANCoder().getLastError());

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    /*System.out.println("back left module: " + m_drivetrainSubsystem.m_backLeftModule.getAbsoluteAngle());
    System.out.println("back right module: " + m_drivetrainSubsystem.m_backRightModule.getSteerAngle());
    System.out.println("front left module: " + m_drivetrainSubsystem.m_frontLeftModule.getSteerAngle());
    System.out.println("front right module: " + m_drivetrainSubsystem.m_frontRightModule.getSteerAngle());*/
    m_drivetrainSubsystem.driveTeleop();

    if (OI.m_controller.getBButton()){
      m_drivetrainSubsystem.stopDrive();
    }
    if(OI.m_controller.getAButton()){
      m_drivetrainSubsystem.resetOdometry(new Pose2d());
    }

    if(OI.m_controller.getAButtonReleased()){
      pneumaticIntakeSubsystem.setState(PneumaticIntakeSubsystem.PneumaticIntakeStates.OFF);
    }

    if(OI.m_controller.getXButtonReleased()){
      if(PneumaticIntakeSubsystem.pneumaticIntakeState==PneumaticIntakeSubsystem.PneumaticIntakeStates.ACTUATING || PneumaticIntakeSubsystem.pneumaticIntakeState==PneumaticIntakeSubsystem.PneumaticIntakeStates.OFF){
        pneumaticIntakeSubsystem.setState(PneumaticIntakeSubsystem.PneumaticIntakeStates.RETRACTING);
      } else if(PneumaticIntakeSubsystem.pneumaticIntakeState==PneumaticIntakeSubsystem.PneumaticIntakeStates.RETRACTING){
        pneumaticIntakeSubsystem.setState(PneumaticIntakeSubsystem.PneumaticIntakeStates.ACTUATING); 
      }
    }

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
    m_drivetrainSubsystem.zeroGyroscope();

  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic(){
    //double d = (m_drivetrainSubsystem.m_frontLeftModule.getCANCoder().getPosition()/Constants.TICKS_PER_METER) % 360;
    //System.out.println("LF Cancoder Position: " + d);
    //m_drivetrainSubsystem.m_frontLeftModule.getCANCoder().getPosition();

    //System.out.println("Error code" + m_drivetrainSubsystem.m_frontLeftModule.getCANCoder().getLastError());
    /*if (m_drivetrainSubsystem.m_frontLeftModule.getCANCoder().getLastError() != ErrorCode.OK) {
      return;
    }*/

    /*intial test!*/
    /*if(m_drivetrainSubsystem.getGyroscopeRotation().getDegrees() <= 370){
      m_drivetrainSubsystem.setSpeed(new ChassisSpeeds(0, 0.2, 0));
    }*/
    m_drivetrainSubsystem.setSpeed(ChassisSpeeds.fromFieldRelativeSpeeds(0.2, 0.2, Math.toRadians(0), m_drivetrainSubsystem.getGyroscopeRotation()));
    
    //m_autoSelected.turnDesiredAngle(180);
    m_drivetrainSubsystem.drive();
    // m_drivetrainSubsystem.m_frontLeftModule.set(0.4, t);
    // t+=0.1;

    
    System.out.println(m_drivetrainSubsystem.getGyroscopeRotation());
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
  
}
