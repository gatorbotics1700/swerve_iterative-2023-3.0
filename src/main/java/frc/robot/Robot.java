// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.autonomous.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.fasterxml.jackson.core.sym.Name;
import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.cscore.CameraServerJNI.TelemetryKind;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable.*;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.ArmTelescopingSubsystem.TelescopingStates;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorStates;
import frc.robot.subsystems.Mechanisms.MechanismStates;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.Constants;
import frc.robot.autonomous.StateWithCoordinate.AutoStates;
import frc.robot.autonomous.StateWithCoordinate;
import frc.robot.subsystems.ArmTelescopingSubsystem.TelescopingStates;
import frc.robot.subsystems.Vision.AprilTagSubsystem;
import frc.robot.subsystems.Vision.LimeLightSubsystem;
import edu.wpi.first.math.geometry.Translation2d;



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
  private final SendableChooser<Boolean> allianceChooser = new SendableChooser<Boolean>();

  public static final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem(); //if anything breaks in the future it might be this
  public static PneumaticIntakeSubsystem m_pneumaticIntakeSubsystem = new PneumaticIntakeSubsystem();
  public static Mechanisms m_mechanisms = new Mechanisms();


  private final LimeLightSubsystem m_limeLightSubsystem = new LimeLightSubsystem();
  private final AprilTagSubsystem m_aprilTagSubsystem = new AprilTagSubsystem();
  
  double t= 0.0;
  boolean override = false;
  ChassisSpeeds m_ChassisSpeeds;
  double mpi = Constants.METERS_PER_INCH;
  public static boolean isBlueAlliance = true;
  public static AutoStates level;
  public static int scoringCol = 0;

  // whole field: 651.683 (inches)
  // private AutonomousBasePD noGo = new AutonomousBasePD(new Pose2d(0, 0, new Rotation2d(0)), new Pose2d(0, 0, new Rotation2d(0)), new Pose2d(0, 0, new Rotation2d(0)), new Pose2d(0,0, new Rotation2d(0)), new Pose2d(0, 0, new Rotation2d(0)), new Pose2d(0,0, new Rotation2d(0)), new Pose2d(0,0, new Rotation2d(0)));
  private AutonomousBaseTimed timedPath = new AutonomousBaseTimed();
  
  private AutonomousBasePD testPath = PDPath.HDLeaveR;
  /*new AutonomousBasePD(
    new Pose2d(0.0, 0.0, new Rotation2d(Math.toRadians(0))), 
    new StateWithCoordinate[]{
      new StateWithCoordinate(AutoStates.FIRST),
      new StateWithCoordinate(AutoStates.DRIVE, new Pose2d(20 * mpi, 0 * mpi, new Rotation2d(Math.toRadians(0)))), 
      //new StateWithCoordinate(AutoStates.DRIVE, new Pose2d(40 * mpi, 0, new Rotation2d(60))), 
      //new StateWithCoordinate(AutoStates.DRIVE, new Pose2d(0 * mpi, 30 * mpi, new Rotation2d(60))), 
      //new StateWithCoordinate(AutoStates.DRIVE, new Pose2d(40 * mpi, 30 * mpi, new Rotation2d(60))), 
      //new StateWithCoordinate(AutoStates.DRIVE, new Pose2d(0 * mpi, 0 * mpi, new Rotation2d(150))), 
      //new StateWithCoordinate(AutoStates.DRIVE, new Pose2d(20 * mpi, 20 * mpi, new Rotation2d(150))),
      new StateWithCoordinate(AutoStates.STOP)
    });*/
  
 
// red alliance  
// half the field (325.8415) - blue x value + half the field (325.8415) = red x value
   
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
    
    allianceChooser.setDefaultOption("blue alliance", true);
    allianceChooser.addOption("blue alliance", false);
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
    SmartDashboard.putNumber("x odometry",DrivetrainSubsystem.m_pose.getX()/Constants.METERS_PER_INCH);
    SmartDashboard.putNumber("y odometry",DrivetrainSubsystem.m_pose.getY()/Constants.METERS_PER_INCH);
    SmartDashboard.putNumber("angle odometry",DrivetrainSubsystem.m_pose.getRotation().getDegrees()%360);
    SmartDashboard.putBoolean("Ready to Score", m_limeLightSubsystem.seeSomething());
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
    m_mechanisms.init();
    //m_drivetrainSubsystem.m_frontLeftModule.getCANCoder().getPosition();
    // System.out.println("Error code" + m_drivetrainSubsystem.m_frontLeftModule.getCANCoder().getLastError());
    System.out.println("current pose: " + DrivetrainSubsystem.m_pose.getX() + " , " + DrivetrainSubsystem.m_pose.getY());
    m_autoSelected = m_chooser.getSelected();
    m_autoSelected.init();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
     //m_AprilTagSubsystem.periodic();
     m_mechanisms.periodic();
     m_autoSelected.periodic();
     
     //System.out.println("Odometry: "+ DrivetrainSubsystem.m_odometry.getPoseMeters());

    //  m_autoSelected.periodic();
     //m_drivetrainSubsystem.drive();
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    //m_aprilTagSubsystem.init();
    //m_drivetrainSubsystem.m_frontLeftModule.getCANCoder().getPosition();
   // System.out.println("Error code" + m_drivetrainSubsystem.m_frontLeftModule.getCANCoder().getLastError());
    //armTelescopingSubsystem.init();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    m_drivetrainSubsystem.driveTeleop();
    //m_mechanisms.periodic();
    //System.out.println("i am in teleop");
    //m_aprilTagSubsystem.periodic();

    if(OI.m_controller.getPOV() >= 225 && OI.m_controller.getPOV() <= 315){
      if (!override){
        System.out.println("dpad 270: left substation");
        level = AutoStates.LEFTPICKUP; 
      }else{
        m_mechanisms.setState(MechanismStates.SHELF);
      }
    }

    if(OI.m_controller.getPOV() >= 45 && OI.m_controller.getPOV() <= 135){
      if (!override){
        System.out.println("dpad 90: right substation");
        level = AutoStates.RIGHTPICKUP; 
      }else{
        m_mechanisms.setState(MechanismStates.SHELF);
      }
    }

    if(OI.m_controller.getBackButton()){
      //m_VisionSubsystem.setState(VisionSubsystem.VisionStates.DETECTTAPE);
    }
    
    if(OI.m_controller.getAButtonReleased()){
      //pneumaticIntakeSubsystem.setState(PneumaticIntakeSubsystem.PneumaticIntakeStates.OFF);
    }

    if(OI.m_controller.getXButtonReleased()){
      if(PneumaticIntakeSubsystem.pneumaticIntakeState==PneumaticIntakeSubsystem.PneumaticIntakeStates.ACTUATING || PneumaticIntakeSubsystem.pneumaticIntakeState==PneumaticIntakeSubsystem.PneumaticIntakeStates.OFF){
        //pneumaticIntakeSubsystem.setState(PneumaticIntakeSubsystem.PneumaticIntakeStates.RETRACTING);
      } else if(PneumaticIntakeSubsystem.pneumaticIntakeState==PneumaticIntakeSubsystem.PneumaticIntakeStates.RETRACTING){
        //pneumaticIntakeSubsystem.setState(PneumaticIntakeSubsystem.PneumaticIntakeStates.ACTUATING); 
      }
    }

    if(OI.joystick.getRawButton(0)){
      buttonLevel(0);
    } else if (OI.joystick.getRawButton(1)){
      buttonLevel(1);
    } else if (OI.joystick.getRawButton(2)){
      buttonLevel(2);
    } else if (OI.joystick.getRawButton(3)){
      buttonLevel(3);
    } else if (OI.joystick.getRawButton(4)){
      buttonLevel(4);
    } else if (OI.joystick.getRawButton(5)){
      buttonLevel(5);
    } else if (OI.joystick.getRawButton(6)){
      buttonLevel(6);
    } else if (OI.joystick.getRawButton(7)){
      buttonLevel(7);
    } else if (OI.joystick.getRawButton(8)){
      buttonLevel(8);
    }
    /*System.out.println("back left module: " + m_drivetrainSubsystem.m_backLeftModule.getAbsoluteAngle());
    System.out.println("back right module: " + m_drivetrainSubsystem.m_backRightModule.getSteerAngle());
    System.out.println("front left module: " + m_drivetrainSubsystem.m_frontLeftModule.getSteerAngle());
    System.out.println("front right module: " + m_drivetrainSubsystem.m_frontRightModule.getSteerAngle());*/

    System.out.println("override is " + override);

    //driver
    if (OI.m_controller.getBButton()){ 
      m_drivetrainSubsystem.stopDrive(); //stop all mech?
    }

    if(OI.m_controller.getXButton()){
    // m_drivetrainSubsystem.pitchBalance(0.0);
    }


    if(OI.m_controller_two.getAButton()){ 
      if (override){
        System.out.println("xbox: low node");
        // m_drivetrainSubsystem.scoreLow();
          //armTelescopingSubsystem.setTState(TelescopingStates.LOW_ARM_LENGTH);
          m_mechanisms.setState(MechanismStates.LOW_NODE);
        } else {
          level = AutoStates.LOWNODE;
        }
    }

    if(OI.m_controller_two.getBButton()){
      if (override){
        System.out.println("xbox: mid");
        //m_drivetrainSubsystem.scoreMid();
        m_mechanisms.setState(MechanismStates.MID_NODE);
      }else{
        level = AutoStates.MIDNODE; 
      }
      
    }

    if(OI.m_controller_two.getYButton()){
      if (override){
        System.out.println("xbox: high node");
        // m_drivetrainSubsystem.scoreHigh();
        m_mechanisms.setState(MechanismStates.HIGH_NODE);
      }else{
        level = AutoStates.HIGHNODE; 
      }

    }

    if(OI.m_controller_two.getXButton()){ //override button
      override = !override;
    }



    if(OI.m_controller_two.getLeftBumperReleased()){ //needs its own button & not enough
      if(PneumaticIntakeSubsystem.pneumaticIntakeState==PneumaticIntakeSubsystem.PneumaticIntakeStates.ACTUATING || PneumaticIntakeSubsystem.pneumaticIntakeState==PneumaticIntakeSubsystem.PneumaticIntakeStates.OFF){
        pneumaticIntakeSubsystem.setStatePneumaticIntake(PneumaticIntakeSubsystem.PneumaticIntakeStates.RETRACTING);
      } else if(PneumaticIntakeSubsystem.pneumaticIntakeState==PneumaticIntakeSubsystem.PneumaticIntakeStates.RETRACTING){
        pneumaticIntakeSubsystem.setStatePneumaticIntake(PneumaticIntakeSubsystem.PneumaticIntakeStates.ACTUATING); 
      }
    }

    if(OI.m_controller_two.getRightBumper()){
    }

    if (OI.m_controller_two.getStartButton()){
      //m_drivetrainSubsystem.stopAllMech();
    }

    if (OI.m_controller_two.getBackButton()){

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
    m_drivetrainSubsystem.resetOdometry(new Pose2d(0, 0, new Rotation2d(Math.toRadians(0))));
    testPath.init();
    
   // m_aprilTagSubsystem.init();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic(){
    //m_aprilTagSubsystem.periodic();
    //testPath.driveDesiredDistance(new Pose2d(20 * Constants.METERS_PER_INCH, 20 * Constants.METERS_PER_INCH, new Rotation2d(Math.toRadians(0))));
    
    m_drivetrainSubsystem.setSpeed(ChassisSpeeds.fromFieldRelativeSpeeds(0.2, 0.0, Math.toRadians(0), m_drivetrainSubsystem.getPoseRotation()));
    
    
    m_drivetrainSubsystem.drive();
    //System.out.println("rotation: " + DrivetrainSubsystem.m_pose.getRotation());

    
    //System.out.println(m_drivetrainSubsystem.getGyroscopeRotation());
    //System.out.println("Front Left Module Postion: " + m_drivetrainSubsystem.m_frontLeftModule.getPosition());
    //System.out.println("Front Right Module Position: " + m_drivetrainSubsystem.m_frontRightModule.getPosition());
    //System.out.println("Back Left Module Position: " + m_drivetrainSubsystem.m_backLeftModule.getPosition());
    //System.out.println("Back Right Module Position: " + m_drivetrainSubsystem.m_backRightModule.getPosition());
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}

  public void buttonLevel(int col){
    if(isBlueAlliance){
      if(level == AutoStates.LOWNODE){
        scoringCol = col + 9;
      } else {
        scoringCol = col;
      }
    } else { //red
      if(level == AutoStates.LOWNODE){
        scoringCol = col + 27;
      } else {
        scoringCol = col + 18;
      }
    }
  }
}

