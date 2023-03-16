// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import frc.robot.Buttons;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.subsystems.*;
import frc.robot.subsystems.ArmPneumaticPivot;
import frc.robot.subsystems.ArmPneumaticPivot.PneumaticPivotStates;
// import frc.robot.subsystems.ArmPneumaticPivot.PneumaticPivotStates;
// import frc.robot.subsystems.PneumaticIntakeSubsystem.PneumaticIntakeStates;
import frc.robot.OI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.autonomous.*;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.fasterxml.jackson.core.sym.Name;

import java.lang.ProcessBuilder.Redirect;

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
import frc.robot.subsystems.PneumaticIntakeSubsystem.PneumaticIntakeStates;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.robot.Constants;
import frc.robot.autonomous.StateWithCoordinate.AutoStates;
import frc.robot.subsystems.ArmTelescopingSubsystem.TelescopingStates;
import frc.robot.subsystems.Vision.*;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.autonomous.PDPath;



/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */


//private AutonomousBasePD mScore = new AutonomousBasePD(new Translation2d(222.037, 0), new Translation2d(135.091, -41.307), new Translation2d(0, -44.163), new Translation2d(222.894, -50.377), new Translation2d(0, -65.388), new Translation2d(0, -65.388));

public class Robot extends TimedRobot {

  private AutonomousBase m_autoSelected;
  private static final Boolean red = false;
  private static final Boolean blue = true;
  private final SendableChooser<AutonomousBase> auto_chooser = new SendableChooser<>();
  private final SendableChooser<Boolean> allianceChooser = new SendableChooser<>();

  public static final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem(); //if anything breaks in the future it might be this
  public static PneumaticIntakeSubsystem m_pneumaticIntakeSubsystem = new PneumaticIntakeSubsystem();
  
  public static Mechanisms m_mechanisms = new Mechanisms();
  public static ArmTelescopingSubsystem armTelescopingSubsystem = new ArmTelescopingSubsystem();

  private static LimeLightSubsystem m_limeLightSubsystem = new LimeLightSubsystem();
  public static AprilTagSubsystem m_aprilTagSubsystem = new AprilTagSubsystem();
  public static ArmPneumaticPivot m_ArmPneumaticPivot = new ArmPneumaticPivot();

  public static Buttons m_buttons = new Buttons();
  
  double t= 0.0;
  boolean override = false;
  ChassisSpeeds m_ChassisSpeeds; 
  double mpi = Constants.METERS_PER_INCH;
  public static Boolean isBlueAlliance = true;
  
  private AutonomousBaseTimed timedPath = new AutonomousBaseTimed();
  private AutonomousBasePD testPath = PDPath.HDLeaveR;
 
// red alliance  
// half the field (325.8415) - blue x value + half the field (325.8415) = red x value
 // whole field: 651.683 (inches)
  
  /**
  * This function is run when the robot is first started up and should be used for any
  * initialization code.
  */
  @Override
  public void robotInit() { //creates options for different autopaths, names are placeholders    
    System.out.println("#I'm Awake");
    

    allianceChooser.setDefaultOption("Blue Alliance", blue);
    allianceChooser.addOption("Red Alliance", red);    
    SmartDashboard.putData("Which alliance?", allianceChooser);


    auto_chooser.setDefaultOption("testPath", testPath);
    auto_chooser.addOption("noGoR!", PDPath.noGoR);
    auto_chooser.addOption("noGoB!", PDPath.noGoB);
    auto_chooser.addOption("HDLeaveB", PDPath.HDLeaveB);
    auto_chooser.addOption("HBLeaveB", PDPath.HBLeaveB);
    auto_chooser.addOption("HBLeaveR", PDPath.HBLeaveR);
    auto_chooser.addOption("HDLeaveR", PDPath.HDLeaveR); 
    auto_chooser.addOption("HDLeaveB", PDPath.HDPlaceLeaveB);
    auto_chooser.addOption("HBLeaveB", PDPath.HBPlaceLeaveB);
    auto_chooser.addOption("HDLeaveR", PDPath.HDPlaceLeaveR); 
    auto_chooser.addOption("HBLeaveR", PDPath.HBPlaceLeaveR);
    auto_chooser.addOption("engageChargeR", PDPath.engageChargeR);
    auto_chooser.addOption("engageChargeB", PDPath.engageChargeB);
    auto_chooser.addOption("HDIntakeEngageB", PDPath.HDIntakeEngageB);
    auto_chooser.addOption("HDIntakeEngageR", PDPath.HDIntakeEngageR);
    auto_chooser.addOption("HD3ScoreR", PDPath.HD3ScoreR);
    auto_chooser.addOption("HD3ScoreB", PDPath.HD3ScoreB);
    auto_chooser.addOption("HB3ScoreR", PDPath.HB3ScoreR);
    auto_chooser.addOption("HB3ScoreB", PDPath.HB3ScoreB);
    auto_chooser.addOption("timed", timedPath);
   // auto_chooser.addOption("Motion profiling tester path", motionProfiling);
    SmartDashboard.putData("Auto choices", auto_chooser);
   
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
    // System.out.println(allianceChooser.getSelected());
    isBlueAlliance = allianceChooser.getSelected();
    SmartDashboard.putBoolean("Alliance: ", isBlueAlliance); 
    SmartDashboard.putNumber("x odometry",DrivetrainSubsystem.m_pose.getX()/Constants.METERS_PER_INCH);
    SmartDashboard.putNumber("y odometry",DrivetrainSubsystem.m_pose.getY()/Constants.METERS_PER_INCH);
    SmartDashboard.putNumber("angle odometry",DrivetrainSubsystem.m_pose.getRotation().getDegrees()%360);
    //SmartDashboard.putBoolean("Ready to Score", m_limeLightSubsystem.seeSomething());
    SmartDashboard.putBoolean("beam broken?", m_pneumaticIntakeSubsystem.isBeamBroken());
    SmartDashboard.putBoolean("cube?", m_pneumaticIntakeSubsystem.getPurple());
    SmartDashboard.putBoolean("cone?", m_pneumaticIntakeSubsystem.getYellow());
    //System.out.println("x: " + DrivetrainSubsystem.m_pose.getX() + "y: " + DrivetrainSubsystem.m_pose.getY() + "rotation: " + DrivetrainSubsystem.m_pose.getRotation().getDegrees()%360);
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

    m_drivetrainSubsystem.setIsBlueAlliance(allianceChooser.getSelected());
    System.out.println("current pose: " + DrivetrainSubsystem.m_pose.getX() + " , " + DrivetrainSubsystem.m_pose.getY());
    m_autoSelected = auto_chooser.getSelected();
    m_autoSelected.init();
    isBlueAlliance = allianceChooser.getSelected();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
     //m_AprilTagSubsystem.periodic();
     m_mechanisms.periodic();
     m_autoSelected.periodic();
     
     //System.out.println("Odometry: "+ DrivetrainSubsystem.m_odometry.getPoseMeters());

     //m_drivetrainSubsystem.drive();
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    m_aprilTagSubsystem.init();
    isBlueAlliance = allianceChooser.getSelected();
    m_mechanisms.init();
    //m_drivetrainSubsystem.resetOdometry(new Pose2d()); //TODO: delete
    m_drivetrainSubsystem.resetOdometry(new Pose2d(AprilTagLocation.scoringPoses[1].getX() -36.5*Constants.METERS_PER_INCH, AprilTagLocation.scoringPoses[1].getY() - 12.0*Constants.METERS_PER_INCH, new Rotation2d(Math.toRadians(0.0))));
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    if(AprilTagSubsystem.states != AprilTagSubsystem.AprilTagSequence.CORRECTPOSITION){
      m_drivetrainSubsystem.driveTeleop();
    }
    
    m_drivetrainSubsystem.drive();
    m_mechanisms.periodic();
    m_aprilTagSubsystem.periodic();
    m_pneumaticIntakeSubsystem.periodic();
    m_buttons.buttonsPeriodic();

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
    //armTelescopingSubsystem.init();
    //m_mechanisms.init();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
   // System.out.println(m_drivetrainSubsystem.m_frontLeftModule.getCANCoder().getAbsolutePosition()%360);
    //m_mechanisms.setState(MechanismStates.HOLDING);
    //m_mechanisms.periodic();
    // if(OI.m_controller.getAButtonPressed()){
    //   armTelescopingSubsystem.setTState(TelescopingStates.MID_ARM_LENGTH);
    // } else if (OI.m_controller.getBButtonPressed()){
    //   System.out.println("b button --> low");
    //   armTelescopingSubsystem.setTState(TelescopingStates.LOW_ARM_LENGTH);
    // } else if (OI.m_controller.getXButtonPressed()){
    //   armTelescopingSubsystem.setTState(TelescopingStates.RETRACTED);
    // } else if(OI.m_controller.getYButton()){
    //   armTelescopingSubsystem.telescopingMotor.set(ControlMode.PercentOutput, -0.2);
    // }
    /*if (OI.m_controller.getLeftBumper()){
      ArmTelescopingSubsystem.telescopingMotor.setSelectedSensorPosition(0);
    } else {
      ArmTelescopingSubsystem.telescopingMotor.set(ControlMode.PercentOutput, 0.2);
    }*/
    m_ArmPneumaticPivot.setState(PneumaticPivotStates.RETRACTING);
   // armTelescopingSubsystem.periodic();
   //m_limeLightSubsystem.setPipeline(0.0);
   //System.out.println("Tv: " + m_limeLightSubsystem.getTv());
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}

  
}

