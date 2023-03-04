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

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.fasterxml.jackson.core.sym.Name;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTable.*;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.ArmTelescopingSubsystem.TelescopingStates;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */


//private AutonomousBasePD mScore = new AutonomousBasePD(new Translation2d(222.037, 0), new Translation2d(135.091, -41.307), new Translation2d(0, -44.163), new Translation2d(222.894, -50.377), new Translation2d(0, -65.388), new Translation2d(0, -65.388));

public class Robot extends TimedRobot {

  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
 // private final AutonomousBase autonomousBasePD = new AutonomousBasePD(new Pose2d(0*Constants.TICKS_PER_INCH, -20*Constants.TICKS_PER_INCH, new Rotation2d(0)), 0.0, new Pose2d(), 0.0);
  public static final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  private static ArmTelescopingSubsystem armTelescopingSubsystem = new ArmTelescopingSubsystem();

  public static DrivetrainSubsystem getDrivetrainSubsystem(){
    return m_drivetrainSubsystem;
  }

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() { //creates options for different autopaths, names are placeholders
    System.out.println("robot init");
  //   System.out.println("#I'm Awake");
  //   m_chooser.setDefaultOption("Default Auto", testPath);
  //   m_chooser.addOption("My Auto 1", noGo);
  //   m_chooser.addOption("My Auto 2",placeNLeave);
  //   m_chooser.addOption("My Auto 3", antiCharge);
  //   m_chooser.addOption("My Auto 4", antiChargeOpposite);
  //   //m_chooser.addOption(name: "My Auto 5", engageCharge);
  //  // m_chooser.addOption(name: "My Auto 6",placeTwoEngage);
  //   m_chooser.addOption("My Auto timed", timedPath);
  //   m_chooser.addOption("Motion profiling path", motionProfiling);

    SmartDashboard.putData("Auto choices", m_chooser);

    // ShuffleboardTab tab = DrivetrainSubsystem.tab;
    //  kP = tab.add("Auto kP", 0.1).getEntry(); 
    //  kI = tab.add("Auto kI", 0.0).getEntry();
    //  kD = tab.add("Auto kD", 0.0).getEntry();
   

  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
 // @Override
  // public void robotPeriodic() {
  //   SmartDashboard.putNumber("x odometry",DrivetrainSubsystem.m_pose.getX()/Constants.TICKS_PER_INCH);
  //   SmartDashboard.putNumber("y odometry",DrivetrainSubsystem.m_pose.getY()/Constants.TICKS_PER_INCH);
  //   m_field.setRobotPose(DrivetrainSubsystem.m_odometry.getPoseMeters());
  // }

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
    armTelescopingSubsystem.init();
    armTelescopingSubsystem.telescopingMotor.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs); //VERY VERY IMPORTANT
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
     armTelescopingSubsystem.setTState(TelescopingStates.SHELF_ARM_LENGTH);
     armTelescopingSubsystem.periodic();
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {

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

    // m_drivetrainSubsystem.m_pose = new Pose2d(20, 30, new Rotation2d(Math.PI/4));
    // System.out.println("m_pose: " + m_drivetrainSubsystem.m_pose);
    // autonomousBasePD.init();
    armTelescopingSubsystem.init();
    armTelescopingSubsystem.telescopingMotor.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs); //VERY VERY IMPORTANT


  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    //armTelescopingSubsystem.telescopingMotor.set(ControlMode.PercentOutput, -0.1);
    //System.out.println("telescoping ticks: " + armTelescopingSubsystem.telescopingMotor.getSelectedSensorPosition());
     armTelescopingSubsystem.setTState(TelescopingStates.RETRACTED);
     armTelescopingSubsystem.periodic();

  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}