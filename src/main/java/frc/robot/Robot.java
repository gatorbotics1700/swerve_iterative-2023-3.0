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
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.geometry.Rotation2d;
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
  private AutonomousBase autonomousBase = new AutonomousBase();
  // = new AutonomousBasePD(new Pose2d(0*Constants.TICKS_PER_INCH, 20*Constants.TICKS_PER_INCH, new Rotation2d()), 90, new Pose2d(), 0.0);
  public static final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem(); //if anything breaks in the future it might be this
  private final Field2d m_field = new Field2d();
  // public static DrivetrainSubsystem getDrivetrainSubsystem(){
  //   return m_drivetrainSubsystem;
  // }

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {

    System.out.println("#I'm Awake");
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto 1", kCustomAuto);
    m_chooser.addOption("My Auto 2", kCustomAuto);
    m_chooser.addOption("My Auto 3", kCustomAuto);
    m_chooser.addOption("My Auto 4", kCustomAuto);
    m_chooser.addOption("My Auto 5", kCustomAuto);
    m_chooser.addOption("My Auto timed", kCustomAuto);

    SmartDashboard.putData("Auto choices", m_chooser);
    //m_drivetrainSubsystem.resetOdometry();
    // m_drivetrainSubsystem.zeroGyroscope();
    // m_drivetrainSubsystem.zeroDriveEncoder();
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
    SmartDashboard.putNumber("x odometry",DrivetrainSubsystem.m_pose.getX()/Constants.TICKS_PER_INCH);
    SmartDashboard.putNumber("y odometry",DrivetrainSubsystem.m_pose.getY()/Constants.TICKS_PER_INCH);
    m_field.setRobotPose(DrivetrainSubsystem.m_odometry.getPoseMeters());
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

    /* m_autoSelected = m_chooser.getSelected();
    // AutonomousBasePD = m_autoSelected;
    if(){
      autonomousBase = new AutonomousBasePD(new Pose2d(), new Pose2d());
      } else if blah {}
      autonomousBase.init(); */


  
  /*
  autopath 1: INCOMPLETE
  goal coordinate 1: (221.313, 15.010)
  goal coordinate 2: (0, 21.225)
  goal coordinate 3: (133.914, 24.081)

  autopath 2: THESE ARE WRONG TO BE FIXED
  goal coordinate 1: (226.403, 19.903)
  goal coordinate 2: (226.277, -2.614)
  goal coordinate 3: (226.277, 45.386)
  goal corrdinate 4(final): (127.692, 0) 
  + amount from moving charging station which is 4.3 when going over both sides
*/
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
     //m_drivetrainSubsystem.driveTeleop();
     autonomousBasePD.periodic();
     m_drivetrainSubsystem.drive();

     //System.out.println("Odometry: "+ DrivetrainSubsystem.m_odometry.getPoseMeters());
    
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    m_drivetrainSubsystem.resetOdometry();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    m_drivetrainSubsystem.driveTeleop();


    if (OI.m_controller.getBButton()){
      m_drivetrainSubsystem.stopDrive();
    }

    if(OI.m_controller.getAButton()){
      m_drivetrainSubsystem.resetOdometry();
    }

    System.out.println("Odometry: "+ DrivetrainSubsystem.m_odometry.getPoseMeters());
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
    autonomousBasePD.init();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    //m_drivetrainSubsystem.driveTeleop();
    autonomousBasePD.periodic();
    m_drivetrainSubsystem.drive();

    if(OI.m_controller.getAButton()){
      m_drivetrainSubsystem.resetOdometry();
    }

    System.out.println("Odometry: "+ DrivetrainSubsystem.m_odometry.getPoseMeters());
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
