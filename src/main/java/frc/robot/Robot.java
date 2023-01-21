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
import frc.robot.autonomous.AutonomousBase.Paths;
import frc.robot.Constants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private AutonomousBase m_autoSelected;
  private final SendableChooser<AutonomousBase> m_chooser = new SendableChooser<AutonomousBase>();
  private AutonomousBase autonomousBase = new AutonomousBase();
  private AutonomousBasePD blueCharge = new AutonomousBasePD(new Translation2d(221.353, 23.720), new Translation2d(0, 21.574), new Translation2d(96.902, 21.574), new Translation2d(96.902, 21.574), new Translation2d(96.902, 21.574), new Translation2d(96.902, 21.574));
  private AutonomousBasePD redCharge = new AutonomousBasePD(new Translation2d(222.624, 15.665), new Translation2d(0, 21.886), new Translation2d(221.671, 67.260), new Translation2d(97.188, 67.260), new Translation2d(97.188, 67.260), new Translation2d(97.188, 67.260));
  private AutonomousBasePD antiCharge = new AutonomousBasePD(new Translation2d(86.840, -45.282), new Translation2d(221.978, 19.463), new Translation2d(135.091, -19.421), new Translation2d(0, -22.277), new Translation2d(222.491, -28.492), new Translation2d(0, -43.502));
  private AutonomousBasePD mScore = new AutonomousBasePD(new Translation2d(222.037, 0), new Translation2d(135.091, -41.307), new Translation2d(0, -44.163), new Translation2d(222.894, -50.377), new Translation2d(0, -65.388), new Translation2d(0, -65.388));
  private AutonomousBaseTimed timedPath = new AutonomousBaseTimed();
  private AutonomousBasePD testPath = new AutonomousBasePD(new Translation2d(0, 20), new Translation2d(0, 20), new Translation2d(0, 20), new Translation2d(0, 20), new Translation2d(0, 20), new Translation2d(0, 20));
  
  public static final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem(); //if anything breaks in the future it might be this
  private final Field2d m_field = new Field2d();

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() { //creates options for different autopaths, names are placeholders
 
    System.out.println("#I'm Awake");
    m_chooser.setDefaultOption("Default Auto", testPath);
    m_chooser.addOption("My Auto 1", blueCharge);
    m_chooser.addOption("My Auto 2", redCharge);
    m_chooser.addOption("My Auto 3", antiCharge);
    m_chooser.addOption("My Auto 4", mScore);
    m_chooser.addOption("My Auto timed", timedPath);

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
    m_autoSelected = m_chooser.getSelected();
    m_autoSelected.init();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
     //m_drivetrainSubsystem.driveTeleop();
     m_autoSelected.periodic();
     m_drivetrainSubsystem.drive();

     //System.out.println("Odometry: "+ DrivetrainSubsystem.m_odometry.getPoseMeters());
    
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    m_drivetrainSubsystem.driveTeleop();


    if (OI.m_controller.getBButton()){
      m_drivetrainSubsystem.stopDrive();
    }

    if(OI.m_controller.getAButton()){ //Katherine is skeptical whether this button work :/
      m_drivetrainSubsystem.resetOdometry();
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
    m_autoSelected.init();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    //m_drivetrainSubsystem.driveTeleop();
    m_autoSelected.periodic();
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
