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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.ArmTelescopingSubsystem.TelescopingStates;
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
  private final AutonomousBase autonomousBasePD = new AutonomousBasePD(new Pose2d(0*Constants.TICKS_PER_INCH, -20*Constants.TICKS_PER_INCH, new Rotation2d(0)), 0.0, new Pose2d(), 0.0);
  private static final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  private static ArmTelescopingSubsystem armTelescopingSubsystem = new ArmTelescopingSubsystem();

  public static DrivetrainSubsystem getDrivetrainSubsystem(){
    return m_drivetrainSubsystem;
  }

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    m_drivetrainSubsystem.resetOdometry();
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
    armTelescopingSubsystem.init();

  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
   
    //armTelescopingSubsystem.timedMoveArm(0.5, false); //you can change forwards to be false to go backwards; time is in seconds

    armTelescopingSubsystem.setTState(TelescopingStates.FULLY_EXTENDED);
    armTelescopingSubsystem.periodic();
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
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
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