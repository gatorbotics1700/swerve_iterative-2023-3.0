// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.subsystems.*;
import frc.robot.OI;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.util.sendable.SendableBuilder.*;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import frc.robot.autonomous.*;
import frc.robot.Constants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableEntry;
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
  private final AutonomousBase autonomousBasePD = new AutonomousBasePD(new Pose2d(0*Constants.TICKS_PER_INCH, -20*Constants.TICKS_PER_INCH, new Rotation2d(0)), 0.0, new Pose2d(), 0.0);
  private static final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  public static double universalPitch=0;

  public static DrivetrainSubsystem getDrivetrainSubsystem(){
    return m_drivetrainSubsystem;
  }

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */


   //NOTE TO L: ORDER IS TURN, DRIVE, PITCH, VELO
  @Override
  public void robotInit() {
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    m_drivetrainSubsystem.resetOdometry();
    // universalPitch = m_drivetrainSubsystem.m_pigeon.getPitch();
    // System.out.println("universal pitch from robot init: " + universalPitch);
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
    
    autonomousBasePD.init();
    /* 
    m_autoSelected = m_chooser.getSelected();
    m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);

    */
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  /* 
    switch (m_autoSelected) {
      case kCustomAuto:
        // Put custom auto code here
        break;
      case kDefaultAuto:
      default:
        // Put default auto code here
        break;
    }
    
    autonomousBasePD.periodic();
    //m_drivetrainSubsystem.drive();
*/
    //m_drivetrainSubsystem.setSpeed(ChassisSpeeds.fromFieldRelativeSpeeds(0.0, 0.2, 0, m_drivetrainSubsystem.getGyroscopeRotation()));
    //m_drivetrainSubsystem.drive();
    m_drivetrainSubsystem.pitchBalance(0.0);

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
    m_drivetrainSubsystem.m_pose = new Pose2d(20, 30, new Rotation2d(Math.PI/4));
    System.out.println("m_pose: " + m_drivetrainSubsystem.m_pose);
    autonomousBasePD.init();

    ShuffleboardTab pidTab = Shuffleboard.getTab("PID Controllers");//creates a new tab for pid controllers
  /* 
    //following code via https://docs.wpilib.org/en/stable/docs/software/dashboards/shuffleboard/layouts-with-code/organizing-widgets.html
    ShuffleboardLayout turnPID = pidTab.getLayout("Turn PID", BuiltInLayouts.kList)//change this layout to support input
    .withSize(2, 2);
    turnPID.add(new ElevatorDownCommand());//what should we put here instead of the commands? i think this is where we should write the info and ask user for input?
    turnPID.add(new ElevatorUpCommand());

    ShuffleboardLayout drivePID = pidTab.getLayout("Drive PID", BuiltInLayouts.kList)//change this layout to support input
    .withSize(2, 2);
    drivePID.add(new ElevatorDownCommand());//what should we put here instead of the commands? i think this is where we should write the info and ask user for input?
    drivePID.add(new ElevatorUpCommand());

    ShuffleboardLayout pitchPID = pidTab.getLayout("Pitch PID", BuiltInLayouts.kList)//change this layout to support input
    .withSize(2, 2);riodic
    pitchPID.add(new ElevatorDownCommand());//what should we put here instead of the commands? i think this is where we should write the info and ask user for input?
    pitchPID.add(new ElevatorUpCommand());

    ShuffleboardLayout veloPID = pidTab.getLayout("Velocity PID", BuiltInLayouts.kList)//change this layout to support input
    .withSize(2, 2);
    pitchPID.add(new ElevatorDownCommand());//what should we put here instead of the commands? i think this is where we should write the info and ask user for input?
    pitchPID.add(new ElevatorUpCommand());

    //following code via https://docs.wpilib.org/en/stable/docs/software/dashboards/shuffleboard/advanced-usage/shuffleboard-tuning-pid.html
    //NetworkTableEntry turnWidget = pidTab.add("Turn PID", 1).getEntry();//NOTE: i think this is an alternative way to create a widget as opposed to what's shown above
  */
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    m_drivetrainSubsystem.setSpeed(new ChassisSpeeds(0.2, 0, 0));
    
    m_drivetrainSubsystem.drive();
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}

  /* 
  @Override
  public void initSendable(SendableBuilder builder){ //via https://docs.wpilib.org/en/stable/docs/software/telemetry/writing-sendable-classes.html
    builder.addDoubleProperty("turnPIDk", this::getSetpoint, this::setSetpoint);//TODO finish adding these, and verify that this works
  }
  */
}
