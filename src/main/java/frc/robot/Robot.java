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
  private AprilTagSubsystem m_AprilTagSubsystem = new AprilTagSubsystem();
  private VisionSubsystem m_VisionSubsystem = new VisionSubsystem();
  public static DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  public static LimeLightSubsystem limeLightSubsystem = new LimeLightSubsystem();
  // whole field: 651.683 
  // center : 325.8415
  private AutonomousBasePD noGo = new AutonomousBasePD(new Pose2d(0, 0, new Rotation2d(0)), new Pose2d(0, 0, new Rotation2d(0)), new Pose2d(0, 0, new Rotation2d(0)), new Pose2d(0,0, new Rotation2d(0)), new Pose2d(0, 0, new Rotation2d(0)), new Pose2d(0,0, new Rotation2d(0)), new Pose2d(0,0, new Rotation2d(0)), 0.0);
  private AutonomousBaseTimed timedPath = new AutonomousBaseTimed();
  private AutonomousBasePD testPath = new AutonomousBasePD(new Pose2d(0, 0, new Rotation2d(180)), new Pose2d(0, -20*Constants.METERS_PER_INCH, new Rotation2d(180)), new Pose2d(10, 30, new Rotation2d(180)), new Pose2d(0, 20, new Rotation2d(180)), new Pose2d(0, 20, new Rotation2d(180)), new Pose2d(0, 20, new Rotation2d(180)), new Pose2d(0, 20, new Rotation2d(180)), 0.0);
  /*
  // blue alliance 
  private AutonomousBasePD HDplaceTwoEngageB = new AutonomousBasePD(new Pose2d(56.069, 20.19, new Rotation2d(0)), new Pose2d(278.999, 36.19, new Rotation2d(0)), new Pose2d(257.650, 64.004, new Rotation2d(0)), new Pose2d(156.859, 83.368, new Rotation2d(0)), new Pose2d(156.859, 83.368, new Rotation2d(0)), new Pose2d(156.859, 83.368, new Rotation2d(0)), new Pose2d(156.859, 83.368, new Rotation2d(0)));
  private AutonomousBasePD HBLeaveB = new AutonomousBasePD(new Pose2d(56.069, 200.046, new Rotation2d(0)), new Pose2d(219.915, 200.046, new Rotation2d(0)), new Pose2d(219.915, 200.046, new Rotation2d(0)), new Pose2d(219.915, 200.046, new Rotation2d(0)), new Pose2d(219.915, 200.046, new Rotation2d(0)), new Pose2d(219.915, 200.046, new Rotation2d(0)), new Pose2d(219.915, 200.046, new Rotation2d(0)));
  private AutonomousBasePD HDLeaveB = new AutonomousBasePD(new Pose2d(56.069, 20.19, new Rotation2d(0)), new Pose2d(219.915, 20.19, new Rotation2d(0)), new Pose2d(219.915, 20.19, new Rotation2d(0)), new Pose2d(219.915, 20.19, new Rotation2d(0)), new Pose2d(219.915, 20.19, new Rotation2d(0)), new Pose2d(219.915, 20.19, new Rotation2d(0)), new Pose2d(219.915, 20.19, new Rotation2d(0)));
  private AutonomousBasePD HDThreeScoreB = new AutonomousBasePD(new Pose2d(56.069, 20.19, new Rotation2d(0)), new Pose2d(278.999, 37.193, new Rotation2d(0)), new Pose2d(56.222, 43.068, new Rotation2d(0)), new Pose2d(197.484,45.934, new Rotation2d(0)), new Pose2d(279.077, 85.622, new Rotation2d(0)), new Pose2d(197.484,40.000, new Rotation2d(0)), new Pose2d(56.154,66.117, new Rotation2d(0)));
  private AutonomousBasePD HBThreeScoreB = new AutonomousBasePD(new Pose2d(56.069, 200.046, new Rotation2d(0)), new Pose2d(278.999, 180.683, new Rotation2d(0)), new Pose2d(56.069, 174.725, new Rotation2d(0)), new Pose2d(207.006, 174.725, new Rotation2d(0)), new Pose2d(278.006, 133.515, new Rotation2d(0)), new Pose2d(200.552, 185.151, new Rotation2d(0)), new Pose2d(57.062, 154.368, new Rotation2d(0)));
  private AutonomousBasePD engageChargeB = new AutonomousBasePD(new Pose2d(56.069, 108.015, new Rotation2d(0)), new Pose2d(152.812, 108.015, new Rotation2d(0)), new Pose2d(152.812, 108.015, new Rotation2d(0)), new Pose2d(152.812, 108.015, new Rotation2d(0)), new Pose2d(152.812, 108.015, new Rotation2d(0)), new Pose2d(152.812, 108.015, new Rotation2d(0)), new Pose2d(152.812, 108.015, new Rotation2d(0)));



  // red alliance  
  // half the field (325.8415) - blue x value + half the field (325.8415) = red x value
  private AutonomousBasePD HDplaceTwoEngageR = new AutonomousBasePD(new Pose2d(595.614, 20.19, new Rotation2d(0)), new Pose2d(372.684, 36.19, new Rotation2d(0)), new Pose2d(394.033, 64.004, new Rotation2d(0)), new Pose2d(494.824, 83.368, new Rotation2d(0)), new Pose2d(494.824, 83.368, new Rotation2d(0)), new Pose2d(494.824, 83.368, new Rotation2d(0)), new Pose2d(494.824, 83.368, new Rotation2d(0)));
  private AutonomousBasePD HBLeaveR = new AutonomousBasePD(new Pose2d(595.614, 200.046, new Rotation2d(0)), new Pose2d(431.768, 200.046, new Rotation2d(0)), new Pose2d(431.768, 200.046, new Rotation2d(0)), new Pose2d(431.768, 200.046, new Rotation2d(0)), new Pose2d(431.768, 200.046, new Rotation2d(0)), new Pose2d(431.768, 200.046, new Rotation2d(0)), new Pose2d(431.768, 200.046, new Rotation2d(0)));
  private AutonomousBasePD HDLeaveR = new AutonomousBasePD(new Pose2d(595.614, 20.19, new Rotation2d(0)), new Pose2d(431.768, 20.19, new Rotation2d(0)), new Pose2d(431.768, 20.19, new Rotation2d(0)), new Pose2d(431.768, 20.19, new Rotation2d(0)), new Pose2d(431.768, 20.19, new Rotation2d(0)), new Pose2d(431.768, 20.19, new Rotation2d(0)), new Pose2d(431.768, 20.19, new Rotation2d(0)));
  private AutonomousBasePD HDThreeScoreR = new AutonomousBasePD(new Pose2d(595.614, 20.19, new Rotation2d(0)), new Pose2d(372.684, 37.193, new Rotation2d(0)), new Pose2d(595.461, 43.068, new Rotation2d(0)), new Pose2d(454.199,45.934, new Rotation2d(0)), new Pose2d(372.606, 85.622, new Rotation2d(0)), new Pose2d(454.199,40.000, new Rotation2d(0)), new Pose2d(595.529,66.117, new Rotation2d(0)));
  private AutonomousBasePD HBThreeScoreR = new AutonomousBasePD(new Pose2d(595.614, 200.046, new Rotation2d(0)), new Pose2d(372.684, 180.683, new Rotation2d(0)), new Pose2d(595.614, 174.725, new Rotation2d(0)), new Pose2d(444.677, 174.725, new Rotation2d(0)), new Pose2d(373.677, 133.515, new Rotation2d(0)), new Pose2d(451.131, 185.151, new Rotation2d(0)), new Pose2d(594.621, 154.368, new Rotation2d(0)));
  private AutonomousBasePD engageChargeR = new AutonomousBasePD(new Pose2d(553.924, 108.015, new Rotation2d(0)), new Pose2d(553.924, 108.015, new Rotation2d(0)), new Pose2d(553.924, 108.015, new Rotation2d(0)), new Pose2d(553.924, 108.015, new Rotation2d(0)), new Pose2d(553.924, 108.015, new Rotation2d(0)), new Pose2d(553.924, 108.015, new Rotation2d(0)), new Pose2d(553.924, 108.015, new Rotation2d(0)));
 
*/
 public static ShuffleboardTab tab = DrivetrainSubsystem.tab;
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
    m_chooser.setDefaultOption("Default testing auto", testPath);
    /*m_chooser.addOption("Nothing! No go!", noGo);
    m_chooser.addOption("Leave community from Hot Dog", HDLeaveB);
    m_chooser.addOption("Leave community from HamBurger", HBLeaveB); 
    m_chooser.addOption("Timed auto", timedPath);

    SmartDashboard.putData("Auto choices", m_chooser);
   */
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
    SmartDashboard.putBoolean("Ready to Score", limeLightSubsystem.seeSomething());
    //m_field.setRobotPose(DrivetrainSubsystem.m_odometry.getEstimatedPosition());
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
      //m_AprilTagSubsystem.init();
    //m_drivetrainSubsystem.m_frontLeftModule.getCANCoder().getPosition();
    //System.out.println("Error code" + m_drivetrainSubsystem.m_frontLeftModule.getCANCoder().getLastError());

    m_autoSelected = m_chooser.getSelected();
    m_autoSelected.init();
    //m_AprilTagSubsystem.init();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
     //m_AprilTagSubsystem.periodic();
     m_autoSelected.periodic();
     //System.out.println("Odometry: "+ DrivetrainSubsystem.m_odometry.getPoseMeters());
    
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    //m_drivetrainSubsystem.resetOdometry(new Pose2d(0.0,0.0, new Rotation2d(Math.toRadians(0.0))));
    m_AprilTagSubsystem.init();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    //m_drivetrainSubsystem.driveTeleop();
    //System.out.println("i am in teleop");
    m_AprilTagSubsystem.periodic();
    //m_AprilTagSubsystem.detectTag();

    if(OI.m_controller.getBackButton()){
      //m_VisionSubsystem.setState(VisionSubsystem.VisionStates.DETECTTAPE);
    }

    if(OI.m_controller.getStartButton()){
      //m_VisionSubsystem.setState(VisionSubsystem.VisionStates.DETECTAPRILTAG);
    }

    //low level
    if(OI.m_controller.getLeftStickButton()){
      VisionSubsystem.level = 0;
    }

    //middle level
    if(OI.m_controller.getRightStickButton()){
      VisionSubsystem.level = 1;
    }

    //high level
    if(OI.m_controller.getXButton()){
      VisionSubsystem.level = 2;
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
    m_drivetrainSubsystem.resetOdometry(new Pose2d(0.0, 0.0, new Rotation2d(Math.toRadians(0.0))));
    //System.out.println("Trajectory: " + Trajectories.uno);
    //m_drivetrainSubsystem.zeroGyroscope();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic(){
    noGo.driveDesiredDistance(new Pose2d(-20*Constants.METERS_PER_INCH, 20*Constants.METERS_PER_INCH, new Rotation2d(0.0)));
    m_drivetrainSubsystem.drive();

    //m_drivetrainSubsystem.resetOdometry(new Pose2d(0.0, 0.0, new Rotation2d(Math.toRadians(0.0))));
    //m_drivetrainSubsystem.setSpeed(ChassisSpeeds.fromFieldRelativeSpeeds(0.2, 0, 0.0, m_drivetrainSubsystem.getGyroscopeRotation()));
    //m_drivetrainSubsystem.drive();
    //System.out.println("Our pose: " + DrivetrainSubsystem.m_pose);
    //m_drivetrainSubsystem.m_frontLeftModule.getCANCoder().getPosition();
    //System.out.println("Error code" + m_drivetrainSubsystem.m_frontLeftModule.getCANCoder().getLastError());
    /*if (m_drivetrainSubsystem.m_frontLeftModule.getCANCoder().getLastError() != ErrorCode.OK) {
      return;
    }*/

    /*intial test!*/
    //m_drivetrainSubsystem.setSpeed(new ChassisSpeeds(0, 0.2, 0));
    
    //m_drivetrainSubsystem.drive();
    
    //System.out.println(m_drivetrainSubsystem.getGyroscopeRotation());
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
  
}
