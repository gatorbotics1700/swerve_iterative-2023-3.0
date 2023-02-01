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
  private AutonomousBasePD noGo = new AutonomousBasePD(new Pose2d(0, 0, new Rotation2d(0)), new Pose2d(0, 0, new Rotation2d(0)), new Pose2d(0, 0, new Rotation2d(0)), new Pose2d(0,0, new Rotation2d(0)), new Pose2d(0, 0, new Rotation2d(0)), new Pose2d(0,0, new Rotation2d(0)), new Pose2d(0,0, new Rotation2d(0)));
  private AutonomousBasePD placeNLeave = new AutonomousBasePD(new Pose2d(0, 0, new Rotation2d(0)), new Pose2d(160.0, 0, new Rotation2d(0)), new Pose2d(160.0, 0, new Rotation2d(0)), new Pose2d(160.0, 0, new Rotation2d(0)), new Pose2d(160.0, 0, new Rotation2d(0)), new Pose2d(160.0, 0, new Rotation2d(0)), new Pose2d(160.0, 0, new Rotation2d(0)));
  private AutonomousBasePD antiCharge = new AutonomousBasePD(new Pose2d(86.840, -45.282, new Rotation2d(0)), new Pose2d(221.978, 19.463, new Rotation2d(0)), new Pose2d(135.091, -19.421, new Rotation2d(0)), new Pose2d(0, -22.277, new Rotation2d(0)), new Pose2d(222.491, -28.492, new Rotation2d(0)), new Pose2d(0, -43.502, new Rotation2d(0)), new Pose2d(0, -43.502, new Rotation2d(0)));
  private AutonomousBasePD antiChargeOpposite = new AutonomousBasePD(new Pose2d(86.840, 45.282, new Rotation2d(0)), new Pose2d(221.978, -19.463, new Rotation2d(0)), new Pose2d(135.091, 19.421, new Rotation2d(0)), new Pose2d(0, 22.277, new Rotation2d(0)), new Pose2d(222.491, 28.492, new Rotation2d(0)), new Pose2d(0, 43.502, new Rotation2d(0)), new Pose2d(0, 43.502, new Rotation2d(0)));
  private AutonomousBasePD engageCharge = new AutonomousBasePD(new Pose2d(97.759, 0, new Rotation2d(0)), new Pose2d(97.759, 0, new Rotation2d(0)), new Pose2d(97.759, 0, new Rotation2d(0)), new Pose2d(97.759, 0, new Rotation2d(0)), new Pose2d(97.759, 0, new Rotation2d(0)), new Pose2d(97.759, 0, new Rotation2d(0)), new Pose2d(97.759, 0, new Rotation2d(0)));
  private AutonomousBasePD placeTwoEngage = new AutonomousBasePD(new Pose2d(223.014, 16.468, new Rotation2d(0)), new Pose2d(0, 22.683, new Rotation2d(0)), new Pose2d(135.615, 25.539, new Rotation2d(0)), new Pose2d(222.191, 64.230, new Rotation2d(0)), new Pose2d(97.711, 64.230, new Rotation2d(0)), new Pose2d(97.711, 64.230, new Rotation2d(0)), new Pose2d(97.711, 64.230, new Rotation2d(0)));
  private AutonomousBaseTimed timedPath = new AutonomousBaseTimed();
  private AutonomousBasePD testPath = new AutonomousBasePD(new Pose2d(0, 0, new Rotation2d(0)), new Pose2d(0, 20, new Rotation2d(0)), new Pose2d(0, 20, new Rotation2d(0)), new Pose2d(0, 20, new Rotation2d(0)), new Pose2d(0, 20, new Rotation2d(0)), new Pose2d(0, 20, new Rotation2d(0)), new Pose2d(0, 20, new Rotation2d(0))); //first is starting position, second is where we want to go
  private AutonomousBaseMP motionProfiling = new AutonomousBaseMP(Trajectories.uno, Trajectories.dos, Trajectories.tres);
  
  //above coordinates didn't work
  private AutonomousBasePD leave = new AutonomousBasePD(new Pose2d(56.069, 17.332, new Rotation2d(0)), new Pose2d(219.915, 17.332, new Rotation2d(0)), new Pose2d(219.915, 17.332, new Rotation2d(0)), new Pose2d(219.915, 17.332, new Rotation2d(0)), new Pose2d(219.915, 17.332, new Rotation2d(0)), new Pose2d(219.915, 17.332, new Rotation2d(0)), new Pose2d(219.915, 17.332, new Rotation2d(0)));
  private AutonomousBasePD placeTwoEngageReal = new AutonomousBasePD(new Pose2d(56.069, 17.332, new Rotation2d(0)), new Pose2d(278.999, 37.193, new Rotation2d(0)), new Pose2d(257.650, 64.004, new Rotation2d(0)), new Pose2d(156.859, 83.368, new Rotation2d(0)), new Pose2d(156.859, 83.368, new Rotation2d(0)), new Pose2d(156.859, 83.368, new Rotation2d(0)), new Pose2d(156.859, 83.368, new Rotation2d(0)));

  //these paths score 3 balls without touching the charge station, requires 7 Pose2ds!
  private AutonomousBasePD threeUnderChargeStation = new AutonomousBasePD(new Pose2d(56.069, 17.332, new Rotation2d(0)), new Pose2d(278.999, 37.193, new Rotation2d(0)), new Pose2d(56.222, 43.068, new Rotation2d(0)), new Pose2d(197.484,45.934, new Rotation2d(0)), new Pose2d(279.077, 85.622, new Rotation2d(0)), new Pose2d(197.484,40.000, new Rotation2d(0)), new Pose2d(56.154,66.117, new Rotation2d(0)));
  private AutonomousBasePD threeAboveChargeStation = new AutonomousBasePD(new Pose2d(56.069, 200.046, new Rotation2d(0)), new Pose2d(278.999, 180.683, new Rotation2d(0)), new Pose2d(56.069, 174.725, new Rotation2d(0)), new Pose2d(207.006, 174.725, new Rotation2d(0)), new Pose2d(278.006, 133.515, new Rotation2d(0)), new Pose2d(200.552, 185.151, new Rotation2d(0)), new Pose2d(57.062, 154.368, new Rotation2d(0)));

  
  public static final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem(); //if anything breaks in the future it might be this
  private final Field2d m_field = new Field2d();
  ChassisSpeeds m_ChassisSpeeds;

  // static ShuffleboardTab tab = DrivetrainSubsystem.tab;
  //sprivate static NetworkTableEntry kP = tab.add("Auto kP", 0.1).getEntry(); 
  //public static GenericEntry kI = tab.add("Auto kI", 0.0).getEntry(); 
  //public static GenericEntry kD = tab.add("Auto kD", 0.0).getEntry();

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  //class AutoPID{
   
    
  //}
 

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
  
  @Override
  public void robotInit() { //creates options for different autopaths, names are placeholders
   // kP = tab.add("Auto kP", 0.1).getEntry(); 
    //kI = tab.add("Auto kI", 0.0).getEntry(); 
    //kD = tab.add("Auto kD", 0.0).getEntry();

      // private ShuffleboardTab tab = Shuffleboard.getTab("Drive");
      // private NetworkTableEntry kP = tab.add("Auto kP", 0.1).getEntry();
   
      // private DifferentialDrive robotDrive = ...;
   
      // public void drive(double left, double right) {
      //   // Retrieve the maximum speed from the dashboard
      //   double max = maxSpeed.getDouble(1.0);
      //   robotDrive.tankDrive(left * max, right * max);
      // }
  

    System.out.println("#I'm Awake");
    m_chooser.setDefaultOption("Default Auto", testPath);
    m_chooser.addOption("My Auto 1", noGo);
    m_chooser.addOption("My Auto 2",placeNLeave);
    m_chooser.addOption("My Auto 3", antiCharge);
    m_chooser.addOption("My Auto 4", antiChargeOpposite);
    m_chooser.addOption("My Auto 5", engageCharge);
    m_chooser.addOption("My Auto 6",placeTwoEngage);
    m_chooser.addOption("My Auto timed", timedPath);
    m_chooser.addOption("Motion profiling path", motionProfiling);
    m_chooser.addOption("three under charge station", threeUnderChargeStation);
    m_chooser.addOption("three above charge station", threeAboveChargeStation);

    SmartDashboard.putData("Auto choices", m_chooser);

    
   

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
    m_autoSelected = m_chooser.getSelected();
    m_autoSelected.init();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    
     m_autoSelected.periodic();
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
    if(OI.m_controller.getAButton()){
      m_drivetrainSubsystem.resetOdometry(new Pose2d());
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
    System.out.println("Trajectory: " + Trajectories.uno);
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {

    /*intial test!*/
    m_drivetrainSubsystem.setSpeed(new ChassisSpeeds(-0.2, -0.2, 0));
    
    m_drivetrainSubsystem.drive();
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
