// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.Mechanisms;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.autonomous.AutonomousBase;
import frc.robot.autonomous.AutonomousBaseTimed;//made changes to this on joanne's computer
import frc.robot.autonomous.AutonomousBasePD;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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
  // DoubleSolenoid solenoidOne = new DoubleSolenoid(10, PneumaticsModuleType.REVPH, 4, 3); 

  private final SendableChooser<AutonomousBase> auto_chooser = new SendableChooser<>();
  private final SendableChooser<Boolean> allianceChooser = new SendableChooser<>();

  public static final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem(); //if anything breaks in the future it might be this
  public static Mechanisms m_mechanisms = new Mechanisms();
  public static Buttons m_buttons = new Buttons();

  double mpi = Constants.METERS_PER_INCH;
  public static Boolean isBlueAlliance = true;
  
  private AutonomousBaseTimed timedPath = new AutonomousBaseTimed();
  private AutonomousBasePD testPath = PDPath.HDLeaveR;
 
// red alliance  
// half the field (325.8415) - blue x value + half the field (325.8415) = red x value
 // whole field: 651.683 (inches)
  
  /**
  * This function is run when the robot is turned on and should be used for any
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
    auto_chooser.addOption("HDPlaceLeaveB", PDPath.HDPlaceLeaveB);
    auto_chooser.addOption("HBPlaceLeaveB", PDPath.HBPlaceLeaveB);
    auto_chooser.addOption("HDPlaceLeaveR", PDPath.HDPlaceLeaveR); 
    auto_chooser.addOption("HBPlaceLeaveR", PDPath.HBPlaceLeaveR);
    auto_chooser.addOption("engageChargeR", PDPath.engageChargeR);
    auto_chooser.addOption("engageChargeB", PDPath.engageChargeB);
    // auto_chooser.addOption("HDIntakeEngageB", PDPath.HDIntakeEngageB);
    // auto_chooser.addOption("HDIntakeEngageR", PDPath.HDIntakeEngageR);
    // auto_chooser.addOption("HD3ScoreR", PDPath.HD3ScoreR);
    // auto_chooser.addOption("HD3ScoreB", PDPath.HD3ScoreB);
    // auto_chooser.addOption("HB3ScoreR", PDPath.HB3ScoreR);
    // auto_chooser.addOption("HB3ScoreB", PDPath.HB3ScoreB);
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
    SmartDashboard.putNumber("x odometry",m_drivetrainSubsystem.getMPoseX()/Constants.METERS_PER_INCH);
    SmartDashboard.putNumber("y odometry",m_drivetrainSubsystem.getMPoseY()/Constants.METERS_PER_INCH);
    SmartDashboard.putNumber("angle odometry",m_drivetrainSubsystem.getMPoseDegrees()%360);
    //SmartDashboard.putBoolean("Ready to Score", m_limeLightSubsystem.seeSomething());
    SmartDashboard.putBoolean("beam broken?", m_mechanisms.pneumaticIntakeSubsystem.isBeamBroken());
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
    m_drivetrainSubsystem.init();
    System.out.println("current pose: " + m_drivetrainSubsystem.getMPoseX() + " , " + m_drivetrainSubsystem.getMPoseY());
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
  public void teleopInit() { //BEFORE TESTING: MAKE SURE YOU HAVE EITHER DEPLOYED OR ADDED DRIVETRAIN INIT
    isBlueAlliance = allianceChooser.getSelected();
    m_mechanisms.init();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    m_buttons.buttonsPeriodic();
    m_drivetrainSubsystem.driveTeleop(); //only sets speed; does not actually drive
    m_drivetrainSubsystem.drive();
    m_mechanisms.periodic();
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}