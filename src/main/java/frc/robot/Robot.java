// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.Mechanisms;
import frc.robot.subsystems.PneumaticArmPivot;
import frc.robot.subsystems.PneumaticArmPivot.PneumaticPivotStates;
import frc.robot.subsystems.PneumaticIntakeSubsystem.PneumaticIntakeStates;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.autonomous.AutonomousBase;
import frc.robot.autonomous.AutonomousBaseTimed;
import frc.robot.autonomous.AutonomousBasePD;
import frc.robot.autonomous.PDPath;
import frc.robot.subsystems.ArmTelescopingSubsystem;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */


//private AutonomousBasePD mScore = new AutonomousBasePD(new Translation2d(222.037, 0), new Translation2d(135.091, -41.307), new Translation2d(0, -44.163), new Translation2d(222.894, -50.377), new Translation2d(0, -65.388), new Translation2d(0, -65.388));

public class Robot extends TimedRobot {

  private AutonomousBase m_auto;
  private static final Boolean red = false;
  private static final Boolean blue = true;
  // DoubleSolenoid solenoidOne = new DoubleSolenoid(10, PneumaticsModuleType.REVPH, 4, 3); 

  private final SendableChooser<PDPath.AUTO_OPTIONS> auto_chooser = new SendableChooser<>();
  private final SendableChooser<Boolean> inverted = new SendableChooser<>();
  private final SendableChooser<Boolean> allianceChooser = new SendableChooser<>();

  public static final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem(); //if anything breaks in the future it might be this
  public static Mechanisms m_mechanisms = new Mechanisms();
  public static Buttons m_buttons = new Buttons();

  double mpi = Constants.METERS_PER_INCH;
  public static Boolean isBlueAlliance = true;
 
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

    //TODO: make the names come from the enum with a for loop cycling through everything in it?
    auto_chooser.setDefaultOption("testPath", PDPath.AUTO_OPTIONS.TESTPATH);
    auto_chooser.addOption("noGoR!", PDPath.AUTO_OPTIONS.NOGO);
    auto_chooser.addOption("HDLeaveB", PDPath.AUTO_OPTIONS.HDLEAVEB);
    auto_chooser.addOption("HBLeaveB", PDPath.AUTO_OPTIONS.HBLEAVEB);
    auto_chooser.addOption("lowHDPlaceLeaveB", PDPath.AUTO_OPTIONS.LOWHDPLACELEAVEB);
    auto_chooser.addOption("lowHBPlaceLeaveB", PDPath.AUTO_OPTIONS.LOWHBPLACELEAVEB);
    auto_chooser.addOption("midHDPlaceLeaveB", PDPath.AUTO_OPTIONS.MIDHDPLACELEAVEB);
    auto_chooser.addOption("midHBPlaceLeaveB", PDPath.AUTO_OPTIONS.MIDHBPLACELEAVEB);
    auto_chooser.addOption("lowTimedEngaged",PDPath.AUTO_OPTIONS.LOWTIMEDENGAGED);
    auto_chooser.addOption("midTimedEngaged",PDPath.AUTO_OPTIONS.MIDTIMEDENGAGED);
    auto_chooser.addOption("driveTimedEngaged",PDPath.AUTO_OPTIONS.DRIVETIMEDENGAGED);
    auto_chooser.addOption("lowOverEngage", PDPath.AUTO_OPTIONS.LOW_OVER_ENGAGE);
    auto_chooser.addOption("midOverEngage", PDPath.AUTO_OPTIONS.MID_OVER_ENGAGE);
    auto_chooser.addOption("timed", PDPath.AUTO_OPTIONS.TIMED);
    inverted.setDefaultOption("true", true);
    inverted.addOption("false", false);
    SmartDashboard.putData("Auto choices", auto_chooser);
    SmartDashboard.putData("telecope inverted", inverted);
   
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
    System.out.println("Auto INIT");
    m_mechanisms.init();
    m_drivetrainSubsystem.init();
    System.out.println("current pose: " + m_drivetrainSubsystem.getMPoseX() + " , " + m_drivetrainSubsystem.getMPoseY());
    PDPath.AUTO_OPTIONS selected = auto_chooser.getSelected();
    m_auto = PDPath.constructAuto(selected);
    //m_mechanisms.elevatorSubsystem.setZeroForAutoHeight();
    //m_auto.init();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
     m_mechanisms.periodic();
     m_auto.periodic();
     m_drivetrainSubsystem.drive();
     
     //System.out.println("Odometry: "+ DrivetrainSubsystem.m_odometry.getPoseMeters());
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() { //BEFORE TESTING: MAKE SURE YOU HAVE EITHER DEPLOYED OR ADDED DRIVETRAIN INIT
    isBlueAlliance = allianceChooser.getSelected();
    m_mechanisms.init(); 
   //m_drivetrainSubsystem.init();
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() { 
    m_buttons.buttonsPeriodic();
    m_mechanisms.periodic();
    m_drivetrainSubsystem.driveTeleop();
    m_drivetrainSubsystem.drive();
    m_mechanisms.armTelescopingSubsystem.setTelescopeInversion(inverted.getSelected());
   
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
    m_drivetrainSubsystem.init();
    //m_mechanisms.init();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    //m_drivetrainSubsystem.setSpeed(ChassisSpeeds.fromFieldRelativeSpeeds(0.4, 0, 0, m_drivetrainSubsystem.getPoseRotation()));
    //m_drivetrainSubsystem.pitchBalace(0.0);
    
    //OFFSETS
    m_drivetrainSubsystem.setSpeed(ChassisSpeeds.fromFieldRelativeSpeeds(0.2, 0, 0, m_drivetrainSubsystem.getPoseRotation()));
    m_drivetrainSubsystem.drive();

    //MECHANISMS
    m_mechanisms.periodic();
    m_buttons.buttonsPeriodic();
    //PneumaticArmPivot.solenoid.set(Value.kForward);
    //System.out.println(m_mechanisms.armTelescopingSubsystem.getArmPosition());
    m_mechanisms.armTelescopingSubsystem.telescopingMotor.setSelectedSensorPosition(0.0);
    m_mechanisms.elevatorSubsystem.elevatorMotor.setSelectedSensorPosition(0.0);
  }
  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}