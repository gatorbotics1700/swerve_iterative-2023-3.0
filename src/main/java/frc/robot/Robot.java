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
import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTable.*;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.ArmTelescopingSubsystem.TelescopingStates;
import frc.robot.subsystems.ElevatorSubsystem.ElevatorStates;
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
  public static final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem(); //if anything breaks in the future it might be this
  private static ArmTelescopingSubsystem armTelescopingSubsystem = new ArmTelescopingSubsystem();
  private static ElevatorSubsystem elevatorSubsystem = new ElevatorSubsystem();
  private final Field2d m_field = new Field2d();
  double t= 0.0;
  ChassisSpeeds m_ChassisSpeeds;
  double mpi = Constants.METERS_PER_INCH;

 
  // whole field: 651.683 (inches)
  // center : 325.8415 (inches)
  // private AutonomousBasePD noGo = new AutonomousBasePD(new Pose2d(0, 0, new Rotation2d(0)), new Pose2d(0, 0, new Rotation2d(0)), new Pose2d(0, 0, new Rotation2d(0)), new Pose2d(0,0, new Rotation2d(0)), new Pose2d(0, 0, new Rotation2d(0)), new Pose2d(0,0, new Rotation2d(0)), new Pose2d(0,0, new Rotation2d(0)));
  private AutonomousBaseTimed timedPath = new AutonomousBaseTimed();
  private AutonomousBasePD testPath = new AutonomousBasePD(new Pose2d(0.0, 0.0, new Rotation2d(Math.toRadians(180.0))), new Pose2d(5 * mpi, -40 * mpi, new Rotation2d(Math.toRadians(180))), new Pose2d(40 * mpi, 0, new Rotation2d(0)), new Pose2d(0, 30 * mpi, new Rotation2d(0)), new Pose2d(40 * mpi, 30 * mpi, new Rotation2d(0)), new Pose2d(0, 0, new Rotation2d(0)), new Pose2d(20 * mpi, 20 * mpi, new Rotation2d(0)), 180.0); 
  
 
// red alliance  
// half the field (325.8415) - blue x value + half the field (325.8415) = red x value
   
  PneumaticIntakeSubsystem pneumaticIntakeSubsystem = new PneumaticIntakeSubsystem();

 public static ShuffleboardTab tab = DrivetrainSubsystem.tab;
    public static GenericEntry test =
    tab.add("test", 10)
      .getEntry();
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
    
    m_chooser.setDefaultOption("testPath", testPath);
    //m_chooser.setDefaultOption("current", CurrentPath);
    // m_chooser.addOption("noGo!", noGo);
    // m_chooser.addOption("HDLeaveB", HDLeaveB);
    // m_chooser.addOption("HBLeaveB", HBLeaveB); 
    // m_chooser.addOption("timed", timedPath);
    // m_chooser.addOption("HDThreeScoreB", HDThreeScoreB);
    // m_chooser.addOption("HBThreeScoreB", HBThreeScoreB);
    // m_chooser.addOption("HDplaceTwoEngageB", HDplaceTwoEngageB);
    // m_chooser.addOption("engageChargeB", engageChargeB);
    // m_chooser.addOption("HDplaceTwoEngageR", HDplaceTwoEngageR);
    // m_chooser.addOption("HBLeaveR", HBLeaveR);
    // m_chooser.addOption("HDLeaveR", HDLeaveR);
    // m_chooser.addOption("HDThreeScoreR", HDThreeScoreR);
    // m_chooser.addOption("HBThreeScoreR", HBThreeScoreR);
    // m_chooser.addOption("engageChargeR", engageChargeR);
    // //m_chooser.addOption("Motion profiling tester path", motionProfiling);
    // SmartDashboard.putData("Auto choices", m_chooser);
   
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
 // @Override
  public void robotPeriodic() {

    SmartDashboard.putNumber("x odometry",DrivetrainSubsystem.m_pose.getX());
    SmartDashboard.putNumber("y odometry",DrivetrainSubsystem.m_pose.getY());

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
  
    // elevatorSubsystem.init();
    // elevatorSubsystem.elevatorMotor.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
    armTelescopingSubsystem.init();

    //m_drivetrainSubsystem.m_frontLeftModule.getCANCoder().getPosition();
    // System.out.println("Error code" + m_drivetrainSubsystem.m_frontLeftModule.getCANCoder().getLastError());
    // System.out.println("current pose: " + DrivetrainSubsystem.m_pose.getX() + " , " + DrivetrainSubsystem.m_pose.getY());
    // m_autoSelected = m_chooser.getSelected();
    // m_autoSelected.init();

  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    //elevatorSubsystem.setState(ElevatorStates.LOW_ELEVATOR_HEIGHT);
    //elevatorSubsystem.periodic();

    armTelescopingSubsystem.setTState(TelescopingStates.MID_ARM_LENGTH); //moved from auto periodic to init
    armTelescopingSubsystem.periodic();

    //  m_autoSelected.periodic();
     //m_drivetrainSubsystem.drive();
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    //m_drivetrainSubsystem.m_frontLeftModule.getCANCoder().getPosition();
   // System.out.println("Error code" + m_drivetrainSubsystem.m_frontLeftModule.getCANCoder().getLastError());

  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    /*System.out.println("back left module: " + m_drivetrainSubsystem.m_backLeftModule.getAbsoluteAngle());
    System.out.println("back right module: " + m_drivetrainSubsystem.m_backRightModule.getSteerAngle());
    System.out.println("front left module: " + m_drivetrainSubsystem.m_frontLeftModule.getSteerAngle());
    System.out.println("front right module: " + m_drivetrainSubsystem.m_frontRightModule.getSteerAngle());*/
    //m_drivetrainSubsystem.driveTeleop();

    //driver
if (OI.m_controller.getBButton()){ 
  m_drivetrainSubsystem.stopDrive(); //stop all mech?
}

if(OI.m_controller.getXButton()){
 // m_drivetrainSubsystem.pitchBalance(0.0);
}

//codriver

/*if (OI.m_controller_two.getPOV() == 0){
  System.out.println("dpad 0: high node");
  m_drivetrainSubsystem.scoreHigh();
}

if (OI.m_controller_two.getPOV() == 90){
  System.out.println("dpad 90: mid node");
  m_drivetrainSubsystem.scoreMid();
  
}

if (OI.m_controller_two.getPOV() == 270){
  System.out.println("dpad 270: substation");
  m_drivetrainSubsystem.substation();

}

if (OI.m_controller_two.getPOV() == 180){
  System.out.println("dpad 180: low node");
  m_drivetrainSubsystem.scoreLow();
}*/

if(OI.m_controller_two.getAButton()){ 
  System.out.println("xbox: low node");
  m_drivetrainSubsystem.scoreLow();
}

if(OI.m_controller_two.getBButton()){
  System.out.println("xbox: mid node");
  //m_drivetrainSubsystem.scoreMid();
  armTelescopingSubsystem.setTState(TelescopingStates.MID_ARM_LENGTH);
 // armTelescopingSubsystem.periodic();
}

if(OI.m_controller_two.getXButton()){
  System.out.println("xbox: substation");
  m_drivetrainSubsystem.substation();
}

if(OI.m_controller_two.getYButton()){
  System.out.println("xbox: high node");
  m_drivetrainSubsystem.scoreHigh();

}

if(OI.m_controller_two.getRightBumper()){
}

if(OI.m_controller_two.getLeftBumperReleased()){ //needs its own button & not enough
  if(PneumaticIntakeSubsystem.pneumaticIntakeState==PneumaticIntakeSubsystem.PneumaticIntakeStates.ACTUATING || PneumaticIntakeSubsystem.pneumaticIntakeState==PneumaticIntakeSubsystem.PneumaticIntakeStates.OFF){
    pneumaticIntakeSubsystem.setStatePneumaticIntake(PneumaticIntakeSubsystem.PneumaticIntakeStates.RETRACTING);
  } else if(PneumaticIntakeSubsystem.pneumaticIntakeState==PneumaticIntakeSubsystem.PneumaticIntakeStates.RETRACTING){
    pneumaticIntakeSubsystem.setStatePneumaticIntake(PneumaticIntakeSubsystem.PneumaticIntakeStates.ACTUATING); 
  }
}

if (OI.m_controller_two.getStartButton()){
  //m_drivetrainSubsystem.stopAllMech();
}

if (OI.m_controller_two.getBackButton()){

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

    // armTelescopingSubsystem.init();
    // armTelescopingSubsystem.telescopingMotor.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs); //VERY VERY IMPORTANT

    //m_drivetrainSubsystem.zeroGyroscope();

    armTelescopingSubsystem.init();
    // armTelescopingSubsystem.telescopingMotor.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs); //VERY VERY IMPORTANT
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    // elevatorSubsystem.setState(ElevatorStates.ZERO);
    // elevatorSubsystem.periodic();
    
    //armTelescopingSubsystem.telescopingMotor.set(ControlMode.PercentOutput, 0.1);
    armTelescopingSubsystem.setTState(TelescopingStates.LOW_ARM_LENGTH);
    armTelescopingSubsystem.periodic();
   
    // m_drivetrainSubsystem.setSpeed(ChassisSpeeds.fromFieldRelativeSpeeds(0.2, 0.0, Math.toRadians(0), m_drivetrainSubsystem.getGyroscopeRotation()));
    // m_drivetrainSubsystem.drive();
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}

