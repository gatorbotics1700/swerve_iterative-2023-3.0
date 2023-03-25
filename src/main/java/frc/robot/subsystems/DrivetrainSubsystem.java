// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import frc.com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import frc.com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import frc.com.swervedrivespecialties.swervelib.SwerveModule;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Robot;


import java.util.function.DoubleSupplier;

import static frc.robot.Constants.*;

import frc.robot.Constants;
import frc.robot.OI;

public class DrivetrainSubsystem {
   private static final double pitchKP = 0.035; //0.025;
   private static final double pitchKI = 0.0;
   private static final double pitchKD = 0.001; //0.001;
   private PIDController pitchController;
   private static final double MINOUTPUT = 0.1;
        

  /**
   * The maximum voltage that will be delivered to the motors.
   * <p>
   * This can be reduced to cap the robot's maximum speed. Typically, this is useful during initial testing of the robot.
   */
  private static final double MAX_VOLTAGE = 16.3;
  //  The formula for calculating the theoretical maximum velocity is:
  //   <Motor free speed RPM> / 60 * <Drive reduction> * <Wheel diameter meters> * pi
  //  By default this value is setup for a Mk3 standa
 // rd module using Falcon500s to drive.
  //  An example of this constant for a Mk4 L2 module with NEOs to drive is:
  //   5880.0 / 60.0 / SdsModuleConfigurations.MK4_L2.getDriveReduction() * SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI
  /**
   * The maximum velocity of the robot in meters per second.
   * <p>
   * This is a measure of how fast the robot should be able to drive in a straight line.
   */
  private static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 *
          SdsModuleConfigurations.MK4_L2.getDriveReduction() *
          SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI;
          // = 5.38281261
  /**
   * The maximum angular velocity of the robot in radians per second.
   * <p>
   * This is a measure of how fast the robot can rotate in place.
   */
  // Here we calculate the theoretical maximum angular velocity. You can also replace this with a measured amount.
  private static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
          Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);

  private SwerveDriveKinematics m_kinematics;

  // By default we use a Pigeon for our gyroscope. But if you use another gyroscope, like a NavX, you can change this.
  // The important thing about how you configure your gyroscope is that rotating the robot counter-clockwise should
  // cause the angle reading to increase until it wraps back over to zero.

  private PigeonIMU m_pigeon;
  // These are our modules. We initialize them in the constructor.
  private SwerveModule m_frontLeftModule;
  private SwerveModule m_frontRightModule;
  private SwerveModule m_backLeftModule;
  private SwerveModule m_backRightModule;

  private SwerveDrivePoseEstimator m_odometry; 
  private Pose2d m_pose;
  private ShuffleboardTab tab;

  //ChassisSpeeds takes in y velocity, x velocity, speed of rotation
  private ChassisSpeeds m_chassisSpeeds;

  public DrivetrainSubsystem() {
        init();
  }

  public void init(){
        System.out.println("Initializing drivetrain subsystem vars");
        pitchController = new PIDController(pitchKP, pitchKI, pitchKD);
        m_kinematics = new SwerveDriveKinematics(
          // Setting up location of modules relative to the center of the robot
          // Front left
          new Translation2d(DRIVETRAIN_WHEELBASE_METERS / 2.0, DRIVETRAIN_TRACKWIDTH_METERS / 2.0),
          //translation2d refers to the robot's x and y position in the larger field coordinate system
          // Front right
          new Translation2d(DRIVETRAIN_WHEELBASE_METERS / 2.0, -DRIVETRAIN_TRACKWIDTH_METERS / 2.0), 
          // Back left
          new Translation2d(-DRIVETRAIN_WHEELBASE_METERS / 2.0, DRIVETRAIN_TRACKWIDTH_METERS / 2.0),
          // Back right
          new Translation2d(-DRIVETRAIN_WHEELBASE_METERS / 2.0, -DRIVETRAIN_TRACKWIDTH_METERS / 2.0)
        );
        m_pigeon = new PigeonIMU(DRIVETRAIN_PIGEON_ID);

        m_pose = new Pose2d();

        tab = Shuffleboard.getTab("Drivetrain");

        m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
        
        // We will use mk4 modules with Falcon 500s with the L2 configuration. 
        m_frontLeftModule = Mk4SwerveModuleHelper.createFalcon500(
            // This parameter is optional, but will allow you to see the current state of the module on the dashboard.
            tab.getLayout("Front Left Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(0, 0),
            // This can be any level from L1-L4 depending on the gear configuration (the levels allow different amounts of speed and torque)
            Mk4SwerveModuleHelper.GearRatio.L2,
            // This is the ID of the drive motor
            FRONT_LEFT_MODULE_DRIVE_MOTOR,
            // This is the ID of the steer motor
            FRONT_LEFT_MODULE_STEER_MOTOR,
            // This is the ID of the steer encoder
            FRONT_LEFT_MODULE_STEER_ENCODER,
            // This is how much the steer encoder is offset from true zero (In our case, zero is facing straight forward)
            FRONT_LEFT_MODULE_STEER_OFFSET
    );

    // We will do the same for the other modules
    //TODO: check if we want to construct on every enable
    m_frontRightModule = Mk4SwerveModuleHelper.createFalcon500(
            tab.getLayout("Front Right Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(2, 0),
            Mk4SwerveModuleHelper.GearRatio.L2,
            FRONT_RIGHT_MODULE_DRIVE_MOTOR,
            FRONT_RIGHT_MODULE_STEER_MOTOR,
            FRONT_RIGHT_MODULE_STEER_ENCODER,
            FRONT_RIGHT_MODULE_STEER_OFFSET
    );

    m_backLeftModule = Mk4SwerveModuleHelper.createFalcon500(
            tab.getLayout("Back Left Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(4, 0),
            Mk4SwerveModuleHelper.GearRatio.L2,
            BACK_LEFT_MODULE_DRIVE_MOTOR,
            BACK_LEFT_MODULE_STEER_MOTOR,
            BACK_LEFT_MODULE_STEER_ENCODER,
            BACK_LEFT_MODULE_STEER_OFFSET
    );

    m_backRightModule = Mk4SwerveModuleHelper.createFalcon500(
            tab.getLayout("Back Right Module", BuiltInLayouts.kList)
                    .withSize(2, 4)
                    .withPosition(6, 0),
            Mk4SwerveModuleHelper.GearRatio.L2,
            BACK_RIGHT_MODULE_DRIVE_MOTOR,
            BACK_RIGHT_MODULE_STEER_MOTOR,
            BACK_RIGHT_MODULE_STEER_ENCODER,
            BACK_RIGHT_MODULE_STEER_OFFSET
    );
    
    m_odometry = new SwerveDrivePoseEstimator(
        m_kinematics, 
        getGyroscopeRotation(), 
        new SwerveModulePosition[] {
                m_frontLeftModule.getSwerveModulePosition(), 
                m_frontRightModule.getSwerveModulePosition(), 
                m_backLeftModule.getSwerveModulePosition(), 
                m_backRightModule.getSwerveModulePosition()
        }, 
        new Pose2d(0, 0, new Rotation2d(Math.toRadians(180)))); //assumes 180 degrees rotation is facing driver station
  }
  
  //from pigeon used for updating our odometry
  //in an unknown, arbitrary frame
  //"do not use unless you know what you are doing" - patricia
  private Rotation2d getGyroscopeRotation() {
        return new Rotation2d(Math.toRadians(m_pigeon.getYaw()));
  }
  //from odometry used for field-relative rotation
  public Rotation2d getPoseRotation() {
        return m_pose.getRotation(); 
  }

  public void resetOdometry(Pose2d start){
        SwerveModulePosition[] positionArray =  new SwerveModulePosition[] {
                new SwerveModulePosition(m_frontLeftModule.getPosition()/Constants.TICKS_PER_METER, new Rotation2d(m_frontLeftModule.getSteerAngle())),
                new SwerveModulePosition(m_frontRightModule.getPosition()/Constants.TICKS_PER_METER, new Rotation2d(m_frontRightModule.getSteerAngle())), 
                new SwerveModulePosition(m_backLeftModule.getPosition()/Constants.TICKS_PER_METER, new Rotation2d(m_backLeftModule.getSteerAngle())),
                new SwerveModulePosition(m_backRightModule.getPosition()/Constants.TICKS_PER_METER, new Rotation2d(m_backRightModule.getSteerAngle()))};
        m_pose = start; //TODO: taken from elsewhere, confirm why we do this
        //System.out.println("position array: " + positionArray.toString());
        //System.out.println("m_pose: " + m_pose.getX() + ", " + m_pose.getY() + ", " + m_pose.getRotation().getDegrees());
        m_odometry.resetPosition(getGyroscopeRotation(), positionArray, m_pose);
        //System.out.println("#resetodometry! new pose: " + m_pose.getX() + " y: " + m_pose.getY());
        m_pose = m_odometry.update(getGyroscopeRotation(), positionArray);
        //System.out.println("m_pose after update in odometry: " + m_pose.getX() + ", " + m_pose.getY() + ", " + m_pose.getRotation().getDegrees());
        //System.out.println("inputs for the reset: " + getGyroscopeRotation() + " " + m_frontLeftModule.getSwerveModulePosition().distanceMeters + " " + m_frontRightModule.getSwerveModulePosition().distanceMeters + " " + m_backLeftModule.getSwerveModulePosition().distanceMeters + " " + m_backRightModule.getSwerveModulePosition().distanceMeters);
}
       // System.out.println("#resetodometry! new pose: " + m_pose.getX()/SWERVE_TICKS_PER_INCH + " y: " + m_pose.getY()/SWERVE_TICKS_PER_INCH);
       // System.out.println("inputs for the reset: " + getGyroscopeRotation() + m_frontLeftModule.getSwerveModulePosition().distanceMeters + m_frontRightModule.getSwerveModulePosition().distanceMeters + m_backLeftModule.getSwerveModulePosition().distanceMeters + m_backRightModule.getSwerveModulePosition().distanceMeters);

  public void setSpeed(ChassisSpeeds chassisSpeeds) {
        m_chassisSpeeds = chassisSpeeds;
  }
  
  public void driveTeleop(){
        DoubleSupplier m_translationXSupplier;
        DoubleSupplier m_translationYSupplier;
        DoubleSupplier m_rotationSupplier;
        //TODO: check negative signs
        m_translationXSupplier = () -> -modifyAxis(OI.m_controller.getLeftY()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND;
        m_translationYSupplier = () -> -modifyAxis(OI.m_controller.getLeftX()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND;
        m_rotationSupplier = () -> -modifyAxis(OI.m_controller.getRightX()) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;
        setSpeed(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        m_translationXSupplier.getAsDouble(),
                        m_translationYSupplier.getAsDouble(),
                        m_rotationSupplier.getAsDouble(),
                        getPoseRotation()
                )
        );
        //TODO: check whether drive() is necessary
  }

  public void drive() { //runs periodically
        //System.out.println("pose before update: " + m_pose.getX()/TICKS_PER_INCH + " and y: " + m_pose.getY()/TICKS_PER_INCH);
        //TODO: check getSteerAngle() is correct and that we shouldn't be getting from cancoder
        SwerveModulePosition[] array =  {
                new SwerveModulePosition(m_frontLeftModule.getPosition()/Constants.TICKS_PER_METER, new Rotation2d(m_frontLeftModule.getSteerAngle())), //from steer motor
                new SwerveModulePosition(m_frontRightModule.getPosition()/Constants.TICKS_PER_METER, new Rotation2d(m_frontRightModule.getSteerAngle())), 
                new SwerveModulePosition(m_backLeftModule.getPosition()/Constants.TICKS_PER_METER, new Rotation2d(m_backLeftModule.getSteerAngle())),
                new SwerveModulePosition(m_backRightModule.getPosition()/Constants.TICKS_PER_METER, new Rotation2d(m_backRightModule.getSteerAngle()))
        };
        m_pose = m_odometry.update(getGyroscopeRotation(),array); 

        //array of states filled with the speed and angle for each module (made from linear and angular motion for the whole robot) 
        SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);
        //desaturatewheelspeeds checks and fixes if any module's wheel speed is above the max
        SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);

        m_frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[0].angle.getRadians());
        m_frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[1].angle.getRadians());
        m_backLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[2].angle.getRadians());
        m_backRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[3].angle.getRadians());

        // System.out.println("The angle of front left module: "+states[0].angle.getDegrees());
        // System.out.println("The angle of front right module: "+states[1].angle.getDegrees());
}

  private static double deadband(double value, double deadband) {
        if (Math.abs(value) > deadband) {
          if (value > 0.0) {
            return (value - deadband) / (1.0 - deadband);
          } else {
            return (value + deadband) / (1.0 - deadband);
          }
        } else {
          return 0.0;
        }
      }
    
   private static double modifyAxis(double value) {
        // Deadband
        value = deadband(value, 0.05);

        // Square the axis
        value = Math.copySign(value * value, value);

        return value;
    }

    //AUTO AND FAILSAFE
    public void stopDrive() {
        setSpeed(ChassisSpeeds.fromFieldRelativeSpeeds(0.0, 0.0, 0.0, getPoseRotation()));
        drive();
   }
   //TODO: look through this function
   public void pitchBalance(double pitchSetpoint){
        System.out.println("pitch: " + m_pigeon.getPitch());
        double pitchAfterCorrection = m_pigeon.getPitch();
        System.out.println("pitch after correcting for universalPitch: " + pitchAfterCorrection);
        pitchController.setSetpoint(pitchSetpoint); 
        double output = pitchController.calculate(pitchAfterCorrection, pitchSetpoint);
        System.out.println("output: " + output); 
        setSpeed(ChassisSpeeds.fromFieldRelativeSpeeds(output, 0, 0, getGyroscopeRotation()));
        drive();
        
        if (Math.abs(pitchAfterCorrection - pitchSetpoint) < 1.0){
            setSpeed(ChassisSpeeds.fromFieldRelativeSpeeds(0, 0, 0,getGyroscopeRotation()));
            //velocityPD(0);
        } else{
                if(Math.abs(output) < MINOUTPUT){
                   output = Math.signum(output) * MINOUTPUT;
                }
        }
    }

    public double getMPoseX(){
        return m_pose.getX();
    }

    public double getMPoseY(){
        return m_pose.getY();
    }

    public double getMPoseDegrees(){
        return m_pose.getRotation().getDegrees();
    }

    public Pose2d getMPose(){ //TODO: do we need this?
        return m_pose;
    }

    public double getPitch(){
        return m_pigeon.getPitch();
    }
   
}
