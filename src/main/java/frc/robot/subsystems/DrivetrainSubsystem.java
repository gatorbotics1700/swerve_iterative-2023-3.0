// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import frc.com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import frc.com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import frc.com.swervedrivespecialties.swervelib.SwerveModule;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.estimator.*;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.controller.PIDController;

import java.util.Arrays;
import java.util.function.DoubleSupplier;

import static frc.robot.Constants.*;
import frc.robot.OI;

public class DrivetrainSubsystem {

  /**
   * The maximum voltage that will be delivered to the motors.
   * <p>
   * This can be reduced to cap the robot's maximum speed. Typically, this is useful during initial testing of the robot.
   */
  public static final double MAX_VOLTAGE = 16.3;
  //  The formula for calculating the theoretical maximum velocity is:
  //   <Motor free speed RPM> / 60 * <Drive reduction> * <Wheel diameter meters> * pi
  //  By default this value is setup for a Mk3 standard module using Falcon500s to drive.
  //  An example of this constant for a Mk4 L2 module with NEOs to drive is:
  //   5880.0 / 60.0 / SdsModuleConfigurations.MK4_L2.getDriveReduction() * SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI
  /**
   * The maximum velocity of the robot in meters per second.
   * <p>
   * This is a measure of how fast the robot should be able to drive in a straight line.
   */
  public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 *
          SdsModuleConfigurations.MK4_L2.getDriveReduction() *
          SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI;
          // = 5.38281261
  /**
   * The maximum angular velocity of the robot in radians per second.
   * <p>
   * This is a measure of how fast the robot can rotate in place.
   */
  // Here we calculate the theoretical maximum angular velocity. You can also replace this with a measured amount.
  public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
          Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);

  public final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
          // Setting up location of modules relative to the center of the robot
          // Front left
          new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
          //translation2d refers to the robot's x and y position in the larger field coordinate system
          // Front right
          new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Back left
          new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
          // Back right
          new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0)
  );

  // By default we use a Pigeon for our gyroscope. But if you use another gyroscope, like a NavX, you can change this.
  // The important thing about how you configure your gyroscope is that rotating the robot counter-clockwise should
  // cause the angle reading to increase until it wraps back over to zero.

  private final PigeonIMU m_pigeon = new PigeonIMU(DRIVETRAIN_PIGEON_ID);
  // These are our modules. We initialize them in the constructor.
  public final SwerveModule m_frontLeftModule;
  public final SwerveModule m_frontRightModule;
  public final SwerveModule m_backLeftModule;
  public final SwerveModule m_backRightModule;

  private double tareLBEncoder = 0.0;
  private double tareLFEncoder = 0.0;
  private double tareRFEncoder = 0.0;
  private double tareRBEncoder = 0.0;

  public static SwerveDrivePoseEstimator m_odometry; 
  public static Pose2d m_pose = new Pose2d();
  public static ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain"); 

  //ChassisSpeeds takes in y velocity, x velocity, speed of rotation
  private ChassisSpeeds m_chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

  public DrivetrainSubsystem() {

    // There are 4 methods you can call to create your swerve modules.
    // The method you use depends on what motors you are using.
    //
    // Mk3SwerveModuleHelper.createFalcon500(...)
    //   Your module has two Falcon 500s on it. One for steering and one for driving.
    //
    // Mk3SwerveModuleHelper.createNeo(...)
    //   Your module has two NEOs on it. One for steering and one for driving.
    //
    // Mk3SwerveModuleHelper.createFalcon500Neo(...)
    //   Your module has a Falcon 500 and a NEO on it. The Falcon 500 is for driving and the NEO is for steering.
    //
    // Mk3SwerveModuleHelper.createNeoFalcon500(...)
    //   Your module has a NEO and a Falcon 500 on it. The NEO is for driving and the Falcon 500 is for steering.
    //
    // Similar helpers also exist for Mk4 modules using the Mk4SwerveModuleHelper class.

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
    
    m_odometry = new SwerveDrivePoseEstimator(m_kinematics, getGyroscopeRotation(), new SwerveModulePosition[] {m_frontLeftModule.getSwerveModulePosition(), m_frontRightModule.getSwerveModulePosition(), m_backRightModule.getSwerveModulePosition(), m_backLeftModule.getSwerveModulePosition()}, new Pose2d());
  }

   /**
   * Sets the gyroscope angle to zero. This can be used to set the direction the robot is currently facing to the
   * 'forwards' direction.
   * sets angle to go straight ahead
   */
  public void setHeading() {
        m_pigeon.setFusedHeading(0.0);
  }  
  
  //returns the direction of the robot, originally in radians, but fromDegrees switches into degrees
  public Rotation2d getGyroscopeRotation() {
        return Rotation2d.fromDegrees(m_pigeon.getYaw());
  }

  public double getEncoderPosition(SwerveModule module) {
        if (module == m_backLeftModule){
                return module.getPosition() - tareLBEncoder;
        } else if (module == m_backRightModule){
                return module.getPosition() - tareRBEncoder;
        } else if (module == m_frontLeftModule){
                return module.getPosition() - tareLFEncoder;
        } else {
                return module.getPosition() - tareRFEncoder;
        }
  }

  public double getDistance(){
        double[] positions = {getEncoderPosition(m_backLeftModule), getEncoderPosition(m_backRightModule), getEncoderPosition(m_frontLeftModule), getEncoderPosition(m_frontRightModule)};
        return getMedian(positions);
  }

  public void resetOdometry(Pose2d start){
        zeroGyroscope();
        SwerveModulePosition [] positionArray =  new SwerveModulePosition[] {
                m_frontLeftModule.getSwerveModulePosition(),
                m_frontRightModule.getSwerveModulePosition(),
                m_backRightModule.getSwerveModulePosition(),
                m_backLeftModule.getSwerveModulePosition() };
        m_pose = start;
        System.out.println("position array: " + positionArray.toString());
        System.out.println("m_pose: " + m_pose);
        m_odometry.resetPosition(getGyroscopeRotation(), positionArray, m_pose);
        
        System.out.println("#resetodometry! new pose: " + m_pose.getX()/TICKS_PER_INCH + " y: " + m_pose.getY()/TICKS_PER_INCH);
        System.out.println("inputs for the reset: " + getGyroscopeRotation() + " " + m_frontLeftModule.getSwerveModulePosition().distanceMeters + " " + m_frontRightModule.getSwerveModulePosition().distanceMeters + " " + m_backLeftModule.getSwerveModulePosition().distanceMeters + " " + m_backRightModule.getSwerveModulePosition().distanceMeters);
}

  public void zeroDriveEncoder(){
        tareLBEncoder = m_backLeftModule.getPosition();
        tareLFEncoder = m_frontLeftModule.getPosition();
        tareRFEncoder = m_frontRightModule.getPosition();
        tareRBEncoder = m_backRightModule.getPosition();
        System.out.println("tared...  " + getDistance());
  }

  public void zeroGyroscope() {
        m_pigeon.setYaw(0.0);
  }  

  public void setSpeed(ChassisSpeeds chassisSpeeds) {
        m_chassisSpeeds = chassisSpeeds;
  }

//   public Pose2d getCurrentPose(){
//         return m_pose;
//   }

//   public SwerveDriveOdometry getOdometry(){
//         return m_odometry;
//   }
  
  public void driveTeleop(){
        DoubleSupplier m_translationXSupplier = () -> -modifyAxis(OI.m_controller.getLeftY()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND;
        DoubleSupplier m_translationYSupplier = () -> -modifyAxis(OI.m_controller.getLeftX()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND;
        DoubleSupplier m_rotationSupplier = () -> -modifyAxis(OI.m_controller.getRightX()) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND;
        
        //setting speed
        setSpeed(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        m_translationXSupplier.getAsDouble(),
                        m_translationYSupplier.getAsDouble(),
                        m_rotationSupplier.getAsDouble(),
                        getGyroscopeRotation()
                )
        );
        //using speed to go
        drive();
  }

  public void drive() { //runs periodically
        //System.out.println("pose before update: " + m_pose.getX()/TICKS_PER_INCH + " and y: " + m_pose.getY()/TICKS_PER_INCH);

        //System.out.println("inputs for the update: " + getGyroscopeRotation() + m_frontLeftModule.getSwerveModulePosition().distanceMeters + m_frontRightModule.getSwerveModulePosition().distanceMeters + m_backLeftModule.getSwerveModulePosition().distanceMeters + m_backRightModule.getSwerveModulePosition().distanceMeters);
        m_pose = m_odometry.update(getGyroscopeRotation(), new SwerveModulePosition[] {m_frontLeftModule.getSwerveModulePosition(), m_frontRightModule.getSwerveModulePosition(), m_backLeftModule.getSwerveModulePosition(), m_backRightModule.getSwerveModulePosition()});
    
        System.out.println("new pose after update: " + m_pose.getX()/TICKS_PER_INCH + " and y: " + m_pose.getY()/TICKS_PER_INCH);
    
        //array of states filled with the speed and angle for each module (made from linear and angular motion for the whole robot) 
        SwerveModuleState[] states = m_kinematics.toSwerveModuleStates(m_chassisSpeeds);
        //desaturatewheelspeeds checks and fixes if any module's wheel speed is above the max
        SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_VELOCITY_METERS_PER_SECOND);
        /*
        double frontLeftSpeed= appliedDrivePID(states[0], m_frontLeftModule);
        double frontRightSpeed= appliedDrivePID(states[1], m_frontRightModule);
        double backLeftSpeed= appliedDrivePID(states[2], m_backLeftModule);
        double backRightSpeed= appliedDrivePID(states[3], m_backRightModule);

        double frontLeftAngle= appliedAnglePID(states[0], m_frontLeftModule);
        double frontRightAngle= appliedAnglePID(states[1], m_frontRightModule);
        double backLeftAngle= appliedAnglePID(states[2], m_backLeftModule);
        double backRightAngle= appliedAnglePID(states[3], m_backRightModule);
        
        //parameters are double driveVoltage, double steerAngle
        m_frontLeftModule.set(frontLeftSpeed / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, frontLeftAngle);
        m_frontRightModule.set(frontRightSpeed / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, frontRightAngle);
        m_backLeftModule.set(backLeftSpeed / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, backLeftAngle);
        m_backRightModule.set(backRightSpeed / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, backRightAngle);
        */
        m_frontLeftModule.set(states[0].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[0].angle.getRadians());
        m_frontRightModule.set(states[1].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[1].angle.getRadians());
        m_backLeftModule.set(states[2].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[2].angle.getRadians());
        m_backRightModule.set(states[3].speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, states[3].angle.getRadians());

}

  private double appliedDrivePID(SwerveModuleState state, SwerveModule module){
        double goalDriveVelocity= state.speedMetersPerSecond;
        double currentDriveVelocity= module.getDriveVelocity();
        PIDController pid = new PIDController(0.001, 0.0, 0.0);
        pid.setTolerance(0.1);
        return pid.calculate(currentDriveVelocity, goalDriveVelocity);
  }

  private double appliedAnglePID(SwerveModuleState state, SwerveModule module){
        double goalAngle= state.angle.getRadians();
        double currentAngle= module.getSteerAngle();
        PIDController pid = new PIDController(0.001, 0.0, 0.0);
        pid.setTolerance(Math.toRadians(1));
        return pid.calculate(goalAngle, currentAngle);
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

    public double getMedian(double[] num){
            Arrays.sort(num);
            if (num.length%2==0){
                    return (num[num.length/2] + num[(num.length-1)/2])/2;
            } else {
                    return num[num.length/2];
            }
    }

    //AUTO AND FAILSAFE
    public void stopDrive() {
        setSpeed(new ChassisSpeeds(0.0, 0.0, 0.0));
        drive();
   }
}
