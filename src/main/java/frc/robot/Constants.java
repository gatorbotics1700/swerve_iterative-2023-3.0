// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final double BUMPER_WIDTH = 0.0762;
    public static final double DRIVETRAIN_WIDTH = 0.3175;
    /**
     * The left-to-right distance between the drivetrain wheels
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.4690872; //units = meters //previously 18.468
    /**
     * The front-to-back distance between the drivetrain wheels.
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_WHEELBASE_METERS = 0.4690872; //units = meter //previously 18.468 

    public static final int DRIVETRAIN_PIGEON_ID = 6; 

    //green
    
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(/*211.37 + 45*/212.08007);//306.123046875-90); //194.41
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(201.09375);//287.22656250000006-45); //305.15625
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(121.8164);//123.48632812500001);//23.73+90
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(20.654);//14.238281250000002+90);//122.87
    
    //public static final double offset = Robot.test.getDouble(DRIVETRAIN_PIGEON_ID); 
    
    //swervo
    /*public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(164.00390625);   //281.07421875-90 //279.316-270
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(341.72149658203125); //343.828125+90 //75.938
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(278.5308837890625);  //231.6796875+90 //320.098
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(52.525390625);//343.828125-90 //250.183*/

    //even can ids are drive, odd can ids are steer
    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 20;
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 21; 
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 2;
    
    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 22; 
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 23; 
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 3; 

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 26; 
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 27; 
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 5; 

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 24; 
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 25; 
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 4; 

    public static final double DRIVE_MOTOR_MIN_VOLTAGE = 0.19;
    public static final double DRIVE_MOTOR_MAX_VOLTAGE = 0.8;
    public static final double STEER_MOTOR_MIN_VOLTAGE = 0.525; 
    public static final double STEER_MOTOR_MAX_VOLTAGE = 0.5;

    public static final double SWERVE_GEAR_RATIO = 6.75;
    public static final double TICKS_PER_REV = 2048;
    public static final double METERS_PER_INCH = 0.0254;
    public static final double SWERVE_WHEEL_DIAMETER = 4.0;
    public static final double SWERVE_TICKS_PER_INCH = TICKS_PER_REV*SWERVE_GEAR_RATIO/SWERVE_WHEEL_DIAMETER/Math.PI; //talonfx drive encoder
    public static final double TICKS_PER_METER = SWERVE_TICKS_PER_INCH/METERS_PER_INCH;
    public static final double TICKS_PER_INCH = TICKS_PER_METER*METERS_PER_INCH;

    //TELESCOPE
    public static final int TELESCOPING_MOTOR_ID = 31;
    //public static final double TELESCOPING_ARM_GEAR_RATIO = 36.0; //In case we need this later

    //ELEVATOR
    public static final int ELEVATOR_CAN_ID = 30;

    //INTAKE
    public static final int INTAKE_SENSOR = 0;
    public static final int BEAM_BREAK_RECEIVER = 9;
    public static final int BEAM_BREAK_SENDER = 10;

    //VISION
    public static final double TAPE_HEIGHT_ONE = 0.0; //TODO: needs to be changed
    public static final double LIMELIGHT_HEIGHT = 52 * METERS_PER_INCH;
    public static final double LIMELIGHT_ANGLE = -24; //in degrees
   
    public static final int kPIDLoopIdx = 0;
    public static final int kTimeoutMs = 500;
}