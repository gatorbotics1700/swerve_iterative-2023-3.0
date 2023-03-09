// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
//import frc.robot.Robot;

import edu.wpi.first.networktables.GenericEntry;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
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
    
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(194.41); //194.41
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(305.15625);
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(23.73+90);//fix this one to like -70 or something
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(122.87);
    
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
    public static final double TELESCOPING_ARM_GEAR_RATIO = 36.0; // as of 2/6
    public static final double FIRST_WHEEL_DIAMETER= 1.13;//0.9; //0.75 inches - rough estimate by sara 2/2
    public static final double SECOND_WHEEL_DIAMETER= 0.87;// 0.8; //0.75 inches - rough estimate by sara 2/2
    public static final double SWERVE_WHEEL_DIAMETER = 4.0;

    public static final double TICKS_PER_REV = 2048;
    public static final double UNDER_TWO_TICKS_PER_INCH = TICKS_PER_REV*TELESCOPING_ARM_GEAR_RATIO/FIRST_WHEEL_DIAMETER/Math.PI; //talonfx drive encoder
    public static final double OVER_TWO_TICKS_PER_INCH = TICKS_PER_REV*TELESCOPING_ARM_GEAR_RATIO/SECOND_WHEEL_DIAMETER/Math.PI;
    public static final double SWERVE_TICKS_PER_INCH = TICKS_PER_REV*SWERVE_GEAR_RATIO/SWERVE_WHEEL_DIAMETER/Math.PI;
    
    public static final int TELESCOPING_MOTOR_ID = 29; //31;
    public static final int BEAM_BREAK_RECEIVER = 0; //port number of beam break receiver
    public static final int BEAM_BREAK_SENDER = 1;
    
    public static final int ELEVATOR_CAN_ID = 9;
    public static final int kTimeoutMs = 1000;
    public static final int kPIDLoopIdx = 0;

    public static final double GEAR_RATIO = 6.75;
    public static final double WHEEL_DIAMETER= 0.1016; //meters, previously 4 inches

    public static final double METERS_PER_INCH = 0.0254; 
    public static final double TICKS_PER_METER = TICKS_PER_REV*SWERVE_GEAR_RATIO/SWERVE_WHEEL_DIAMETER/Math.PI/METERS_PER_INCH;
    public static final double TICKS_PER_INCH = TICKS_PER_METER*METERS_PER_INCH;
}
