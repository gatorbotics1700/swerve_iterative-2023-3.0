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
    /**
     * The left-to-right distance between the drivetrain wheels
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = 18.468;
    /**
     * The front-to-back distance between the drivetrain wheels.
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_WHEELBASE_METERS = 18.468; 

    public static final int DRIVETRAIN_PIGEON_ID = 6; 

    //green
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(193.36);
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(305.24);
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(276.86);
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(122.87);

    //swervo
    /*public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(185.713); 
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(341.982);
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(229.064); 
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(341.904);*/

    //even can ids are drive, odd can ids are steer
    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 26;
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 27; 
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 5;
    
    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 20; 
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 21; 
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 2; 

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 24; 
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 25; 
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 4; 

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 22; 
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 23; 
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 3; 

    public static final double DRIVE_MOTOR_MIN_VOLTAGE = 0.19;
    public static final double DRIVE_MOTOR_MAX_VOLTAGE = 0.8;
    public static final double STEER_MOTOR_MIN_VOLTAGE = 0.02; 
    public static final double STEER_MOTOR_MAX_VOLTAGE = 0.5;
    public static final double TELESCOPING_ARM_GEAR_RATIO = 36.0; // as of 2/6
    public static final double WHEEL_DIAMETER= 0.75; //inches - rough estimate by sara 2/2
    public static final double TICKS_PER_REV = 2048;
    public static final double TICKS_PER_INCH = TICKS_PER_REV*TELESCOPING_ARM_GEAR_RATIO/WHEEL_DIAMETER/Math.PI; //talonfx drive encoder

    public static final int TELESCOPING_MOTOR_ID = 31;
}
