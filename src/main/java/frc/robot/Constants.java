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

    //even can ids are drive, odd can ids are steer
    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 26;
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 27; 
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 5;
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(98.3); // FIXME Measure and set front left steer offset

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 20; 
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 21; 
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 2; 
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(251.4); // FIXME Measure and set front right steer offset

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 24; 
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 25; 
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 4; 
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(138.34); // FIXME Measure and set back left steer offset

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 22; 
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 23; 
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 3; 
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(79.45); // FIXME Measure and set back right steer offset

    public static final int TELESCOPING_MOTOR_ID = 7;
    public static final int ARM_ROTATION_MOTOR_ID = 8;
    public static final int ELEVATOR_CAN_ID = 9;
    public static final int LEFT_LINK_CAN_ID = 10;
    public static final int RIGHT_LINK_CAN_ID = 11;

    public static final double DRIVE_MOTOR_MIN_VOLTAGE = 0.19;
    public static final double DRIVE_MOTOR_MAX_VOLTAGE = 0.8;
    public static final double STEER_MOTOR_MIN_VOLTAGE = 0.02; 
    public static final double STEER_MOTOR_MAX_VOLTAGE = 0.5;
    public static final double GEAR_RATIO = 6.75;
    public static final double ELEVATOR_SPROCKET_DIAMETER= 1.05; //inches
    public static final double TICKS_PER_REV = 2048;
    public static final double ELEVATOR_GEAR_RATIO = 25.0; //says arya on jan 30th // 25 motor spins for 1 shaft spin

    public static final double TICKS_PER_INCH = (TICKS_PER_REV*ELEVATOR_GEAR_RATIO)/(ELEVATOR_SPROCKET_DIAMETER*Math.PI); //talonfx drive encoder

    public static final int kTimeoutMs = 1000;
    public static final int kPIDLoopIdx = 0; //not sure what this value does or if this value should be diff for rotation, telescoping, elevator
    
	/* Choose so that Talon does not report sensor out of phase */
    public static boolean kSensorPhase = true; //Sensor phase describes the relationship between the motor output direction (positive vs negative) and sensor velocity (positive vs negative). For soft-limits and closed-loop features to function correctly, the sensor measurement and motor output must be “in-phase”.

    //public static double deadband = 0.5; //arbitrary
}
