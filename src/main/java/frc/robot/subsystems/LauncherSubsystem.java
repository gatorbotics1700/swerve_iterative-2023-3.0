package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;

import frc.robot.Constants;
import frc.robot.OI;

public class LauncherSubsystem {
    public TalonFX launchWheelMotor; 
    public TalonFX feedWheelMotor; 
    public boolean isRunning;
    public double speed;
    public LauncherSubsystem(){
        launchWheelMotor = new TalonFX(Constants.LAUNCH_MOTOR_ID);
        feedWheelMotor = new TalonFX(Constants.FEED_MOTOR_ID);
        init();
    }

    public void init() {
        isRunning = false;
        speed = 0;
    }

    public void periodic() {
        if(isRunning == true){
            setLaunchWheel(this.speed);
            setFeedWheel(this.speed);
        }
        else {
            stop();
        }
    }
    public void setLaunchWheel(double speed) {
        launchWheelMotor.set(ControlMode.PercentOutput, speed);
    }
    public void setFeedWheel(double speed) {
        feedWheelMotor.set(ControlMode.PercentOutput, speed);
    }
    public void stop() {
       launchWheelMotor.set(ControlMode.PercentOutput, 0);
       feedWheelMotor.set(ControlMode.PercentOutput, 0);
    }

    
}