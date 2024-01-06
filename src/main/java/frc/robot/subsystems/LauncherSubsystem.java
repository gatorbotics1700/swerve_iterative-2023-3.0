package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import frc.robot.OI;

public class LauncherSubsystem {
    public TalonFX launchWheelMotor; 
    public TalonFX feedWheelMotor; 
    public boolean isRunning;
    public LauncherSubsystem(){
        init();
    }

    public void init() {
        isRunning = false;
    }

    public void periodic() {
        if(isRunning == true){
            setLaunchWheel(0.5);
            setFeedWheel(0.5);
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