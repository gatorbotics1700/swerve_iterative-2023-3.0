package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;

import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;

import frc.robot.Gains;

public class ArmTelescopingSubsystem {

    public static TelescopingStates tState = TelescopingStates.RETRACTED; //should this be retracted or mid? what is the equivalent to off?

    private static final int HIGHARMTICKS = 246875;
    private static final int MIDARMTICKS = 34375 + 25000;
    private static final int SHELFARMTICKS = 0;
    private static final int LOWARMTICKS = 0;
    private static final int RETRACTEDTICKS = 0;
    private static final int DEADBAND = 12000;
    
    public TalonFX telescopingMotor = new TalonFX(Constants.TELESCOPING_MOTOR_ID);
    private double startTime;
    public double tareEncoder;

    double _kP = 0.35;
    double _kI = 0;
    double _kD = 0;
    public int _kIzone = 0;
    public double _kPeakOutput = 1.0;

    private int deadband = 15000;
    private double desiredTicks;
    public Gains telescopeGains = new Gains(_kP, _kI, _kD, _kIzone, _kPeakOutput);

    public static enum TelescopingStates{
        RETRACTED, //zero 
        LOW_ARM_LENGTH,
        SHELF_ARM_LENGTH,
        MID_ARM_LENGTH,
        HIGH_ARM_LENGTH;
    }

    public void init(){
        System.out.println("telescoping init!! :)");
        telescopingMotor.setInverted(true); //forward = clockwise, changed on 2/9
        telescopingMotor.setNeutralMode(NeutralMode.Brake);

        //telescopingMotor.selectProfileSlot(0, 0);
        telescopingMotor.configAllowableClosedloopError(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
        telescopingMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.kPIDLoopIdx, Constants.kTimeoutMs);//determine what these values would be for us
        telescopingMotor.config_kP(Constants.kPIDLoopIdx, telescopeGains.kP, Constants.kTimeoutMs);
        telescopingMotor.config_kI(Constants.kPIDLoopIdx, telescopeGains.kI, Constants.kTimeoutMs);
        telescopingMotor.config_kD(Constants.kPIDLoopIdx, telescopeGains.kD, Constants.kTimeoutMs);

    }

    public void periodic(){//sam requests that we can operate arm length by stick on xbox
        //telescopingMotor.set(ControlMode.PercentOutput, 0.2);
        System.out.println("current telescoping arm motor position:" + telescopingMotor.getSelectedSensorPosition());
        if (tState == TelescopingStates.RETRACTED){
            telescopingMotor.set(ControlMode.Position, 0);
            desiredTicks = RETRACTEDTICKS;
            telescopeDeadband(RETRACTEDTICKS);
        } else if (tState == TelescopingStates.LOW_ARM_LENGTH){
            desiredTicks = LOWARMTICKS; //official 2/13
            telescopingMotor.set(ControlMode.Position, desiredTicks);
            //System.out.println("tryinggg to go low");
            //System.out.println("our ticks: " + telescopingMotor.getSelectedSensorPosition());
            //System.out.println("desired telescope ticks: " + desiredTicks);
            //System.out.println("error: " + (desiredTicks - telescopingMotor.getSelectedSensorPosition()));
            telescopeDeadband(LOWARMTICKS);
        } else if (tState == TelescopingStates.SHELF_ARM_LENGTH){
            desiredTicks = SHELFARMTICKS; //official 2/13
            telescopingMotor.set(ControlMode.Position, desiredTicks-tareEncoder); // goes with 90 degrees rotation 
            //System.out.println("error: " + (desiredTicks - telescopingMotor.getSelectedSensorPosition()));
            telescopeDeadband(SHELFARMTICKS);
        }else if (tState == TelescopingStates.MID_ARM_LENGTH){
            desiredTicks = MIDARMTICKS; //should be 2.75 inches
            telescopingMotor.set(ControlMode.Position, desiredTicks-tareEncoder); // goes with 90 degrees rotation 
            //System.out.println("tryinggg to go mid");
            //System.out.println("our ticks: " + telescopingMotor.getSelectedSensorPosition());
            //System.out.println("desired telescope ticks: " + desiredTicks);
            //System.out.println("error: " + (desiredTicks - telescopingMotor.getSelectedSensorPosition()));
            telescopeDeadband(MIDARMTICKS);
        }else if(tState == TelescopingStates.HIGH_ARM_LENGTH){ //high arm length
            desiredTicks = HIGHARMTICKS; //should be 19.75 inches
            telescopingMotor.set(ControlMode.Position, desiredTicks-tareEncoder); 
            telescopeDeadband(HIGHARMTICKS);
        } else { //retracted again for safety
            telescopingMotor.set(ControlMode.PercentOutput, 0);
        }
    }

    public void telescopeDeadband(double desiredTicks){
        System.out.println("difference: " + (desiredTicks - telescopingMotor.getSelectedSensorPosition()));
        if (Math.abs(desiredTicks - telescopingMotor.getSelectedSensorPosition()) < deadband){
            telescopingMotor.set(ControlMode.PercentOutput, 0);
            System.out.println("TELESCOPE STOPPED");
        }
    }
    
    public double getArmPosition(){
        return telescopingMotor.getSelectedSensorPosition();
    }
    
    public void timedMoveArm(double time, boolean forwards){ //time in seconds
        System.out.println("moving arm >:)");
        System.out.println("start time: " + startTime);
        double milliTime = time * 1000;
        System.out.println("milli time: " + milliTime);
        if(forwards == true){
            if(System.currentTimeMillis() - startTime <= milliTime){
                telescopingMotor.set(ControlMode.PercentOutput,0.2);
            }
            else{
                telescopingMotor.set(ControlMode.PercentOutput,0);
            }
        }else{
            if(System.currentTimeMillis() - startTime <= milliTime){
                telescopingMotor.set(ControlMode.PercentOutput,-0.2);
            }
            else{
                telescopingMotor.set(ControlMode.PercentOutput,0);
            }
        }
    }

    public void setTState(TelescopingStates newState){
        tState = newState;
    }
  
    public double getTicks() {
        return telescopingMotor.getSelectedSensorPosition() - tareEncoder;
    }

    public boolean isAtHigh(){
        return Math.abs(HIGHARMTICKS-telescopingMotor.getSelectedSensorPosition())<DEADBAND; //a little over an inch deadband
    }

    public boolean isAtMid(){
        return Math.abs(MIDARMTICKS-telescopingMotor.getSelectedSensorPosition())<DEADBAND;
    }

    public boolean isAtLow(){
        return Math.abs(LOWARMTICKS-telescopingMotor.getSelectedSensorPosition())<DEADBAND;
    }

    public boolean isAtShelf(){
        return Math.abs(SHELFARMTICKS - telescopingMotor.getSelectedSensorPosition()) < DEADBAND;
    }

    public boolean isAtZero(){
        return Math.abs(telescopingMotor.getSelectedSensorPosition()) < DEADBAND;
    }

}
