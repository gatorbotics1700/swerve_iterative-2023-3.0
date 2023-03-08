package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;

import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;

import frc.robot.Gains;

public class ArmTelescopingSubsystem {

    public static TelescopingStates tState = TelescopingStates.RETRACTED; //should this be retracted or mid? what is the equivalent to off?

    public TalonFX telescopingMotor = new TalonFX(Constants.TELESCOPING_MOTOR_ID);
    private double startTime;
    private double desiredInches;
    private double desiredTicks;
    public double tareEncoder;

    double _kP = 0.35;
    double _kI = 0.05;
    double _kD = 0;
    public int _kIzone = 0;
    public double _kPeakOutput = 1.0;

    private int deadband = 15000;
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
        telescopingMotor.setInverted(false); //forward = clockwise, changed on 2/9
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
            desiredInches = 0; 
            desiredTicks = 0;
            telescopeDeadband();
        } else if (tState == TelescopingStates.LOW_ARM_LENGTH){
            desiredInches = 3; //official 2/13
            determineRightTicks();
            System.out.println("desired ticks: " + desiredTicks);
            telescopingMotor.set(ControlMode.Position, desiredTicks);
            System.out.println("error: " + (desiredTicks - telescopingMotor.getSelectedSensorPosition()));
            telescopeDeadband();
        } else if (tState == TelescopingStates.SHELF_ARM_LENGTH){
            desiredInches = 6; //official 2/13
            determineRightTicks(); 
            telescopingMotor.set(ControlMode.Position, desiredTicks-tareEncoder); // goes with 90 degrees rotation 
            System.out.println("error: " + (desiredTicks - telescopingMotor.getSelectedSensorPosition()));
            telescopeDeadband();
        }else if (tState == TelescopingStates.MID_ARM_LENGTH){
            desiredInches = 23; //official 2/13
            determineRightTicks(); 
            telescopingMotor.set(ControlMode.Position, desiredTicks-tareEncoder); // goes with 90 degrees rotation 
            System.out.println("error: " + (desiredTicks - telescopingMotor.getSelectedSensorPosition()));
            telescopeDeadband();
        }else if(tState == TelescopingStates.HIGH_ARM_LENGTH){ //high arm length
            desiredInches = 32; //official 2/13
            determineRightTicks();
            telescopingMotor.set(ControlMode.Position, desiredTicks-tareEncoder); 
            telescopeDeadband();
        }
        else { //retracted again for safety
            telescopingMotor.set(ControlMode.Position, 0);
            desiredInches = 0; 
            desiredTicks = 0;
            telescopeDeadband();
        }
    }

    public void determineRightTicks(){
        if (telescopingMotor.getSelectedSensorPosition() < 2 * Constants.UNDER_TWO_TICKS_PER_INCH){
            desiredTicks = 2 * Constants.UNDER_TWO_TICKS_PER_INCH; 
        } else{
            desiredTicks = (desiredInches-2) * Constants.OVER_TWO_TICKS_PER_INCH;
        }
        System.out.println("desired ticks from inside method: " + desiredTicks);
    }
    public void telescopeDeadband(){
        if (Math.abs(desiredTicks - telescopingMotor.getSelectedSensorPosition()) < deadband){
            telescopingMotor.set(ControlMode.PercentOutput, 0);
            System.out.println("STOPPED");
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
            while(System.currentTimeMillis() - startTime <= milliTime){
                telescopingMotor.set(ControlMode.PercentOutput,0.2);
            }
            telescopingMotor.set(ControlMode.PercentOutput,0);
        }else{
            while(System.currentTimeMillis() - startTime <= milliTime){
                telescopingMotor.set(ControlMode.PercentOutput,-0.2);
            }
            telescopingMotor.set(ControlMode.PercentOutput,0);
        }
    }

    public void setTState(TelescopingStates newState){
        tState = newState;
    }

    public void tareEncoder() {
        tareEncoder = telescopingMotor.getSelectedSensorPosition();

    }
    public double getTicks() {
        return telescopingMotor.getSelectedSensorPosition() - tareEncoder;
    }

    public boolean isHigh(){
        if(Math.abs(desiredInches-telescopingMotor.getSelectedSensorPosition()-32)<3*Constants.UNDER_TWO_TICKS_PER_INCH){
            return true;
        }
        return false; 
    }

}
