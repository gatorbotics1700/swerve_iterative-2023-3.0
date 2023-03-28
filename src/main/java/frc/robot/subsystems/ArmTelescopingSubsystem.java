package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;

import frc.robot.Gains;
import frc.robot.OI;

public class ArmTelescopingSubsystem {

    private TelescopingStates tState;

    private static final int MIDARMTICKS = 231686 - 10000 + 34900; //TODO: tune these lengths vv 
    private static final int SHELFARMTICKS = 312500; //+ 34900;
    private static final int LOWARMTICKS = 34900;
    private static final int SUBTICKS = 75000 + 34900;
    private static final int RETRACTEDTICKS = 0;
    private static final int DEADBAND = 15000;
    private static final int MAX_TICKS = 231686 + 34900; 
    
    public TalonFX telescopingMotor; //TODO: private
    private static final double _kP = 0.35;
    private static final double _kI = 0;
    private static final double _kD = 0;
    private static final int _kIzone = 0;
    private static final double _kPeakOutput = 1.0;

    private Gains telescopeGains = new Gains(_kP, _kI, _kD, _kIzone, _kPeakOutput);

    public static enum TelescopingStates{
        RETRACTED, //zero 
        LOW_ARM_LENGTH,
        SHELF_ARM_LENGTH,
        MID_ARM_LENGTH,
        SINGLE_SUBSTATION,
        MANUAL;
    }

    public ArmTelescopingSubsystem(){
        telescopingMotor = new TalonFX(Constants.TELESCOPING_MOTOR_ID);
        init();
    }

    public void init(){
        System.out.println("telescoping init!! :)");
        tState = TelescopingStates.RETRACTED;
        telescopingMotor.setInverted(true); //forward = clockwise, changed on 2/9
        telescopingMotor.setNeutralMode(NeutralMode.Brake);

        // telescopingMotor.selectProfileSlot(0, 0); //TODO: find out what this does
        telescopingMotor.configAllowableClosedloopError(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs); //TODO: confirm that this does not cause delay WHEN motor is plugged in
        telescopingMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
        telescopingMotor.config_kP(Constants.kPIDLoopIdx, telescopeGains.kP, Constants.kTimeoutMs);
        //telescopingMotor.setSelectedSensorPosition(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs); //TODO: why was this commented out?

    }

    public void periodic(){
        if (tState == TelescopingStates.RETRACTED){
            telescopingMotor.set(ControlMode.Position, RETRACTEDTICKS);
            telescopeDeadband(RETRACTEDTICKS);
        } else if (tState == TelescopingStates.LOW_ARM_LENGTH){
            telescopingMotor.set(ControlMode.Position, LOWARMTICKS);
            telescopeDeadband(LOWARMTICKS);
        } else if (tState == TelescopingStates.SHELF_ARM_LENGTH){
            telescopingMotor.set(ControlMode.Position, SHELFARMTICKS);  
            telescopeDeadband(SHELFARMTICKS);
        } else if (tState == TelescopingStates.MID_ARM_LENGTH){
            telescopingMotor.set(ControlMode.Position, MIDARMTICKS); 
            telescopeDeadband(MIDARMTICKS);
        } else if (tState == TelescopingStates.SINGLE_SUBSTATION){
            telescopingMotor.set(ControlMode.Position, SUBTICKS);
            telescopeDeadband(SUBTICKS);
        } else if(tState == TelescopingStates.MANUAL){
            manual();
        } else { //retracted again for safety
            telescopingMotor.set(ControlMode.PercentOutput, 0);
        }
    }

    private void telescopeDeadband(double desiredTicks){
        if (Math.abs(desiredTicks - telescopingMotor.getSelectedSensorPosition()) < DEADBAND){
            telescopingMotor.set(ControlMode.PercentOutput, 0);
        }
    }
    
    public double getArmPosition(){
        return telescopingMotor.getSelectedSensorPosition();
    }

    public void setTState(TelescopingStates newState){
        tState = newState;
    }

    public void manual(){
        //if(getArmPosition() <= MAX_TICKS && getArmPosition() >= 0) {
            if(OI.getLeftAxis() > 0.2){
                telescopingMotor.set(ControlMode.PercentOutput,-0.2);
            }else if(OI.getLeftAxis() < -0.2){
                telescopingMotor.set(ControlMode.PercentOutput,0.2);
            } else {
                telescopingMotor.set(ControlMode.PercentOutput, 0);
            }
        //} else {
            //telescopingMotor.set(ControlMode.PercentOutput, 0);
        //}
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

    public boolean isAtRetracted(){
        return Math.abs(telescopingMotor.getSelectedSensorPosition()) < DEADBAND;
    }

    public void setTelescopePosition(){
        telescopingMotor.setSelectedSensorPosition(0.0);
    }

}