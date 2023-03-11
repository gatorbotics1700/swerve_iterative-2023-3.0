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

    public TalonFX telescopingMotor = new TalonFX(Constants.TELESCOPING_MOTOR_ID);
    private double startTime;
    private double desiredInches;
    public double tareEncoder;
    // public Compressor compressor = new Compressor(PneumaticsModuleType.REVPH);

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
        System.out.println("telescoping arm state: " + tState);
        System.out.println("current telescoping arm motor position: " + telescopingMotor.getSelectedSensorPosition());
        if (tState == TelescopingStates.RETRACTED){
            telescopingMotor.set(ControlMode.Position, 0);
            desiredInches = 0; 
            telescopeDeadband(0);
        } else if (tState == TelescopingStates.LOW_ARM_LENGTH){
            desiredTicks = 0; //37500 ticks is 3 inches
            //double desiredTicks = determineRightTicks(desiredInches);
            System.out.println("set state to low arm length");
            System.out.println("desired ticks: " + desiredTicks);
            telescopingMotor.set(ControlMode.Position, desiredTicks);
            System.out.println("error: " + (desiredTicks - telescopingMotor.getSelectedSensorPosition()));
            telescopeDeadband(desiredTicks);
        } else if (tState == TelescopingStates.SHELF_ARM_LENGTH){
            desiredTicks = 0; 
            telescopingMotor.set(ControlMode.Position, desiredTicks-tareEncoder); // goes with 90 degrees rotation 
            System.out.println("error: " + (desiredTicks - telescopingMotor.getSelectedSensorPosition()));
            telescopeDeadband(desiredTicks);
        }else if (tState == TelescopingStates.MID_ARM_LENGTH){
            desiredTicks = 34375; //should be 2.75 inches
            telescopingMotor.set(ControlMode.Position, desiredTicks-tareEncoder); // goes with 90 degrees rotation 
            System.out.println("error: " + (desiredTicks - telescopingMotor.getSelectedSensorPosition()));
            telescopeDeadband(desiredTicks);
        }else if(tState == TelescopingStates.HIGH_ARM_LENGTH){ //high arm length
            desiredTicks = 246875; //should be 19.75 inches
            telescopingMotor.set(ControlMode.Position, desiredTicks-tareEncoder); 
            telescopeDeadband(desiredTicks);
        }
        else { //retracted again for safety
            telescopingMotor.set(ControlMode.Position, 0);
            desiredInches = 0; 
            telescopeDeadband(0);
        }
    }

    public void determineRightTicks(){
        // if (desiredInches <= 2){
        //     System.out.println("under two ticks per inch");
        //     return 2 * Constants.UNDER_TWO_TICKS_PER_INCH; 
        // } else{
        //     System.out.println("over two ticks per inch");
        //     return 2 * Constants.UNDER_TWO_TICKS_PER_INCH + (desiredInches-2) * Constants.OVER_TWO_TICKS_PER_INCH;
        // }
    }

    public void telescopeDeadband(double desiredTicks){
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

}
