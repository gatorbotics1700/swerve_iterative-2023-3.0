package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import frc.robot.Constants;
import frc.robot.Gains;//i think this means we do not need to have our own gains file

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.math.controller.PIDController;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController.*;


public class ArmRotationSubsystem {
    //these values are to be determined (untested)
    double _kP = 1.0;
    double _kI = 0.0;
    double _kD = 0.0;
    double _kF = 0.0;
    int _kIzone = 0;
    double _kPeakOutput = 0.0;
    
    public static ArmRotationStates rState = ArmRotationStates.ZERO;//should this be retracted or mid? what is the equivalent to off?

    TalonFX armRotationMotor = new TalonFX(Constants.ARM_ROTATION_MOTOR_ID);//maybe this motor should be renamed to make it more descriptive

    Gains armRotationGains = new Gains(_kP, _kI, _kD, _kF, _kIzone, _kPeakOutput);


    public static enum ArmRotationStates{
        ZERO, //vertical 
        NINETY,
        MID_ROTATION_ANGLE, //off the shelf
        LOW_ROTATION_ANGLE,
        HIGH_ROTATION_ANGLE;
    }
    
    public void setRState(ArmRotationStates newState){
        rState = newState;
    }

    public ArmRotationSubsystem(){}

    public void init(){
        armRotationMotor.setInverted(false);
        armRotationMotor.setNeutralMode(NeutralMode.Brake);
        //need to confirm with kim that we can zero motor when arm is vertical, then have negative 90 and 90 respectively when it rotates

        //configuring deadband
        armRotationMotor.configAllowableClosedloopError(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
		/* Config Position Closed Loop gains in slot0, tsypically kF stays zero. */
		armRotationMotor.config_kF(Constants.kPIDLoopIdx, armRotationGains.kF, Constants.kTimeoutMs);
		armRotationMotor.config_kP(Constants.kPIDLoopIdx, armRotationGains.kP, Constants.kTimeoutMs);
		armRotationMotor.config_kI(Constants.kPIDLoopIdx, armRotationGains.kI, Constants.kTimeoutMs);
		armRotationMotor.config_kD(Constants.kPIDLoopIdx, armRotationGains.kD, Constants.kTimeoutMs);
    }


    public void periodic(){
        if (rState == ArmRotationStates.ZERO){
            armRotationMotor.set(ControlMode.Position, 0);
            armRotationMotor.set(ControlMode.PercentOutput, 0);
        } else if (rState == ArmRotationStates.NINETY){
            armRotationMotor.set(ControlMode.Position, 90); //the following values are arbitrary
        } else if (rState == ArmRotationStates.LOW_ROTATION_ANGLE){
            armRotationMotor.set(ControlMode.Position, 8); 
        }else if (rState == ArmRotationStates.HIGH_ROTATION_ANGLE){
            armRotationMotor.set(ControlMode.Position, 5); 
        }else{
            armRotationMotor.set(ControlMode.Position, 15); 
        }
    }

    
    public void rotateArm(double armRotationSetpoint){//setpoint in radians
        armRotationSetpoint = armRotationSetpoint * Constants.TICKS_PER_INCH;//will probably have to change because wheel diameter in this constant is not relevant
        armRotationMotor.set(ControlMode.Position, armRotationSetpoint);//change to whatever motor we use for arm
    }

    public double ticksToDegrees(double ticks){
        return(ticks * 360 * Constants.ARM_ROTATION_GEAR_RATIO / Constants.TICKS_PER_REV);
    }

    public double getRotationAngle(){
        return ticksToDegrees(armRotationMotor.getSelectedSensorPosition());//returns current rotation angle
    }    

    //potentiometer? flight sensor?
    //AD worried about slipping with encoder and gravity
    //absolute tick count
    //KBR says no worries

}