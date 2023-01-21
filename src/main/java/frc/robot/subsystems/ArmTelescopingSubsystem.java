package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import frc.robot.Constants;
import frc.robot.Gains;

import com.ctre.phoenix.motorcontrol.ControlMode;

public class ArmTelescopingSubsystem {

    //these values are to be determined (untested)
    double _kP = 1.0;
    double _kI = 0.0;
    double _kD = 0.0;
    double _kF = 0.0;
    int _kIzone = 0;
    double _kPeakOutput = 0.0;

    public static TelescopingStates tState = TelescopingStates.RETRACTED;//should this be retracted or mid? what is the equivalent to off?

    TalonFX telescopingMotor = new TalonFX(Constants.TELESCOPING_MOTOR_ID);//maybe this motor should be renamed to make it more descriptive

    Gains armTelescopingGains = new Gains(_kP, _kI, _kD, _kF, _kIzone, _kPeakOutput);

    //armMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.kPIDLoopIdx, Constants.kTimeoutMs);//determine what these values would be for us

    /*public static enum ArmStates{
        OFF, //fully retracted
        FULLY_EXTENDED,
        LOW_ARM_LENGTH, // arm length when scoring on low node
        MID_ARM_LENGTH, // also shelf height
        HIGH_ARM_LENGTH;
    }*/

    double extensionVal = 56.26;//this value should be the full extension length of the arm minus the length of it at zero

    public static enum TelescopingStates{
        RETRACTED,
        FULLY_EXTENDED,
        LOW_ARM_LENGTH,
        MID_ARM_LENGTH,
        HIGH_ARM_LENGTH;
    }
    
    public void setTState(TelescopingStates newState){
        tState = newState;
    }

    public ArmTelescopingSubsystem(){}

    public void init(){
        telescopingMotor.setInverted(false);

        //configuring deadband
        telescopingMotor.configAllowableClosedloopError(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
		/* Config Position Closed Loop gains in slot0, tsypically kF stays zero. */
		telescopingMotor.config_kF(Constants.kPIDLoopIdx, armTelescopingGains.kF, Constants.kTimeoutMs);
		telescopingMotor.config_kP(Constants.kPIDLoopIdx, armTelescopingGains.kP, Constants.kTimeoutMs);
		telescopingMotor.config_kI(Constants.kPIDLoopIdx, armTelescopingGains.kI, Constants.kTimeoutMs);
		telescopingMotor.config_kD(Constants.kPIDLoopIdx, armTelescopingGains.kD, Constants.kTimeoutMs);
    }

    public void periodic(){//sam requests that we can operate arm length by stick on xbox
        if (tState == TelescopingStates.RETRACTED){
            telescopingMotor.set(ControlMode.Position, 0);
            telescopingMotor.set(ControlMode.PercentOutput, 0);
        } else if (tState == TelescopingStates.FULLY_EXTENDED){
            telescopingMotor.set(ControlMode.Position, extensionVal); //confirmed
        } else if (tState == TelescopingStates.LOW_ARM_LENGTH){
            telescopingMotor.set(ControlMode.Position, 8); // replace position value w low length
        }else if (tState == TelescopingStates.MID_ARM_LENGTH){
            telescopingMotor.set(ControlMode.Position, 5); // goes with 90 degrees rotation // replace position value w mid length
        }else{
            telescopingMotor.set(ControlMode.Position, 15); // replace position value w high length
        }
    }

    
    public void moveArm(double armSetpoint){//setpoint in inches
        armSetpoint = armSetpoint * Constants.TICKS_PER_INCH;//will probably have to change because wheel diameter in this constant is not relevant
        telescopingMotor.set(ControlMode.Position, armSetpoint);//change to whatever motor we use for arm
    }

    public double getArmPosition(){
        return telescopingMotor.getSelectedSensorPosition();
    }    

    //public void armPID(double armLengthSetpoint){
       // armLengthController.setSetpoint(armLengthSetpoint);
        //double output = armLengthController.calculate(getArmLength(), armLengthSetpoint);
        //System.out.println("output: " + output);
        //armMotor.set(ControlMode.PercentOutput, output);//written as _talon.set(TalonFXControlMode.PercentOutput, leftYstick); in documentation. leftystick is joy.gety
    //}

     // if (Math.abs(leftYstick) < 0.10/*this number should be changed through testing*/) {
    //     /* Within 10% of zero */
    //     leftYstick = 0;//left joystick y axis in documentation. assign later
    // }

}
