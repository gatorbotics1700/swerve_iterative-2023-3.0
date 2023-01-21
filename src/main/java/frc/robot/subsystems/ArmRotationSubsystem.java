package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.math.controller.PIDController;
import com.ctre.phoenix.motorcontrol.NeutralMode;


public class ArmRotationSubsystem {
    //variables go here
    //double armLengthKP = 0.04; //actual vals to be determined
    //double armLengthKI = 0.000002;
    //double armLengthKD = 0.005;
    //PIDController armLengthController = new PIDController(armLengthKP, armLengthKI, armLengthKD);
    public static ArmRotationStates rState = ArmRotationStates.ZERO;//should this be retracted or mid? what is the equivalent to off?

    TalonFX armRotationMotor = new TalonFX(Constants.ARM_ROTATION_MOTOR_ID);//maybe this motor should be renamed to make it more descriptive

    //armMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.kPIDLoopIdx, Constants.kTimeoutMs);//determine what these values would be for us


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

}