package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.ControlMode;//written as import com.ctre.phoenix.motorcontrol.TalonFXControlMode; in some documentation. is ours okay??
import edu.wpi.first.math.controller.PIDController;

public class ArmSubsystem {

    //variables go here
    //double armLengthKP = 0.04; //actual vals to be determined
    //double armLengthKI = 0.000002;
    //double armLengthKD = 0.005;
    //PIDController armLengthController = new PIDController(armLengthKP, armLengthKI, armLengthKD);
    public static ArmStates armState = ArmStates.OFF;

    TalonFX armMotor = new TalonFX(Constants.ARM_MOTOR_ID);

    //armMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.kPIDLoopIdx, Constants.kTimeoutMs);//determine what these values would be for us

    public static enum ArmStates{
        OFF, //fully retracted
        FULLY_EXTENDED,
        LOW_ARM_LENGTH, // arm length when scoring on low node
        MID_ARM_LENGTH, // also shelf height
        HIGH_ARM_LENGTH;
    }
    
    public void setState(ArmStates newState){
        armState = newState;
    }

    public ArmSubsystem(){}

    public void init(){
        armMotor.setInverted(false);
    }

    public void periodic(){
        if (armState == ArmStates.OFF){
            armMotor.set(ControlMode.Position, 0);
            armMotor.set(ControlMode.PercentOutput, 0);
        } else if (armState == ArmStates.FULLY_EXTENDED){
            armMotor.set(ControlMode.Position, 56.25); //confirmed
        } else if (armState == ArmStates.LOW_ARM_LENGTH){
            armMotor.set(ControlMode.Position, 8); // replace position value w low length
        }else if (armState == ArmStates.MID_ARM_LENGTH){
            armMotor.set(ControlMode.Position, 5); // goes with 90 degrees rotation // replace position value w mid length
        }else{
            armMotor.set(ControlMode.Position, 15); // replace position value w high length
        }
    }

    
    public void moveArm(double armSetpoint){//setpoint in inches
        armSetpoint = armSetpoint * Constants.TICKS_PER_INCH;//will probably have to change because wheel diameter in this constant is not relevant
        armMotor.set(ControlMode.Position, armSetpoint);//change to whatever motor we use for arm
    }

    public double getArmLength(){
        return armMotor.getSelectedSensorPosition();
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
