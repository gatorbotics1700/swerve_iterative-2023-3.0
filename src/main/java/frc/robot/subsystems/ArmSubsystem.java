package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.ControlMode;//written as import com.ctre.phoenix.motorcontrol.TalonFXControlMode; in some documentation. is ours okay??
import edu.wpi.first.math.controller.PIDController;

public class ArmSubsystem {

    //variables go here
    double armLengthKP = 0.04; //actual vals to be determined
    double armLengthKI = 0.000002;
    double armLengthKD = 0.005;
    PIDController armLengthController = new PIDController(armLengthKP, armLengthKI, armLengthKD);

    TalonFX armMotor = new TalonFX(Constants.ARM_MOTOR_ID);

    armMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.kPIDLoopIdx, Constants.kTimeoutMs);//determine what these values would be for us

    public ArmSubsystem(){

    }

    public double getArmLength(){
        return armMotor.getSelectedSensorPosition();
    }

    if (Math.abs(leftYstick) < 0.10/*this number should be changed through testing*/) {
        /* Within 10% of zero */
        leftYstick = 0;//left joystick y axis in documentation. assign later
    }

    //public void armPID(double armLengthSetpoint){
       // armLengthController.setSetpoint(armLengthSetpoint);
        //double output = armLengthController.calculate(getArmLength(), armLengthSetpoint);
        //System.out.println("output: " + output);
        //armMotor.set(ControlMode.PercentOutput, output);//written as _talon.set(TalonFXControlMode.PercentOutput, leftYstick); in documentation. leftystick is joy.gety
    //}

    public void telescoping(double armSetpoint){//setpoint in inches
        armSetpoint = armSetpoint * Constants.TICKS_PER_INCH;//will probably have to change because wheel diameter in this constant is not relevant
        armMotor.set(ControlMode.Position, armSetpoint);//change to whatever motor we use for arm
    }

}
