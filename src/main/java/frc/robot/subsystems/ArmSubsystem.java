package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.math.controller.PIDController;

public class ArmSubsystem {

    //variables go here
    double armLengthKP = 0.04; //actual vals to be determined
    double armLengthKI = 0.000002;
    double armLengthKD = 0.005;
    PIDController armLengthController = new PIDController(armLengthKP, armLengthKI, armLengthKD);

    TalonFX armMotor = new TalonFX(Constants.ARM_MOTOR_ID);

    public ArmSubsystem(){

    }

    public double getArmLength(){
        return armMotor.getSelectedSensorPosition();
    }

    public void armPID(double armLengthSetpoint){
        armLengthController.setSetpoint(armLengthSetpoint);
        double output = armLengthController.calculate(getArmLength(), armLengthSetpoint);
        System.out.println("output: " + output);
        armMotor.set(ControlMode.PercentOutput, output);
    }

    public void telescoping(double armSetpoint){//setpoint in inches
        armSetpoint = armSetpoint * Constants.TICKS_PER_INCH;//will probably have to change because wheel diameter in this constant is not relevant
        armMotor.set(ControlMode.Position, armSetpoint);//change to whatever motor we use for arm
    }

}
