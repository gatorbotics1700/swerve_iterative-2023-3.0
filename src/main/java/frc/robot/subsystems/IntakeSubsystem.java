package frc.robot.subsystems;

import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

//karys did this!
public class IntakeSubsystem {
    public static TalonFX intakeMotor = new TalonFX(Constants.INTAKE_MOTOR_ID);

    IntakeStates intakeState = IntakeStates.STOP;

    public IntakeSubsystem(){

    }

    public static enum IntakeStates{
        STOP,
        FORWARD,
        BACKWARD;
    }

    public void init(){
        intakeMotor.setInverted(false);
    }

    public void periodic(){
        if (intakeState == IntakeStates.STOP){
            intakeMotor.set(ControlMode.PercentOutput, 0.0);
        } else if (intakeState == IntakeStates.FORWARD){
            intakeMotor.set(ControlMode.PercentOutput, 0.5);
        } else {
            intakeMotor.set(ControlMode.PercentOutput, -0.5);
        }
    }

    public void runMotors(){
        intakeMotor.set(ControlMode.PercentOutput, -0.5);
    }

}