package frc.robot.subsystems;
import frc.robot.OI;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;


public class IntakeSubsystem {
    public static TalonFX intakeMotor = new TalonFX(Constants.INTAKE_MOTOR_ID);

    IntakeStates intakeState = IntakeStates.STOP;
    
    public static enum IntakeStates{
        STOP,
        FORWARD,
        BACKWARD;
    }
    
    public static double intakeOn, intakeOff; //intakeOff is outake

    public void triggerPeriodic() {
        intakeOn = triggerThreshold (OI.m_codriver_controller.getLeftTriggerAxis());
        intakeOff = triggerThreshold (OI.m_codriver_controller.getRightTriggerAxis());
    }

    public static double triggerThreshold(double tValue) {
        if (Math.abs(tValue) < 0.1) { //change this value to be more specific if wanted
            return 0;
        } else {
            return 1 * tValue;
        }
        //trigger axis is bound by the range [0,1] not [-1,1]
    }

    public IntakeSubsystem(){

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

    public void runMotorsForward(){
        intakeMotor.set(ControlMode.PercentOutput, 0.5);
    }

    public void runMotorsBackward(){
        intakeMotor.set(ControlMode.PercentOutput, -0.5);
    }

    public void setState(IntakeStates newState){
        intakeState = newState;
    }

}