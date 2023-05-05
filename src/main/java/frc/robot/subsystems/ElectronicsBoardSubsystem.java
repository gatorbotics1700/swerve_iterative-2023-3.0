package frc.robot.subsystems;

import frc.robot.OI;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.ControlMode;

public class ElectronicsBoardSubsystem {
    TalonFX motor = new TalonFX(Constants.ELECTRONICS_BOARD_MOTOR_ID);

    public void runMotor(){
        motor.set(ControlMode.PercentOutput, OI.getElecAxis());
    }
}
