package frc.robot.subsystems;
import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import frc.robot.Gains;

public class FourBarLinkageSubsystem {
    //these values are to be determined (untested)
    public double _kP = 1.0;
    public double _kI = 0.0;
    public double _kD = 0.0;
    public int _kIzone = 0;
    public double _kPeakOutput = 1.0;

    public static TalonFX leftLinkMotor = new TalonFX(Constants.LEFT_LINK_CAN_ID);
    public static TalonFX rightLinkMotor = new TalonFX(Constants.RIGHT_LINK_CAN_ID);
    public static LinkStateHolder linkStateHolder = LinkStateHolder.ZERO_HEIGHT;

    public Gains linkGains = new Gains(_kP, _kI, _kD, _kIzone, _kPeakOutput);

    public static enum LinkStateHolder{
        ZERO_HEIGHT,
        LOWEST_HEIGHT,
        MID_HEIGHT,
        HIGH_HEIGHT,
        FULLY_EXTENDED
    }
    public void init(){
        leftLinkMotor.setInverted(false);
        rightLinkMotor.setInverted(false);
        leftLinkMotor.setNeutralMode(NeutralMode.Brake);
        rightLinkMotor.setNeutralMode(NeutralMode.Brake);

        leftLinkMotor.configAllowableClosedloopError(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
        rightLinkMotor.configAllowableClosedloopError(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);

        leftLinkMotor.config_kP(Constants.kPIDLoopIdx, linkGains.kP, Constants.kTimeoutMs);
		leftLinkMotor.config_kI(Constants.kPIDLoopIdx, linkGains.kI, Constants.kTimeoutMs);
		leftLinkMotor.config_kD(Constants.kPIDLoopIdx, linkGains.kD, Constants.kTimeoutMs);

        rightLinkMotor.config_kP(Constants.kPIDLoopIdx, linkGains.kP, Constants.kTimeoutMs);
		rightLinkMotor.config_kI(Constants.kPIDLoopIdx, linkGains.kI, Constants.kTimeoutMs);
		rightLinkMotor.config_kD(Constants.kPIDLoopIdx, linkGains.kD, Constants.kTimeoutMs);
    }

    public void periodic(){
        if(linkStateHolder == LinkStateHolder.ZERO_HEIGHT) {
            leftLinkMotor.set(ControlMode.Position, 0); // the following if statements are not tested, and should probably be changed before testing(the positions)
            rightLinkMotor.set(ControlMode.Position, 0); 
        }
        else if(linkStateHolder == LinkStateHolder.LOWEST_HEIGHT) {
            leftLinkMotor.set(ControlMode.Position, 0); 
            rightLinkMotor.set(ControlMode.Position, 0); 
        }
        else if(linkStateHolder == LinkStateHolder.MID_HEIGHT) {
            leftLinkMotor.set(ControlMode.Position, 0);
            rightLinkMotor.set(ControlMode.Position, 0); 
        }
        else if(linkStateHolder == LinkStateHolder.HIGH_HEIGHT) {
            leftLinkMotor.set(ControlMode.Position, 0); 
            rightLinkMotor.set(ControlMode.Position, 0);
        }
        else{
            leftLinkMotor.set(ControlMode.Position, 0);
            rightLinkMotor.set(ControlMode.Position, 0); 
        }
    }

    public void setState(LinkStateHolder newLinkState){
        linkStateHolder = newLinkState;
    }
}
