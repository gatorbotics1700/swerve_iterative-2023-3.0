package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.ControlMode;
import frc.robot.Gains;

public class ArmTelescopingSubsystem {

    //PIDController armLengthController = new PIDController(armLengthKP, armLengthKI, armLengthKD);
    public static TelescopingStates tState = TelescopingStates.RETRACTED;//should this be retracted or mid? what is the equivalent to off?

    TalonFX telescopingMotor = new TalonFX(Constants.TELESCOPING_MOTOR_ID);//maybe this motor should be renamed to make it more descriptive
    double startTime;
    public double tareEncoder;
    //armMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, Constants.kPIDLoopIdx, Constants.kTimeoutMs);//determine what these values would be for us

    public int _kIzone = 0;
    public double _kPeakOutput = 1.0;

    double telescopeKP = 0.01;
    double telescopeKD = 0;
    double telescopeKI = 0;
    double extensionVal = 56.26;//this value should be the full extension length of the arm minus the length of it at zero
    public Gains telescopeGains = new Gains(telescopeKP, telescopeKI, telescopeKD, _kIzone, _kPeakOutput);

    public static enum TelescopingStates{
        RETRACTED, //zero state
        FULLY_EXTENDED,
        LOW_ARM_LENGTH,
        MID_ARM_LENGTH,
        HIGH_ARM_LENGTH;
    }

    public ArmTelescopingSubsystem(){}

    public void init(){
        System.out.println("telescoping init!! :)");
        telescopingMotor.setInverted(true); //sets the forward direction of the motor to counter clockwise
        startTime = System.currentTimeMillis();

    }

    public void periodic(){//sam requests that we can operate arm length by stick on xbox
        System.out.println("current telescoping arm motor position:" + telescopingMotor.getSelectedSensorPosition());
        
        //all of the motor values need to be changed to ticks
        if (tState == TelescopingStates.RETRACTED){
            telescopingMotor.set(ControlMode.Position, 0);
        } else if (tState == TelescopingStates.FULLY_EXTENDED){
            telescopingMotor.set(ControlMode.Position, (extensionVal * Constants.TICKS_PER_INCH)-tareEncoder); //confirmed
        } else if (tState == TelescopingStates.LOW_ARM_LENGTH){
            telescopingMotor.set(ControlMode.Position, (10 * Constants.TICKS_PER_INCH)-tareEncoder); // replace position value w low length
            System.out.println("error: " + (10 * Constants.TICKS_PER_INCH - telescopingMotor.getSelectedSensorPosition()));
        }else if (tState == TelescopingStates.MID_ARM_LENGTH){
            telescopingMotor.set(ControlMode.Position, (5 * Constants.TICKS_PER_INCH)-tareEncoder); // goes with 90 degrees rotation // replace position value w mid length
        }else{
            telescopingMotor.set(ControlMode.Position, (15 * Constants.TICKS_PER_INCH)-tareEncoder); // replace position value w high length
        }
    }
    

    
    public void moveArm(double armSetpoint){//setpoint in inches
        armSetpoint = armSetpoint * Constants.TICKS_PER_INCH;//will probably have to change because wheel diameter in this constant is not relevant
        telescopingMotor.set(ControlMode.Position, armSetpoint);//change to whatever motor we use for arm
    }

    public double getArmPosition(){
        return telescopingMotor.getSelectedSensorPosition();
    }
    
    public void timedMoveArm(double time, boolean forwards){ //time in seconds
        System.out.println("moving arm >:)");
        System.out.println("start time: " + startTime);
        double milliTime = time * 1000;
        System.out.println("milli time: " + milliTime);
        if(forwards == true){
            while(System.currentTimeMillis() - startTime <= milliTime){
                telescopingMotor.set(ControlMode.PercentOutput,0.2);
            }
            telescopingMotor.set(ControlMode.PercentOutput,0);
        }else{
            while(System.currentTimeMillis() - startTime <= milliTime){
                telescopingMotor.set(ControlMode.PercentOutput,-0.2);
            }
            telescopingMotor.set(ControlMode.PercentOutput,0);
        }
        
    }

    public void setTState(TelescopingStates newState){
        tState = newState;
    }

    public void tareEncoder() {
        tareEncoder = telescopingMotor.getSelectedSensorPosition();

    }
    public double getTicks() {
        return telescopingMotor.getSelectedSensorPosition() - tareEncoder;
    }

}
