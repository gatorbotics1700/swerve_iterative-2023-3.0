package frc.robot.subsystems;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kForward;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kOff;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kReverse;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.Constants;
import frc.robot.OI;

public class PneumaticIntakeSubsystem {
    public static enum PneumaticIntakeStates{
        ACTUATING, 
        RETRACTING, 
        OFF;
    }

    public static PneumaticIntakeStates pneumaticIntakeState = PneumaticIntakeStates.OFF;

    public double colorThreshold = 0.03;
    //we are using double solenoid and a rev pneumatics hub
    public static DoubleSolenoid solenoidOne = new DoubleSolenoid(7, PneumaticsModuleType.REVPH, 8, 10); 
    public static Compressor compressor = new Compressor(PneumaticsModuleType.REVPH); 
    // Initializes a DigitalInput on DIO 0 (roborio is built in w/ 10 DIOs (digital input-output ports))
    private DigitalInput beambreakSensor = new DigitalInput(Constants.intakeSensor); 

    /**
     * Change the I2C port below to match the connection of your color sensor
     */
    private final I2C.Port i2cPort = I2C.Port.kOnboard;

    /**
     * A Rev Color Sensor V3 object is constructed with an I2C port as a 
     * parameter. The device will be automatically initialized with default 
     * parameters.
     */
    private ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort); //random port val

    /**
     * A Rev Color Match object is used to register and detect known colors. This can 
     * be calibrated ahead of time or during operation.
     * 
     * This object uses a simple euclidian distance to estimate the closest match
     * with given confidence range.
     */
    private final ColorMatch colorMatcher = new ColorMatch();

    /**
     * Note: Any example colors should be calibrated as the user needs, these
     * are here as a basic example.
     */
    
    // private final Color kPurpleTarget = new Color(0.494, 0.020, 0.910); //rgb: 126, 5, 232 // acc purple
    // private final Color kYellowTarget = new Color(0.941, 0.745, 0.039); //rgb: 240, 190, 10 // acc yellow 
    private final Color kPurpleTarget = new Color(0.218, 0.367, 0.415); // fake purple (what it sees - aka teal)
    private final Color kYellowTarget = new Color(0.355, 0.547, 0.100); //fake yellow (what it sees - aka marsh green)
    private final Color kUnknownTarget = new Color(0,0,0); //black

    public void init(){
        solenoidOne.set(kOff);
        System.out.println("solenoid set to off");
        colorMatcher.addColorMatch(kPurpleTarget);
        colorMatcher.addColorMatch(kYellowTarget);
        colorMatcher.addColorMatch(kUnknownTarget);
    }

    public void setStatePneumaticIntake(PneumaticIntakeStates newState){
        pneumaticIntakeState = newState;
    }

    public void switchState_beamBreakSensor (){ //switch state based on sensor reading
        if (beambreakSensor.get() == true ){ //circuit is open meaning it sees something
            //setState(PneumaticIntakeStates.ACTUATING);
        } else { // circuit is closed meaning it doesn't seem something
            //setState(PneumaticIntakeStates.RETRACTING); //confirmed b/c OFF stops/disables the solenoids
        }
        System.out.println("beambreak sensor: " + beambreakSensor.get());
    }

    public void switchState_colorSensor(){
        Color detectedColor = colorSensor.getColor();

         /**
         * Run the color match algorithm on our detected color
         */
        String colorString;
        ColorMatchResult match = colorMatcher.matchClosestColor(detectedColor);
        
        if((Math.abs(detectedColor.red-kPurpleTarget.red)>colorThreshold && Math.abs(detectedColor.green-kPurpleTarget.green)>colorThreshold) || (Math.abs(detectedColor.red-kPurpleTarget.red)>colorThreshold && Math.abs(detectedColor.blue-kPurpleTarget.blue)>colorThreshold) || (Math.abs(detectedColor.green-kPurpleTarget.green)>colorThreshold && Math.abs(detectedColor.blue-kPurpleTarget.blue)>colorThreshold)){
            System.out.println("diff w/ purple red: " + (detectedColor.red-kPurpleTarget.red));
            System.out.println("diff w/ purple green: " + (detectedColor.green-kPurpleTarget.green));
            System.out.println("diff w/ purple blue: " + (detectedColor.blue-kPurpleTarget.blue));

            System.out.println("here!!!!");
            if((Math.abs(detectedColor.red-kYellowTarget.red)>colorThreshold && Math.abs(detectedColor.green-kYellowTarget.green)>colorThreshold) || (Math.abs(detectedColor.red-kYellowTarget.red)>colorThreshold && Math.abs(detectedColor.blue-kYellowTarget.blue)>colorThreshold) || (Math.abs(detectedColor.green-kYellowTarget.green)>colorThreshold && Math.abs(detectedColor.blue-kYellowTarget.blue)>colorThreshold)){
                System.out.println("diff w/ yellos red: " + (detectedColor.red-kYellowTarget.red));
                System.out.println("diff w/ yellow green: " + (detectedColor.green-kYellowTarget.green));
                System.out.println("diff w/ yellow blue: " + (detectedColor.blue-kYellowTarget.blue));
                colorString = "Unknown";
                //match should become black here
                match = colorMatcher.matchClosestColor(new Color(0,0,0));
            }
        }
        
        if (match.color == kPurpleTarget) {
            colorString = "Purple";
        } else if (match.color == kYellowTarget){//means its yellow :DDD
            colorString = "Yellow";
        } else{
            colorString = "Unknown"; 
        }
        System.out.println(colorString + " detected");

        /**
         * Open Smart Dashboard or Shuffleboard to see the color detected by the 
         * sensor.
         */
        SmartDashboard.putNumber("Red", detectedColor.red);
        SmartDashboard.putNumber("Green", detectedColor.green);
        SmartDashboard.putNumber("Blue", detectedColor.blue);
        SmartDashboard.putNumber("Confidence", match.confidence);
        SmartDashboard.putString("Detected Color", colorString);
    }

    public void periodic(){
        if(pneumaticIntakeState == PneumaticIntakeStates.ACTUATING){
            solenoidOne.set(kForward);
            System.out.println("Solenoid Actuating");
        } else if (pneumaticIntakeState == PneumaticIntakeStates.RETRACTING){
            solenoidOne.set(kReverse);
            System.out.println("Solenoid Retracting");
        }else{
            solenoidOne.set(kOff);
            System.out.println("Solenoid Off");
        } 
        //switchState_beamBreakSensor(); 
        //switchState_colorSensor();
    }

    public boolean getPSI(){
        System.out.println(compressor.getCurrent());
        return compressor.getPressureSwitchValue();
    }

    public static void setState(PneumaticIntakeStates newPneumaticIntakeState){
        pneumaticIntakeState = newPneumaticIntakeState;
    }
    
}
