package frc.robot.subsystems;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kForward;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kOff;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kReverse;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorMatch;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import frc.robot.Constants;


public class PneumaticIntakeSubsystem {
    private static boolean isPurple;
    private static boolean isYellow;

    public static enum PneumaticIntakeStates{
        ACTUATING, 
        RETRACTING, 
        OFF;
    }
    String colorString;
    public static PneumaticIntakeStates pneumaticIntakeState = PneumaticIntakeStates.ACTUATING;

    public static final double COLOR_THRESHOLD = 0.03;
    private static DoubleSolenoid solenoidOne = new DoubleSolenoid(10, PneumaticsModuleType.REVPH, 5, 7); 
    // Initializes a DigitalInput on DIO 0 (roborio is built in w/ 10 DIOs (digital input-output ports))
    private DigitalInput beambreakSensor = new DigitalInput(Constants.BEAM_BREAK_RECEIVER); 
    public Compressor compressor = new Compressor(PneumaticsModuleType.REVPH); 

    /**
     * Change the I2C port below to match the connection of your color sensor
    */
    private final I2C.Port i2cPort = I2C.Port.kOnboard;

    /**
     * A Rev Color Sensor V3 object is constructed with an I2C port as a 
     * parameter. The device will be automatically initialized with default 
     * parameters.
     */
    private ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);

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
        //solenoidOne.set(kOff);
        System.out.println("solenoid set to off");
        colorMatcher.addColorMatch(kPurpleTarget);
        colorMatcher.addColorMatch(kYellowTarget);
        colorMatcher.addColorMatch(kUnknownTarget);
        //compressor.enableDigital();
    }

    public void setStatePneumaticIntake(PneumaticIntakeStates newState){
        pneumaticIntakeState = newState;
    }

    public void switchState_colorSensor(){
        Color detectedColor = colorSensor.getColor();

         /*
         * Run the color match algorithm on our detected color
         */
        ColorMatchResult match = colorMatcher.matchClosestColor(detectedColor);

        boolean purpleNotRedThreshold = (Math.abs(detectedColor.red-kPurpleTarget.red)>COLOR_THRESHOLD);
        boolean purpleNotGreenThreshold = (Math.abs(detectedColor.green-kPurpleTarget.green)>COLOR_THRESHOLD);
        boolean purpleNotBlueThreshold = (Math.abs(detectedColor.blue-kPurpleTarget.blue)>COLOR_THRESHOLD);

        boolean yellowNotRedThreshold = (Math.abs(detectedColor.red-kYellowTarget.red)>COLOR_THRESHOLD);
        boolean yellowNotGreenThreshold = (Math.abs(detectedColor.green-kYellowTarget.green)>COLOR_THRESHOLD);
        boolean yellowNotBlueThreshold = (Math.abs(detectedColor.blue-kYellowTarget.blue)>COLOR_THRESHOLD);
        
        if((purpleNotRedThreshold && purpleNotGreenThreshold) || (purpleNotRedThreshold &&purpleNotBlueThreshold) || (purpleNotGreenThreshold && purpleNotBlueThreshold)){
            if((yellowNotRedThreshold && yellowNotGreenThreshold) || (yellowNotRedThreshold && yellowNotBlueThreshold) || (yellowNotGreenThreshold && yellowNotBlueThreshold)){
                colorString = "Unknown";
                //match should become black here
                match = colorMatcher.matchClosestColor(new Color(0,0,0));
            }
        }

        if (match.color == kPurpleTarget) {
            colorString = "Purple";
            isPurple = true;
            isYellow = false;
        } else if (match.color == kYellowTarget){//means its yellow :DDD
            colorString = "Yellow";
            isYellow = true;
            isPurple = false;
        } else{
            colorString = "Unknown"; 
            isPurple = false;
            isYellow = false;
        }
        System.out.println(colorString + " detected");
    }

    public void periodic(){
       if(pneumaticIntakeState == PneumaticIntakeStates.ACTUATING){
            solenoidOne.set(kForward);
            //System.out.println("Solenoid Actuating");
        } else if (pneumaticIntakeState == PneumaticIntakeStates.RETRACTING){
            solenoidOne.set(kReverse);
            //System.out.println("Solenoid Retracting");
        }else{
            solenoidOne.set(kOff);
            //System.out.println("Solenoid Off");
        }
    }

    public boolean getYellow(){
        return isYellow;
    }

    public boolean getPurple(){
        return isPurple;
    }

    public void setState(PneumaticIntakeStates newPneumaticIntakeState){
        pneumaticIntakeState = newPneumaticIntakeState;
    }

    public boolean isBeamBroken(){
        return beambreakSensor.get(); 
    }

    
}
