package frc.robot.subsystems.Vision;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardComponent;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import frc.robot.subsystems.*;
import frc.robot.Constants; 
public class LimeLightSubsystem {
    public final double MOUNTINGANGLE = 2.0; //degrees change
    public final double LOWERHEIGHT = 22.124; //inches height of bottom to bottom of tape of shorter pole
    public final double HIGHERHEIGHT = 41.125; //inches height of bottom to bottom of tape of shorter pole
    public final double ROBOTHEIGHT = 20.0; //inches change
    public final double IDEALDISTANCE = 40.0; //inches change
    public static double initialPosition;
    public static double tv;
    public static double tx;
    public static double ty;
    public static double ta;
    public static boolean readyToShoot;

    public NetworkTable networkTable = NetworkTableInstance.getDefault().getTable("limelight");

    private static AprilTagSubsystem aprilTagSubsystem = new AprilTagSubsystem();
    // this variable determines whether the Limelight has a valid target
    public void reset(){
        tv = networkTable.getEntry("tv").getDouble(0.0);
        tx = networkTable.getEntry("tx").getDouble(0.0);
        ty = networkTable.getEntry("ty").getDouble(0.0);
        ta = networkTable.getEntry("ta").getDouble(0.0);
    }
    public static void limelightData(){
        SmartDashboard.putNumber("tv:" , tv);
        SmartDashboard.putNumber("tx:", tx);
        SmartDashboard.putNumber("ty:", ty);
        SmartDashboard.putNumber("ta:", ta);
    }
    
    public void setPipeline(double pipeline){
        
        networkTable.getEntry("pipeline").setNumber(pipeline);
        System.out.println("pipeline:" + networkTable.getEntry("pipeline").getDouble(0.0));
        System.out.println("tv: " + networkTable.getEntry("tv").getDouble(0.0));
    }
    public static enum LimelightStates{
        SCANTAPE,
        OFF;   
    } 
    public static LimelightStates state = LimelightStates.OFF;
    public void init(){
        setState(LimelightStates.SCANTAPE);
        //initialPosition = DrivetrainSubsystem.getTicks();

    }
    public void periodic(){
        if (state == LimelightStates.SCANTAPE) { 
            double yDistance = (Constants.TAPE_HEIGHT_ONE-Constants.LIMELIGHT_HEIGHT) / Math.tan(Constants.LIMELIGHT_ANGLE+ty);
            double xDistance = yDistance / Math.tan((90 - DrivetrainSubsystem.m_pose.getRotation().getDegrees()));
        }else { //off
            
        }
    }
    public static void setState(LimelightStates newState){
        state = newState;
    }
    // turning tx and ty into distance (in)
    /*public double yDistanceFromIdeal(){
        double distanceFromTarget = (LOWERHEIGHT- ROBOTHEIGHT)/Math.tan(Math.toRadians(MOUNTINGANGLE + ty));
        System.out.println("distance from target: " + distanceFromTarget);
        double distanceFromIdeal = distanceFromTarget - IDEALDISTANCE;
        System.out.println("distance from ideal: " + distanceFromIdeal);
        return distanceFromIdeal;
    }*/
    
    public boolean isThereTarget(){
        if(tv != 0){
            return true;
        }
        return false;
    }
    public double getTv(){
        return networkTable.getEntry("tv").getDouble(0.0);
    }
    public double getTx(){
        return networkTable.getEntry("tx").getDouble(0.0);
    }

    public boolean seeSomething(){
        if(tv == 1){
            return true;
        }
        else{
            return false;
        }
    }

}
