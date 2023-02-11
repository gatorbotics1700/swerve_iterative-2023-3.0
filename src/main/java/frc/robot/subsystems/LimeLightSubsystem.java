package frc.robot.subsystems;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardComponent;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import frc.robot.subsystems.*;
public class LimelightSubsystem {
    public final double MOUNTINGANGLE = 2.0; //degrees change
    public final double LOWERHEIGHT = 22.124; //inches height of bottom to bottom of tape of shorter pole
    public final double HIGHERHEIGHT = 41.125; //inches height of bottom to bottom of tape of shorter pole
    public final double ROBOTHEIGHT = 20.0; //inches change
    public final double IDEALDISTANCE = 40.0; //inches change
    public static double initialPosition;
    public static double tx_0;
    public static double tx_1;
    public static double tx_2;
    private static double tv;
    private static double tx;
    private static double ty;
    private static double ta;

    private AprilTagSubsystem aprilTagSubsystem = new AprilTagSubsystem();
    // this variable determines whether the Limelight has a valid target
    public void reset(){
        tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0.0);
        tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0.0);
        ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0.0);
        ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0.0);
    }
    public static void limelightData(){
        tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0.0);
        tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0.0);
        ty = NetworkTableInstance.getDefault().getTable("Limelight").getEntry("ty").getDouble(0.0);
        ta = NetworkTableInstance.getDefault().getTable("Limelight").getEntry("ta").getDouble(0.0);
        SmartDashboard.putNumber("tv:" , tv);
        SmartDashboard.putNumber("tx:", tx);
        SmartDashboard.putNumber("ty:", ty);
        SmartDashboard.putNumber("ta:", ta);
    }
    /*
`   pipeline declaring in c++
    std::shared_ptr<NetworkTable> networkTable = nt::NetworkTableInstance::GetDefault()
            .GetTable(“limelight”);
    networkTable->PutNumber(“pipeline”, pipeline);
    */
    public NetworkTable networkTable = NetworkTableInstance.getDefault().getTable("limelight");
    public void setPipeline(double pipeline){
        
        networkTable.getEntry("pipeline").setNumber(pipeline);
        System.out.println("pipeline:" + networkTable.getEntry("pipeline").getDouble(0.0));
        System.out.println("tv: " + networkTable.getEntry("tv").getDouble(0.0));
    }
    public static enum LimelightStates{
        SCANAPRILTAG,
        SCANTAPE,
        OFF;
    }
    public static LimelightStates state = LimelightStates.OFF;
    public void init(){
        setState(LimelightStates.SCANAPRILTAG);
        //initialPosition = DrivetrainSubsystem.getTicks();
        tx_0 = 0.0;
        tx_1 = 0.0;
    }
    public void periodic(){
        if (state == LimelightStates.SCANAPRILTAG){ //turret
            networkTable.getEntry("ledMode").setNumber(1); //force led off, can be customized in pipeline
            aprilTagSubsystem.setState(AprilTagSubsystem.AprilTagSequence.DETECT);
        } else if (state == LimelightStates.SCANTAPE) { //turret
            networkTable.getEntry("ledMode").setNumber(3); //force led on
        } else {
            /*shootSubsystem.setState(ShootState.OFF);
            intakeSubsystem.setState(IntakeStates.OFF);
            transitionSubsystem.setState(TransitionState.OFF);
            sensorSubsystem.setState(SensorStates.OFF);*/
        }
    }
    public static void setState(LimelightStates newState){
        state = newState;
    }
    // turning tx and ty into distance (in)
    public double yDistanceFromIdeal(){
        double distanceFromTarget = (LOWERHEIGHT- ROBOTHEIGHT)/Math.tan(Math.toRadians(MOUNTINGANGLE + ty));
        System.out.println("distance from target: " + distanceFromTarget);
        double distanceFromIdeal = distanceFromTarget - IDEALDISTANCE;
        System.out.println("distance from ideal: " + distanceFromIdeal);
        return distanceFromIdeal;
    }
    public void scanPipeline(){
        reset();
        if(tv==1.0){
            //do stuff
        }
    }
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
        return tx;
    }
//a bit worried about the infinate loop possibilities between scanzero and scanone
}
