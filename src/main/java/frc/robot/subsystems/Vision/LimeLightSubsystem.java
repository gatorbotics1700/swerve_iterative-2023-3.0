package frc.robot.subsystems.Vision;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTable;

public class LimeLightSubsystem {
    public static double tv;

    public NetworkTable networkTable = NetworkTableInstance.getDefault().getTable("limelight");

    public void reset(){
        tv = networkTable.getEntry("tv").getDouble(0.0);
    }

    public void setPipeline(double pipeline){
        networkTable.getEntry("pipeline").setNumber(pipeline);
    }

    public double getTv(){
        return networkTable.getEntry("tv").getDouble(0.0);
    }
}
