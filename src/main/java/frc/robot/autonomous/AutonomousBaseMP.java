package frc.robot.autonomous;

import java.util.ArrayList;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import java.lang.System.currentTimeMillis;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AutonomousBaseMP extends AutonomousBase{

    private double timeStart;
    private double timeElapsed = 0;
    private Pose2d starting;
    private Pose2d ending;
    private Translation2d interior1;
    private Translation2d interior2;
    private Translation2d interior3;
    private HolonomicDriveController controller = new HolonomicDriveController(
            new PIDController(1, 0, 0), new PIDController(1, 0, 0),
            new ProfiledPIDController(1, 0, 0
                new TrapezoidProfile.Constraints(6.28, Math.PI)));
        // trapezoid profile takes in max rotation velocity and max rotation acceleration 
        // PIDController #1: the first arg rep how many m/s added in the x direction for every meter of error in the x direction
        // PIDController #2 : the first arg rep how many m/s added in the y direction for every meter of error in the y direction

    public AutonomousBaseMP(Pose2d starting, Pose2d ending, Translation2d interior1, Translation2d interior2, Translation2d interior3){
        this.starting = starting;
        this.ending = ending;
        this.interior1 = interior1;
        this.interior2 = interior2;
        this.interior3 = interior3;
    }

    @Override
    public void init(){
        double timeStart = currentTimeMillis();
        generateTrajectory();
    }
    
    @Override
    public void periodic(){
        double timeElapsed = currentTimeMillis() - timeStart;
        followTrajectory();
    }

    public void generateTrajectory(){
        ArrayList<Translation2d> interiorWaypoints = new ArrayList<Translation2d>();
        interiorWaypoints.add(interior1);
        interiorWaypoints.add(interior2);
        interiorWaypoints.add(interior3);

        TrajectoryConfig config = new TrajectoryConfig(0, 0, false, {SwerveDriveKinematicsConstraint, MaxVelocityConstraint});

        Trajectory trajectory = TrajectoryGenerator.gerateTrajectory(
            starting,
            interiorWaypoints, 
            ending,
            config
        );
    }

    public void followTrajectory(Trajectory trajectory){
        
        Trajectory.State goal = trajectory.sample(timeElapsed);

        ChassisSpeeds adjustedSpeeds = controller.calculate(
            DrivetrainSubsystem.m_pose, goal, Rotation2d.fromDegrees(0));
        
        DrivetrainSubsystem.setSpeed(adjustedSpeeds);
    }

}