package frc.com.swervedrivespecialties.swervelib;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import com.ctre.phoenix.sensors.CANCoder;

public interface SwerveModule {
    double getDriveVelocity();

    double getSteerAngle();

    double getPosition();

    SwerveModulePosition getSwerveModulePosition();

    void set(double driveVoltage, double steerAngle);

    CANCoder getCANCoder();
}
