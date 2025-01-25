package frc.robot.Subsystems.DriveTrain;

import edu.wpi.first.math.geometry.Rotation2d;

public class DriveTrainSimIO extends DriveTrain {
    
    Rotation2d angle;

    public DriveTrainSimIO() {
        angle = new Rotation2d();
    }

    public Rotation2d getGyroAngle() {
        if (angle != null) return angle;
        return new Rotation2d();
    }

    public void resetGyroAngle() {
        angle = new Rotation2d();
    }

    public SwerveModule initializeModule(int drive_port, int steer_port, int sensor_port){
        return new SwerveModuleSimIO();
    }

    @Override
    public void periodic() {
        super.periodic();
        angle = angle.plus(Rotation2d.fromRadians(kinematics.toChassisSpeeds(getModuleStates()).omegaRadiansPerSecond * 0.02));
    }
}
