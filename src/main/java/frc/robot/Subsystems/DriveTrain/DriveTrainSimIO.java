package frc.robot.Subsystems.DriveTrain;

import edu.wpi.first.math.geometry.Rotation2d;

import frc.robot.Constants.RobotConstants;

public class DriveTrainSimIO extends DriveTrain {
    
    private Rotation2d angle;
    private double gyro_rate;

    protected DriveTrainSimIO() {
        angle = new Rotation2d();
    }

    public Rotation2d getGyroAngle() {
        if (angle != null) return angle;
        return new Rotation2d();
    }
    
    public double getGyroRate(){
        return gyro_rate;
    }

    public void setGyroAngle(double angle) {
        this.angle = Rotation2d.fromDegrees(-angle);
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
        Rotation2d diff = Rotation2d.fromRadians(
            kinematics.toChassisSpeeds(getModuleStates()).omegaRadiansPerSecond * RobotConstants.LOOP_TIME_SECONDS);
        gyro_rate = diff.getDegrees();
        angle = angle.plus(diff);
    }
}
