package frc.robot.Subsystems.DriveTrain;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Constants.RobotConstants;

public class DriveTrainSimIO extends DriveTrain {
    
    Rotation2d angle;

    protected DriveTrainSimIO() {
        angle = new Rotation2d();
    }

    public Rotation2d getGyroAngle() {
        if (angle != null) return angle;
        return new Rotation2d();
    }
    
    public double getGyroRate(){
        return 0.0;
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
        angle = angle.plus(Rotation2d.fromRadians(
            kinematics.toChassisSpeeds(getModuleStates()).omegaRadiansPerSecond * RobotConstants.LOOP_TIME_SECONDS));
    }
}
