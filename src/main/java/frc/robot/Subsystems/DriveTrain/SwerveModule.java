package frc.robot.Subsystems.DriveTrain;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotState;

public abstract class SwerveModule {

    public SwerveModuleState target_state;

    protected SwerveModule() {
        target_state = new SwerveModuleState();

    }

    public void setState(SwerveModuleState state) {
        target_state = state;
    }

    public void update() {
        setSpeed(target_state.speedMetersPerSecond);
        setAngle(target_state.angle);
    }

    abstract double getSpeed();
    abstract double getPosition();
    abstract Rotation2d getAngle();
    abstract SwerveModuleState getModuleState();
    abstract SwerveModulePosition getModulePosition();
    
    protected abstract void setSpeed(double speed);
    protected abstract void setAngle(Rotation2d angle);
}
