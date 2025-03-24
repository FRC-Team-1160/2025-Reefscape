package frc.robot.Subsystems.DriveTrain;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/** Represents a single swerve module. */
public abstract class SwerveModule {

    public SwerveModuleState target_state;
    public double acceleration_ff;

    /** Class constructor. */
    protected SwerveModule() {
        target_state = new SwerveModuleState();
        acceleration_ff = 0;
    }

    /** 
     * Sets the target state for this module.
     * @param state The new target state.
     */
    public void setState(SwerveModuleState state) {
        target_state = state;
    }

    /**
     * Resends input to the motors.
     */
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
