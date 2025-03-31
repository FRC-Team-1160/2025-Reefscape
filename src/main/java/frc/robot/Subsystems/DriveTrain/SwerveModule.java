package frc.robot.Subsystems.DriveTrain;

import java.util.List;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/** Represents a single swerve module. */
public abstract class SwerveModule {

    public record FullModuleState(SwerveModuleState state, double acceleration) {
        public static FullModuleState kZero = new FullModuleState(new SwerveModuleState());

        public FullModuleState(SwerveModuleState state) {
            this(state, 0);
        }
    }

    public FullModuleState target_state;
    public Translation2d offset;

    /** Class constructor. */
    protected SwerveModule() {
        target_state = FullModuleState.kZero;
    }

    /** 
     * Sets the target state for this module.
     * @param state The new target state.
     */
    public void setState(SwerveModuleState state) {
        setState(new FullModuleState(state));
    }

    /**
     * Sets the target state for this module, including acceleration.
     * @param state The new target state.
     */
    public void setState(FullModuleState state) {
        target_state = state;
    }

    /**
     * Resends input to the motors.
     */
    public void update() {
        applyState(target_state);
    }

    public void setSpeed(double speed) {
        setSpeed(speed, 0);
    }

    public void applyState(SwerveModuleState state) {
        applyState(new FullModuleState(state));
    }

    public void applyState(FullModuleState state) {
        setSpeed(state.state.speedMetersPerSecond, state.acceleration);
        setAngle(state.state.angle);
    }

    abstract double getSpeed();
    abstract double getPosition();
    abstract Rotation2d getAngle();
    abstract SwerveModuleState getModuleState();
    abstract SwerveModulePosition getModulePosition();

    protected abstract void setSpeed(double speed, double acceleration);
    protected abstract void setAngle(Rotation2d angle);

    public abstract List<TalonFX> getTalons();
}
