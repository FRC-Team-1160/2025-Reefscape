package frc.robot.Subsystems.DriveTrain;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class FullModuleState {
        
    public SwerveModuleState base_state;
    private double acceleration;
    public double angular_speed;
    public boolean relative_acceleration;

    public FullModuleState(SwerveModuleState state, 
                            double acceleration, 
                            double angular_speed, 
                            boolean relative_acceleration) {
        this.base_state = state;
        this.acceleration = acceleration;
        this.angular_speed = angular_speed;
        this.relative_acceleration = relative_acceleration;
    }

    public FullModuleState(SwerveModuleState state) {
        this(state, 0, 0, false);
    }

    public FullModuleState() {
        this(new SwerveModuleState());
    }

    public FullModuleState withState(SwerveModuleState state) {
        this.base_state = state;
        return this;
    }

    public FullModuleState withAcceleration(double acceleration, boolean relative) {
        this.acceleration = acceleration;
        this.relative_acceleration = relative;
        return this;
    }

    public FullModuleState withAngularSpeed(double omegaRadiansPerSecond) {
        this.angular_speed = omegaRadiansPerSecond;
        return this;
    }

    public double getAcceleration() {
        return relative_acceleration ? acceleration * Math.signum(base_state.speedMetersPerSecond) : acceleration;
    }

    public boolean optimize(Rotation2d currentAngle) {
        var delta = base_state.angle.minus(currentAngle);
        if (Math.abs(delta.getDegrees()) > 90.0) {
            base_state.speedMetersPerSecond *= -1;
            base_state.angle = base_state.angle.rotateBy(Rotation2d.kPi);
            return true;
        }
        return false;
    }
}