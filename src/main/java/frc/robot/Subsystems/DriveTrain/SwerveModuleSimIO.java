package frc.robot.Subsystems.DriveTrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModuleSimIO extends SwerveModule {

    public Rotation2d angle;
    public double speed;
    public double position;

    public PIDController sim_angle_pid;
    public PIDController sim_velocity_pid;

    public SwerveModuleSimIO() {
        angle = new Rotation2d();
        speed = 0;

        sim_angle_pid = new PIDController(1, 0, 0); //tune to actual swerve
        sim_angle_pid.enableContinuousInput(-0.5, 0.5);
        sim_velocity_pid = new PIDController(0.5, 0, 0); //tune to actual swerve
    }

    public void setAngle(Rotation2d angle) {
        sim_angle_pid.setSetpoint(angle.getRotations());
    }

    public void setSpeed(double speed) {
        sim_velocity_pid.setSetpoint(speed);
    }

    @Override
    public void update() {
        super.update();
        angle = angle.plus(Rotation2d.fromRotations(sim_angle_pid.calculate(angle.getRotations())));
        speed += sim_velocity_pid.calculate(speed);
        position += speed * 0.02;
    }

    public Rotation2d getAngle() { return angle; }

    public double getSpeed() { return speed; }

    public double getPosition() { return position; }

    protected SwerveModuleState getModuleState() {
        return new SwerveModuleState(getSpeed(), getAngle());
    }

    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(getPosition(), getAngle());
    }
    
}
