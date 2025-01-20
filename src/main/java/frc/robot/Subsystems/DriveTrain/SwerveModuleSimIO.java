package frc.robot.Subsystems.DriveTrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

public class SwerveModuleSimIO extends SwerveModule {

    public Rotation2d m_angle;
    public double m_speed;
    public double position;

    public PIDController sim_angle_pid;
    public PIDController sim_velocity_pid;

    public SwerveModuleSimIO() {
        m_angle = new Rotation2d();
        m_speed = 0;

        sim_angle_pid = new PIDController(0.1, 0, 0); //tune to actual swerve
        sim_angle_pid.enableContinuousInput(-0.5, 0.5);
        sim_velocity_pid = new PIDController(0.4, 0, 0); //tune to actual swerve
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
        m_angle = m_angle.plus(Rotation2d.fromRotations(sim_angle_pid.calculate(m_angle.getRotations())));
        m_speed += sim_velocity_pid.calculate(m_speed);
        position += m_speed * 0.02;
    }

    public Rotation2d getAngle() { return m_angle; }

    public double getSpeed() { return m_speed; }

    public double getPosition() { return position; }

    public SwerveModuleState getModuleState() {
        return new SwerveModuleState(getSpeed(), getAngle());
    }

    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(getPosition(), getAngle());
    }
    
}
