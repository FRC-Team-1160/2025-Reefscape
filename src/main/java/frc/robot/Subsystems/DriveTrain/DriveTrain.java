package frc.robot.Subsystems.DriveTrain; //Accidentally changed the folder name to be uppercase this year, oh well :P

import java.io.IOException;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.TalonFX;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Robot;
import frc.robot.Constants.PortConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.SwerveConstants;

/** The drive subsystem. */
public abstract class DriveTrain extends SubsystemBase {

    public static final DriveTrain instance = Robot.isReal() ? new DriveTrainRealIO() : new DriveTrainSimIO();

    /** The drive train's SwerveModule objects. */
    public SwerveModule[] modules;
    /** @hidden */
    public SwerveDriveKinematics kinematics;
    /** The desired module states. */
    @AutoLogOutput
    public SwerveModuleState[] module_states;

    public FullModuleState[] full_module_states;

    public record AccelerationFeedforward(double acceleration, boolean adjust) {
        public static AccelerationFeedforward kZero = new AccelerationFeedforward(0, false);
    }

    public AccelerationFeedforward[] accel_feedforwards;

    public RobotConfig config;

    /** Creates a new DriveTrain. */
    protected DriveTrain() {

        modules = new SwerveModule[] {
            initializeModule(PortConstants.DRIVE_MOTOR_FRONT_LEFT, PortConstants.STEER_MOTOR_FRONT_LEFT,
                PortConstants.FRONT_LEFT_CODER, new Translation2d(SwerveConstants.OFFSET, SwerveConstants.OFFSET)),
            initializeModule(PortConstants.DRIVE_MOTOR_FRONT_RIGHT, PortConstants.STEER_MOTOR_FRONT_RIGHT,
                PortConstants.FRONT_RIGHT_CODER, new Translation2d(SwerveConstants.OFFSET, -SwerveConstants.OFFSET)),
            initializeModule(PortConstants.DRIVE_MOTOR_BACK_LEFT, PortConstants.STEER_MOTOR_BACK_LEFT,
                PortConstants.BACK_LEFT_CODER, new Translation2d(-SwerveConstants.OFFSET, SwerveConstants.OFFSET)),
            initializeModule(PortConstants.DRIVE_MOTOR_BACK_RIGHT, PortConstants.STEER_MOTOR_BACK_RIGHT,
                PortConstants.BACK_RIGHT_CODER, new Translation2d(-SwerveConstants.OFFSET, -SwerveConstants.OFFSET))
        };

        module_states = Collections.nCopies(4, new SwerveModuleState()).toArray(SwerveModuleState[]::new);
        full_module_states = Collections.nCopies(4, new FullModuleState()).toArray(FullModuleState[]::new);

        kinematics = new SwerveDriveKinematics(
            modules[0].offset, // front left
            modules[1].offset, // front right
            modules[2].offset, // back left
            modules[3].offset // back right
        );

        accel_feedforwards = Collections.nCopies(4, AccelerationFeedforward.kZero)
            .toArray(AccelerationFeedforward[]::new);

        try {
            config = RobotConfig.fromGUISettings();
        } catch (IOException | ParseException e) {
            e.printStackTrace();
            config = null;
        }
        
        setupDashboard();
    }

    /**
     * Calculates and sends inputs to swerve modules given field-relative speeds. Calls {@link #setSwerveDrive(ChassisSpeeds)}.
     * @param x_speed    The X-axis speed in m/s. Forward is positive.
     * @param y_speed    The Y-axis speed in m/s. Left is positive.
     * @param a_speed    Angular speed in rad/s. CCW is positive.
     */
    public void setSwerveDrive(double x_speed, double y_speed, double a_speed) {
        // Convert speeds from field's frame of reference to robot's frame of reference
        setSwerveDrive(ChassisSpeeds.fromFieldRelativeSpeeds(x_speed, y_speed, a_speed, getGyroAngle()));
    }

    /**
     * Calculates and sends inputs to swerve modules given robot-relative speeds. Assumes discretization is needed. 
     * @param chassis_speeds The desired robot-relative chassis speeds.
     */
    public void setSwerveDrive(ChassisSpeeds chassis_speeds) {
        setSwerveDrive(chassis_speeds, true);
    }

    /**
     * Calculates and sends inputs to swerve modules given robot-relative speeds.
     * @param chassis_speeds The desired robot-relative chassis speeds.
     * @param discretize Whether or not to perform discretization. Some library methods provide pre-discretized chassis speeds.
     */
    public void setSwerveDrive(ChassisSpeeds chassis_speeds, boolean discretize) {

        Logger.recordOutput("DriveTrain/Input Speeds", chassis_speeds);

        if (discretize) chassis_speeds = discretize_chassis_speeds(chassis_speeds);

        module_states = kinematics.toSwerveModuleStates(chassis_speeds);

        // change target wheel directions if the wheel has to rotate more than 90*
        for (int i = 0; i < module_states.length; i++) {
            module_states[i].optimize(modules[i].getAngle());
        }

        // normalize wheel speeds of any are greater than max speed
        SwerveDriveKinematics.desaturateWheelSpeeds(module_states, SwerveConstants.MAX_SPEED);
        
        for (int i = 0; i < module_states.length; i++) {
            full_module_states[i].withState(module_states[i]).optimize(modules[i].getAngle());
        }

        setModules(module_states);
    }

    /**
     * Sends calculated inputs to swerve modules.
     * @param module_states The desired module states.
     */
    public void setModules(SwerveModuleState[] states) {
        for (int i = 0; i < modules.length; i++) {
            modules[i].setState(states[i]);
        }
    }

    public void setModules(FullModuleState[] states) {
        for (int i = 0; i < modules.length; i++) {
            modules[i].setState(states[i]);
        }
    }

    public void setModules(boolean reset) {
        setModules(full_module_states);
        full_module_states = Collections.nCopies(4, new FullModuleState()).toArray(FullModuleState[]::new);
    }

    /**
     * Accounts for drift while simultaneously translating and rotating through discretization.
     * @param speeds The desired chassis speeds.
     * @param multiplier The number of time loops to discretize over.
     * @return The adjusted chassis speeds.
     */
    public ChassisSpeeds discretize_chassis_speeds(ChassisSpeeds speeds, double multiplier) {
        double dt = RobotConstants.LOOP_TIME_SECONDS;
        // Makes a Pose2d for the target delta over one time loop
        var desired_delta_pose = new Pose2d(
                speeds.vxMetersPerSecond * dt,
                speeds.vyMetersPerSecond * dt,
                new Rotation2d(speeds.omegaRadiansPerSecond * dt * multiplier) // tunable
        );
        // Makes a Twist2d object that maps new pose to delta pose
        var twist = new Pose2d().log(desired_delta_pose);

        return new ChassisSpeeds((twist.dx / dt), (twist.dy / dt), (speeds.omegaRadiansPerSecond));
    }

    public ChassisSpeeds discretize_chassis_speeds(ChassisSpeeds speeds) {
        return discretize_chassis_speeds(speeds, 1);
    }

    /**
     * Returns the measured swerve module positions for odometry.
     * @return The measured swerve module positions.
     */
    public SwerveModulePosition[] getModulePositions() {
        return Arrays.stream(modules).map(module -> ((SwerveModule) module).getModulePosition())
            .toArray(SwerveModulePosition[]::new);
    }

    /**
     * Returns the measured swerve module states for odometry and telemetry.
     * @return The measured swerve module states.
     */
    @AutoLogOutput
    public SwerveModuleState[] getModuleStates() {
        return Arrays.stream(modules).map(module -> ((SwerveModule) module).getModuleState())
            .toArray(SwerveModuleState[]::new);
    }

    /**
     * Calculates current speeds using SwerveDriveKinematics odometry.
     * @return The calculated robot-relative chassis speeds.
     */
    public ChassisSpeeds getOdomSpeeds() {
        return kinematics.toChassisSpeeds(getModuleStates());
    }

    /**
     * Calculates current target speeds using SwerveDriveKinematics odometry and target states.
     * @return The calculated robot-relative chassis speeds.
     */
    public ChassisSpeeds getTargetOdomSpeeds() {
        return kinematics.toChassisSpeeds(module_states);
    }

    public double[] calculateAccelerationFeedforwards(Transform2d acceleration) {
        double mag = acceleration.getTranslation().getNorm();
        Rotation2d ang = acceleration.getTranslation().getAngle();
        double rot = acceleration.getRotation().getRotations();

        return Arrays.stream(modules).mapToDouble(
                module -> mag * ang.minus(module.getAngle()).getCos()
                     + rot * ang.minus(module.offset.getAngle().plus(Rotation2d.kPi)).getCos()
            ).toArray();
    }

    public double[] calculateSteerFeedforwards(ChassisSpeeds desired_speeds) {
        SwerveModuleState[] target_states = kinematics.toSwerveModuleStates(desired_speeds);
        double[] out_feedforwards = new double[modules.length];
        for (int i = 0; i < modules.length; i++) {
            out_feedforwards[i] = target_states[i].angle.minus(modules[i].getAngle())
                .div(RobotConstants.LOOP_TIME_SECONDS).getRotations();
        }
        return out_feedforwards;
    }

    public void setFeedforwards(double[] feedforwards, boolean adjustSign) {
        Logger.recordOutput("DriveTrain/Feedforwards", feedforwards);
        for (int i = 0; i < modules.length; i++) {
            full_module_states[i].withAcceleration(feedforwards[i], adjustSign);
        }
    }

    public void setSteerFeedforwards(double[] feedforwards) {
        Logger.recordOutput("DriveTrain/Steer Feedforwards", feedforwards);
        for (int i = 0; i < modules.length; i++) {
            full_module_states[i].withAngularSpeed(feedforwards[i]);
        }
    }

    public abstract List<TalonFX> getTalons();

    /**
     * One-time method to instantiate NT publishers for Elastic.
     */
    private void setupDashboard() {

        // Create swerve drive publishers for elastic dashboard (one time setup, auto-call lambdas)
        SmartDashboard.putData("Swerve Target States", new Sendable() {
            @Override
            public void initSendable(SendableBuilder builder) {
                builder.setSmartDashboardType("SwerveDrive");

                builder.addDoubleProperty("Front Left Angle", 
                    () -> module_states[0].angle.getDegrees(), null);
                builder.addDoubleProperty("Front Left Velocity", 
                    () -> module_states[0].speedMetersPerSecond, null);

                builder.addDoubleProperty("Front Right Angle", 
                    () -> module_states[1].angle.getDegrees(), null);
                builder.addDoubleProperty("Front Right Velocity", 
                    () -> module_states[1].speedMetersPerSecond, null);

                builder.addDoubleProperty("Back Left Angle", 
                    () -> module_states[2].angle.getDegrees(), null);
                builder.addDoubleProperty("Back Left Velocity", 
                    () -> module_states[2].speedMetersPerSecond, null);

                builder.addDoubleProperty("Back Right Angle", 
                    () -> module_states[3].angle.getDegrees(), null);
                builder.addDoubleProperty("Back Right Velocity", 
                    () -> module_states[3].speedMetersPerSecond, null);

                builder.addDoubleProperty("Robot Angle", 
                    () -> getGyroAngle().getDegrees(), null);
            }
        });
        // Same here
        SmartDashboard.putData("Swerve Real States", new Sendable() {
            @Override
            public void initSendable(SendableBuilder builder) {
                builder.setSmartDashboardType("SwerveDrive");

                builder.addDoubleProperty("Front Left Angle", 
                    () -> modules[0].getAngle().getDegrees(), null);
                builder.addDoubleProperty("Front Left Velocity", 
                    () -> modules[0].getSpeed(), null);

                builder.addDoubleProperty("Front Right Angle", 
                    () -> modules[1].getAngle().getDegrees(), null);
                builder.addDoubleProperty("Front Right Velocity", 
                    () -> modules[1].getSpeed(), null);

                builder.addDoubleProperty("Back Left Angle", 
                    () -> modules[2].getAngle().getDegrees(), null);
                builder.addDoubleProperty("Back Left Velocity", 
                    () -> modules[2].getSpeed(), null);

                builder.addDoubleProperty("Back Right Angle", 
                    () -> modules[3].getAngle().getDegrees(), null);
                builder.addDoubleProperty("Back Right Velocity", 
                    () -> modules[3].getSpeed(), null);

                builder.addDoubleProperty("Robot Angle", 
                    () -> getGyroAngle().getDegrees(), null);
            }
        });

    }

    /**
     * Gets either the measured yaw from the AHRS or the calculated angle from the
     * simulation.
     * Forward is 0, CCW is positive.
     * @return The robot yaw.
     */
    @AutoLogOutput
    public abstract Rotation2d getGyroAngle();
    /**
     * Returns the rate at which the gyro is spinning.
     * @return The gyro rate in degrees per second.
     */
    public abstract double getGyroRate();

    /**
     * Resets the gyroscope angle to 0.
     */
    public abstract void resetGyroAngle();

    public abstract void setGyroAngle(double angle);

    /**
     * Creates either a SwerveModuleRealIO or SwerveModuleSimIO object.
     * @param drive_port    The port number of the drive motor.
     * @param steer_port    The port number of the steer motor.
     * @param sensor_port   The port number of the module's CANcoder.
     * @param offset        The offset of the module from the robot center.
     * @return The constructed SwerveModule object.
     */
    protected abstract SwerveModule initializeModule(int drive_port, int steer_port, int sensor_port, Translation2d offset);

    @Override
    public void periodic() {
        for (SwerveModule module : modules) module.update();
    }
    
}
