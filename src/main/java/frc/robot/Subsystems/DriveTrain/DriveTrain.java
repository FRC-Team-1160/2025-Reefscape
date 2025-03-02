package frc.robot.Subsystems.DriveTrain; //Accidentally changed the folder name to be uppercase this year, oh well :P

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.PortConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.SwerveConstants;

public abstract class DriveTrain extends SubsystemBase {

    public static final DriveTrain instance = Robot.isReal() ? new DriveTrainRealIO() : new DriveTrainSimIO();

    /** The drive train's SwerveModule objects. */
    public SwerveModule[] modules;
    /** @hidden */
    public SwerveDriveKinematics kinematics;
    /** The desired module states. */
    public SwerveModuleState[] module_states;
    /** Odometry-based 2d pose. */
    public Pose2d odom_pose;
    /** State publisher for AdvantageScope. */
    protected StructArrayPublisher<SwerveModuleState> real_states_pub, target_states_pub;
    /** State publisher for AdvantageScope. */
    protected StructPublisher<Rotation2d> gyro_pub;

    /** Creates a new DriveTrain. */
    protected DriveTrain() {
        kinematics = new SwerveDriveKinematics(
                new Translation2d(SwerveConstants.OFFSET, SwerveConstants.OFFSET), // front left
                new Translation2d(SwerveConstants.OFFSET, -SwerveConstants.OFFSET), // front right
                new Translation2d(-SwerveConstants.OFFSET, SwerveConstants.OFFSET), // back left
                new Translation2d(-SwerveConstants.OFFSET, -SwerveConstants.OFFSET) // back right
        );

        modules = new SwerveModule[4];
        modules[0] = initializeModule(PortConstants.DRIVE_MOTOR_FRONT_LEFT, PortConstants.STEER_MOTOR_FRONT_LEFT,
            PortConstants.FRONT_LEFT_CODER);
        modules[1] = initializeModule(PortConstants.DRIVE_MOTOR_FRONT_RIGHT, PortConstants.STEER_MOTOR_FRONT_RIGHT,
                PortConstants.FRONT_RIGHT_CODER);
        modules[2] = initializeModule(PortConstants.DRIVE_MOTOR_BACK_LEFT, PortConstants.STEER_MOTOR_BACK_LEFT,
                PortConstants.BACK_LEFT_CODER);
        modules[3] = initializeModule(PortConstants.DRIVE_MOTOR_BACK_RIGHT, PortConstants.STEER_MOTOR_BACK_RIGHT,
                PortConstants.BACK_RIGHT_CODER);

        module_states = new SwerveModuleState[] {
                new SwerveModuleState(),
                new SwerveModuleState(),
                new SwerveModuleState(),
                new SwerveModuleState()
        };

        odom_pose = new Pose2d();

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

        if (discretize) chassis_speeds = discretize_chassis_speeds(chassis_speeds);

        module_states = kinematics.toSwerveModuleStates(chassis_speeds);

        // change target wheel directions if the wheel has to rotate more than 90*
        for (int i = 0; i < module_states.length; i++) {
            module_states[i].optimize(modules[i].getAngle());
        }

        // normalize wheel speeds of any are greater than max speed
        SwerveDriveKinematics.desaturateWheelSpeeds(module_states, SwerveConstants.MAX_SPEED);
        
        setModules(module_states);
    }

    /**
     * Sends calculated inputs to swerve modules.
     * @param module_states The desired module states.
     */

    public void setModules(SwerveModuleState[] module_states) {
        for (int i = 0; i < modules.length; i++) {
            modules[i].setState(module_states[i]);
        }
    }

    // Thanks to Team 4738 for modified discretization code
    /**
     * Accounts for drift while simultaneously translating and rotating through discretization.
     * @param speeds The desired chassis speeds.
     * @return The adjusted chassis speeds.
     */

    public ChassisSpeeds discretize_chassis_speeds(ChassisSpeeds speeds) {
        double dt = RobotConstants.LOOP_TIME_SECONDS;
        // Makes a Pose2d for the target delta over one time loop
        var desired_delta_pose = new Pose2d(
                speeds.vxMetersPerSecond * dt,
                speeds.vyMetersPerSecond * dt,
                new Rotation2d(speeds.omegaRadiansPerSecond * dt * 1) // tunable
        );
        // Makes a Twist2d object that maps new pose to delta pose
        var twist = new Pose2d().log(desired_delta_pose);

        return new ChassisSpeeds((twist.dx / dt), (twist.dy / dt), (speeds.omegaRadiansPerSecond));
    }

    /**
     * Returns the measured swerve module positions for odometry.
     * @return The measured swerve module positions.
     */

    public SwerveModulePosition[] getModulePositions() {
        var positions = new SwerveModulePosition[modules.length];
        for (int i = 0; i < modules.length; i++) {
            positions[i] = modules[i].getModulePosition();
        }
        return positions;
    }

    /**
     * Returns the measured swerve module states for odometry and telemetry.
     * @return The measured swerve module states.
     */
    @AutoLogOutput(key = "Custom/hi2")
    public SwerveModuleState[] getModuleStates() {
        var states = new SwerveModuleState[modules.length];
        for (int i = 0; i < modules.length; i++) {
            states[i] = modules[i].getModuleState();
        }
        return states;
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

    /**
     * One-time method to instantiate NT publishers for AdvantageScope and Elastic.
     */

    private void setupDashboard() {

        // Instantiate network publishers for advantagescope
        NetworkTable swerve = NetworkTableInstance.getDefault().getTable("swerve");
        real_states_pub = swerve.getStructArrayTopic("States", SwerveModuleState.struct).publish();
        target_states_pub = swerve.getStructArrayTopic("Target States", SwerveModuleState.struct).publish();
        gyro_pub = swerve.getStructTopic("Gyro", Rotation2d.struct).publish();

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
     * Publishes telemetry readings to AdvantageScope.
     */

    private void publishAdv() {
        real_states_pub.set(getModuleStates());
        target_states_pub.set(module_states);
        gyro_pub.set(getGyroAngle());
    }

    /**
     * Gets either the measured yaw from the AHRS or the calculated angle from the
     * simulation.
     * Forward is 0, CCW is positive.
     * @return The robot yaw.
     */

    public abstract Rotation2d getGyroAngle();

    public abstract void resetGyroAngle();

    /**
     * Creates either a SwerveModuleRealIO or SwerveModuleSimIO object.
     * @param drive_port    The port number of the drive motor.
     * @param steer_port    The port number of the steer motor.
     * @param sensor_port The port number of the module's CANcoder.
     * @return The constructed SwerveModule object.
     */

    protected abstract SwerveModule initializeModule(int drive_port, int steer_port, int sensor_port);

    @Override
    public void periodic() {
        for (SwerveModule module : modules) {
            module.update();
        }
        
        publishAdv();

    }
    
}
