// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.DriveTrain; //Accidentally changed the folder name to be uppercase this year, oh well :P

import java.io.IOException;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public abstract class DriveTrain extends SubsystemBase {

  /** The drive train's SwerveModule objects. */
  public SwerveModule[] m_modules;
  /** @hidden */
  public SwerveDriveKinematics m_kinematics;
  /** The desired module states. */
  public SwerveModuleState[] m_module_states;

  /** Module positions for SwerveDrivePoseEstimator. */
  public SwerveModulePosition[] m_module_positions;
  /** Pose estimator. */
  public SwerveDrivePoseEstimator m_pose_estimator;
  /** Odometry-based 2d pose. */
  public Pose2d m_odom_pose;

  /** State publisher for AdvantageScope. */
  protected StructArrayPublisher<SwerveModuleState> adv_real_states_pub, adv_target_states_pub;
  /** State publisher for AdvantageScope. */
  protected StructPublisher<Rotation2d> adv_gyro_pub;

  public DriveTrain() {
    m_kinematics = new SwerveDriveKinematics(
        new Translation2d(Constants.Swerve.OFFSET, Constants.Swerve.OFFSET), // front left
        new Translation2d(Constants.Swerve.OFFSET, -Constants.Swerve.OFFSET), // front right
        new Translation2d(-Constants.Swerve.OFFSET, Constants.Swerve.OFFSET), // back left
        new Translation2d(-Constants.Swerve.OFFSET, -Constants.Swerve.OFFSET) // back right
    );

    m_modules = new SwerveModule[4];
    m_modules[0] = initializeModule(Constants.Port.FRONT_LEFT_DRIVE_MOTOR, Constants.Port.FRONT_LEFT_STEER_MOTOR,
        Constants.Port.FRONT_LEFT_CODER);
    m_modules[1] = initializeModule(Constants.Port.FRONT_RIGHT_DRIVE_MOTOR, Constants.Port.FRONT_RIGHT_STEER_MOTOR,
        Constants.Port.FRONT_RIGHT_CODER);
    m_modules[2] = initializeModule(Constants.Port.BACK_LEFT_DRIVE_MOTOR, Constants.Port.BACK_LEFT_STEER_MOTOR,
        Constants.Port.BACK_LEFT_CODER);
    m_modules[3] = initializeModule(Constants.Port.BACK_RIGHT_DRIVE_MOTOR, Constants.Port.BACK_RIGHT_STEER_MOTOR,
        Constants.Port.BACK_RIGHT_CODER);

    m_module_states = new SwerveModuleState[] {
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState()
    };

    m_module_positions = new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
    };

    m_odom_pose = new Pose2d();
    m_pose_estimator = new SwerveDrivePoseEstimator(m_kinematics, getGyroAngle(), m_module_positions, m_odom_pose);

    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings();
    } catch (IOException | ParseException e) {
      e.printStackTrace();
      return;
    }
    AutoBuilder.configure(
        () -> m_odom_pose,
        m_pose_estimator::resetPose,
        () -> m_kinematics.toChassisSpeeds(m_module_states),
        (speeds, feedforwards) -> setSwerveDrive(speeds),
        new PPHolonomicDriveController(
            new PIDConstants(Constants.Auto.translation_kP, Constants.Auto.translation_kI,
                Constants.Auto.translation_kD),
            new PIDConstants(Constants.Auto.rotation_kP, Constants.Auto.rotation_kI, Constants.Auto.rotation_kD)),
        config,
        () -> {
          var alliance = DriverStation.getAlliance();
          if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
          }
          return false;
        },
        this // Reference to this subsystem to set requirements
    );

    setupDashboard();
  }

  /**
   * Creates either a SwerveModuleRealIO or SwerveModuleSimIO object.
   * 
   * @param drive_port  port number of the drive motor
   * @param steer_port  port number of the steer motor
   * @param sensor_port port number of the module's CANcoder
   * @return The constructed SwerveModule object
   */

  protected abstract SwerveModule initializeModule(int drive_port, int steer_port, int sensor_port);

  /**
   * Calculates and sends inputs to swerve modules given field-relative speeds.
   * Calls setSwerveDrive(ChassisSpeeds chassis_speeds)
   * 
   * @param x_metersPerSecond  X-axis speed in m/s. Forward is positive.
   * @param y_metersPerSecond  Y-axis speed in m/s. Right is positive.
   * @param a_radiansPerSecond Angular speed in rad/s. CCW is positive.
   */

  public void setSwerveDrive(double x_metersPerSecond, double y_metersPerSecond, double a_radiansPerSecond) {
    // converts speeds from field's frame of reference to robot's frame of reference
    ChassisSpeeds chassis_speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
        x_metersPerSecond,
        y_metersPerSecond,
        a_radiansPerSecond,
        getGyroAngle());
    setSwerveDrive(chassis_speeds);
  }

  /**
   * Calculates and sends inputs to swerve modules given robot-relative speeds.
   * 
   * @param chassis_speeds The desired robot-relative chassis speeds.
   */

  public void setSwerveDrive(ChassisSpeeds chassis_speeds) {
    // fix weird change over time shenanigans

    SmartDashboard.putNumber("in_x", chassis_speeds.vxMetersPerSecond);
    SmartDashboard.putNumber("in_y", chassis_speeds.vyMetersPerSecond);

    chassis_speeds.omegaRadiansPerSecond *= -1;

    chassis_speeds = discretize_chassis_speeds(chassis_speeds);

    SmartDashboard.putNumber("in_a", chassis_speeds.omegaRadiansPerSecond);

    m_module_states = m_kinematics.toSwerveModuleStates(chassis_speeds);

    // change target wheel directions if the wheel has to rotate more than 90*
    for (int i = 0; i < m_module_states.length; i++) {
      m_module_states[i].optimize(m_modules[i].getAngle());
    }

    // normalize wheel speeds of any are greater than max speed
    SwerveDriveKinematics.desaturateWheelSpeeds(m_module_states, Constants.Swerve.MAX_SPEED);
    
    setModules(m_module_states);

    for (int i = 0; i < m_modules.length; i++) {
      m_module_positions[i] = m_modules[i].getModulePosition();
    }
  }

  /**
   * Sends calculated inputs to swerve modules.
   * 
   * @param module_states The desired module states.
   */

  public void setModules(SwerveModuleState[] module_states) {
    for (int i = 0; i < m_modules.length; i++) {
      m_modules[i].setState(module_states[i]);
    }
  }

  // Thanks to Team 4738 for modified discretization code

  /**
   * Accounts for drift while simultaneously translating and rotating by
   * discretizing.
   * 
   * @param speeds Desired chassis speeds.
   * @return Adjusted chassis speeds.
   */

  public ChassisSpeeds discretize_chassis_speeds(ChassisSpeeds speeds) {
    double dt = Constants.Robot.LOOP_TIME_SECONDS;
    // makes a Pose2d for the target delta over one time loop
    var desiredDeltaPose = new Pose2d(
        speeds.vxMetersPerSecond * dt,
        speeds.vyMetersPerSecond * dt,
        new Rotation2d(speeds.omegaRadiansPerSecond * dt * 1) // tunable
    );
    // makes a Twist2d object that maps new pose to delta pose
    var twist = new Pose2d().log(desiredDeltaPose);

    return new ChassisSpeeds((twist.dx / dt), (twist.dy / dt), (speeds.omegaRadiansPerSecond));
  }

  /**
   * Returns the measured swerve module positions for odometry.
   * 
   * @return The measured swerve module positions.
   */

  public SwerveModulePosition[] getModulePositions() {
    var positions = new SwerveModulePosition[m_modules.length];
    for (int i = 0; i < m_modules.length; i++) {
      positions[i] = m_modules[i].getModulePosition();
    }
    return positions;
  }

  /**
   * Returns the measured swerve module states for telemetry.
   * 
   * @return The measured swerve module states.
   */

  public SwerveModuleState[] getModuleStates() {
    var states = new SwerveModuleState[m_modules.length];
    for (int i = 0; i < m_modules.length; i++) {
      states[i] = m_modules[i].getModuleState();
    }
    return states;
  }

  /**
   * Gets either the measured yaw from the AHRS or the calculated angle from the
   * simulation.
   * Forward is 0, CCW is positive.
   * 
   * @return The robot yaw.
   */

  public abstract Rotation2d getGyroAngle();

  public abstract void resetGyroAngle();

  /**
   * One-time method to instantiate NT publishers for AdvantageScope and Elastic.
   */

  private void setupDashboard() {

    // instantiate network publishers for advantagescope
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable adv_swerve = inst.getTable("adv_swerve");
    adv_real_states_pub = adv_swerve.getStructArrayTopic("States", SwerveModuleState.struct).publish();
    adv_target_states_pub = adv_swerve.getStructArrayTopic("Target States", SwerveModuleState.struct).publish();
    adv_gyro_pub = adv_swerve.getStructTopic("Gyro", Rotation2d.struct).publish();

    // create swerve drive publishers for elastic dashboard (one time setup,
    // auto-call lambdas)
    SmartDashboard.putData("Swerve Target States", new Sendable() {
      @Override
      public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("SwerveDrive");

        builder.addDoubleProperty("Front Left Angle", () -> m_module_states[0].angle.getDegrees(), null);
        builder.addDoubleProperty("Front Left Velocity", () -> m_module_states[0].speedMetersPerSecond, null);

        builder.addDoubleProperty("Front Right Angle", () -> m_module_states[1].angle.getDegrees(), null);
        builder.addDoubleProperty("Front Right Velocity", () -> m_module_states[1].speedMetersPerSecond, null);

        builder.addDoubleProperty("Back Left Angle", () -> m_module_states[2].angle.getDegrees(), null);
        builder.addDoubleProperty("Back Left Velocity", () -> m_module_states[2].speedMetersPerSecond, null);

        builder.addDoubleProperty("Back Right Angle", () -> m_module_states[3].angle.getDegrees(), null);
        builder.addDoubleProperty("Back Right Velocity", () -> m_module_states[3].speedMetersPerSecond, null);

        builder.addDoubleProperty("Robot Angle", () -> getGyroAngle().getDegrees(), null);
      }
    });
    // same here
    SmartDashboard.putData("Swerve Real States", new Sendable() {
      @Override
      public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("SwerveDrive");

        builder.addDoubleProperty("Front Left Angle", () -> m_modules[0].getAngle().getDegrees(), null);
        builder.addDoubleProperty("Front Left Velocity", () -> m_modules[0].getSpeed(), null);

        builder.addDoubleProperty("Front Right Angle", () -> m_modules[1].getAngle().getDegrees(), null);
        builder.addDoubleProperty("Front Right Velocity", () -> m_modules[1].getSpeed(), null);

        builder.addDoubleProperty("Back Left Angle", () -> m_modules[2].getAngle().getDegrees(), null);
        builder.addDoubleProperty("Back Left Velocity", () -> m_modules[2].getSpeed(), null);

        builder.addDoubleProperty("Back Right Angle", () -> m_modules[3].getAngle().getDegrees(), null);
        builder.addDoubleProperty("Back Right Velocity", () -> m_modules[3].getSpeed(), null);

        builder.addDoubleProperty("Robot Angle", () -> getGyroAngle().getDegrees(), null);
      }
    });

  }

  /**
   * Publishes telemetry readings to AdvantageScope.
   */

  private void publishAdv() {
    adv_real_states_pub.set(getModuleStates());
    adv_target_states_pub.set(m_module_states);
    adv_gyro_pub.set(getGyroAngle());
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("gyro", getGyroAngle().getDegrees());
    SmartDashboard.putNumber("pose_angle", m_odom_pose.getRotation().getDegrees());
    for (SwerveModule module : m_modules) {
      module.update();
    }

    m_odom_pose = m_pose_estimator.update(getGyroAngle(), getModulePositions());

    publishAdv();

  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
