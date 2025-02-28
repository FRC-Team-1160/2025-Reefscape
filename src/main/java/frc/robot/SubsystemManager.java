package frc.robot;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import org.json.simple.parser.ParseException;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ConnectedMotorValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.RobotConstants.ComponentZeroPoses;
import frc.robot.Constants.VisionConstants.AlgaeParams;
import frc.robot.RobotUtils.ArticulatedPose;
import frc.robot.SubsystemManager.RobotState.DriveStates;
import frc.robot.SubsystemManager.RobotState.ElevatorStates;
import frc.robot.Subsystems.DriveTrain.DriveTrain;
import frc.robot.Subsystems.DriveTrain.DriveTrainRealIO;
import frc.robot.Subsystems.DriveTrain.DriveTrainSimIO;
import frc.robot.Subsystems.Elevator.Elevator;
import frc.robot.Subsystems.Elevator.ElevatorRealIO;
import frc.robot.Subsystems.Elevator.ElevatorSimIO;
import frc.robot.Subsystems.Elevator.Elevator.TargetState;
import frc.robot.Subsystems.Funnel.Funnel;
import frc.robot.Subsystems.Funnel.FunnelRealIO;
import frc.robot.Subsystems.Vision.Vision;
import frc.robot.Subsystems.Vision.ObjectDetection;
import frc.robot.Subsystems.Vision.VisionTarget;

public class SubsystemManager {

    public class RobotState {
        enum DriveStates {
            DRIVER_CONTROL,
            PID_TRACKING,
            PID_ALIGNING,
            PATHPLANNER_CONTROL
        }

        enum ElevatorStates {
            FULL_CONTROL // Control via setpoint
        }
    
        public DriveStates drive_state = DriveStates.DRIVER_CONTROL;
        public ElevatorStates elevator_state = ElevatorStates.FULL_CONTROL;
    }

    public DriveTrain m_drive;
    public Elevator m_elevator;
    public Vision m_vision;
    public ObjectDetection m_object_detection;
    public Funnel m_funnel;

    public SwervePIDController m_swerve_pid_controller;
    public PathplannerController m_pathplanner_controller;

    public RobotState m_robot_state = new RobotState();
    public Commands commands = new Commands();

    public Supplier<Double> getStickX, getStickY, getStickA, getStickEl;

    public Pose2d robot_pose;

    public SwerveDrivePoseEstimator pose_estimator;
    
    public VisionTarget tracked_target;

    StructPublisher<Pose2d> adv_pose_pub;
    StructArrayPublisher<Pose3d> adv_components_pub;

    public Orchestra orchestra;

    /** Creates a new SubsystemManager. */
    public SubsystemManager(
        Supplier<Double> getStickX, 
        Supplier<Double> getStickY, 
        Supplier<Double> getStickA, 
        Supplier<Double> getStickEl) {

        // Initialize subsystems
        if (Robot.isReal()) {
            // If motors aren't connected, assume mechanism isn't attached/working and run it as simulation
            m_drive = new DriveTrainRealIO();
            for (TalonFX talon : ((DriveTrainRealIO) m_drive).getTalons()) {
                if (talon.getConnectedMotor().getValue() == ConnectedMotorValue.Unknown) {
                    m_drive = new DriveTrainSimIO();
                    break;
                }
            }
            m_elevator = new ElevatorRealIO();
            for (TalonFX talon : ((ElevatorRealIO) m_elevator).getTalons()) {
                if (talon.getConnectedMotor().getValue() == ConnectedMotorValue.Unknown) {
                    m_elevator = new ElevatorSimIO();
                    break;
                }
            }
            m_funnel = new FunnelRealIO();
        } else {
            m_drive = new DriveTrainSimIO();
            m_elevator = new ElevatorSimIO();
        }

        robot_pose = new Pose2d();
        pose_estimator = new SwerveDrivePoseEstimator(
            m_drive.kinematics, 
            m_drive.getGyroAngle(), 
            m_drive.getModulePositions(), 
            robot_pose, 
            VecBuilder.fill(0.02, 0.02, 0.02), 
            VecBuilder.fill(0.05, 0.05, 0.05));


        // Vision needs to be initialized afterwards to have access to pose_estimator
        m_vision = new Vision(pose_estimator, this::getPoseEstimate);

        this.getStickX = getStickX;
        this.getStickY = getStickY;
        this.getStickA = getStickA;
        this.getStickEl = getStickEl;

        m_swerve_pid_controller = new SwervePIDController(
            this::getPoseEstimate, 
            this::getPidReferenceSpeeds, 
            1);

        m_pathplanner_controller = new PathplannerController(this::getPoseEstimate);

        // Configure Autobuilder
        RobotConfig config;
        try {
            config = RobotConfig.fromGUISettings();
        } catch (IOException | ParseException e) {
            e.printStackTrace();
            return;
        }

        AutoBuilder.configure(
            pose_estimator::getEstimatedPosition,
            pose_estimator::resetPose,
            m_drive::getOdomSpeeds,
            // Pathplanner commands are redirected to the PathplannerController instance
            (speeds, feedforwards) -> m_pathplanner_controller.acceptGeneratedSpeeds(speeds),
            new PPHolonomicDriveController(
                new PIDConstants(
                    AutoConstants.translation_kP, 
                    AutoConstants.translation_kI,
                    AutoConstants.translation_kD),
                new PIDConstants(
                    AutoConstants.rotation_kP, 
                    AutoConstants.rotation_kI, 
                    AutoConstants.rotation_kD)),
            config,
            () -> RobotUtils.isRedAlliance(),
            m_drive // Reference to drive subsystem to set requirements; unfortunately required to instantiate
        );

        setupDashboard();

        orchestra = new Orchestra();
        setupOrchestra();

    }

    public void setupDashboard() {
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable adv_swerve = inst.getTable("adv_swerve");
        adv_pose_pub = adv_swerve.getStructTopic("Pose", Pose2d.struct).publish();
        adv_components_pub = adv_swerve.getStructArrayTopic("Component Poses", Pose3d.struct).publish();
    }

    /**
     * Returns the estimated pose. Updates PoseEstimator with odometry readings for most accurate estimate.
     * @return The updated pose estimate.
     */
    public Pose2d getPoseEstimate() {
        SmartDashboard.putNumber("estimate x", pose_estimator.getEstimatedPosition().getX());
        return pose_estimator.updateWithTime(
            Timer.getTimestamp(), 
            m_drive.getGyroAngle(), 
            m_drive.getModulePositions());
    }

    /**
     * Returns the current robot speeds for reference for pid control.
     * @return The current robot-relative chassis speeds.
     */
    public ChassisSpeeds getPidReferenceSpeeds() {
        // If starting a new pid, use measured speeds for smooth transition; else, use target speeds for accurate acceleration
        if (m_swerve_pid_controller.reset_speeds) {
            m_swerve_pid_controller.reset_speeds = false;
            return m_drive.getOdomSpeeds();
        } else {
            return m_drive.getTargetOdomSpeeds();
        }
    }

    public void update() {

        m_vision.update();
        // Get odometry readings BEFORE running 
        robot_pose = getPoseEstimate();

        switch (m_robot_state.drive_state) {

            case PATHPLANNER_CONTROL:
                m_drive.setSwerveDrive(m_pathplanner_controller.generated_speeds);
                break;

            case PID_ALIGNING:
                m_drive.setSwerveDrive(m_swerve_pid_controller.calculate(true));
                break;
            case PID_TRACKING:
                if (tracked_target != null && tracked_target.timeout < AlgaeParams.DETECTION_LIMIT) {
                    m_swerve_pid_controller.target_pose = tracked_target.getPose();

                    if (tracked_target.getDistance(robot_pose) < 1.5)
                        m_vision.setCameraPipelines(Vision.CameraMode.kStereoAlgae);

                    m_drive.setSwerveDrive(
                        m_swerve_pid_controller.calculate(), 
                        false);
                    break;
                } else {
                    tracked_target = null;
                    // No break; control switches back to the driver if no valid target is present
                }

            default:
                double stick_x = MathUtil.applyDeadband(-getStickX.get(), 0.1, 1)
                     * SwerveConstants.DRIVE_SPEED;
                double stick_y = MathUtil.applyDeadband(-getStickY.get(), 0.1, 1)
                     * SwerveConstants.DRIVE_SPEED;
                double stick_a = MathUtil.applyDeadband(-getStickA.get(), 0.1, 1)
                     * SwerveConstants.TURN_SPEED;

                double stick_el = getStickEl.get();

                double stick_speed = RobotUtils.hypot(stick_x, stick_y);
                // Renormalize movement if combined vector is overspeed
                stick_x *= Math.min(SwerveConstants.DRIVE_SPEED / stick_speed, 1);
                stick_y *= Math.min(SwerveConstants.DRIVE_SPEED / stick_speed, 1);

                m_drive.setSwerveDrive(stick_x, stick_y, stick_a);
        }
        publishAdv();
    }

    public void publishAdv() {
        new ArticulatedPose(getPoseEstimate(), m_elevator.getElevatorHeight(), m_elevator.getWristAngle().getRadians())
            .publish(adv_pose_pub, adv_components_pub);
    }

    public void setupOrchestra() {
        setupOrchestra(Integer.MAX_VALUE);
    }

    public void setupOrchestra(int tracks) {

        if (Robot.isSimulation()) return; // Abort if robot is in simulation

        orchestra.clearInstruments();

        List<TalonFX> instruments = new ArrayList<TalonFX>();

        try {
            instruments.addAll(((DriveTrainRealIO) m_drive).getTalons());
        } catch (Exception e) {
            e.printStackTrace();
        }
        try {
            instruments.addAll(((ElevatorRealIO) m_elevator).getTalons());
        } catch (Exception e) {
            e.printStackTrace();
        }

        // Increment track number by 1 when adding; reset when max tracks reached
        for (int i = 0; i < instruments.size(); i++) { 
            orchestra.addInstrument(instruments.get(i), i % tracks);
        }
    }
    
    public class Commands {

        public Command alignReef() {
            return getAlignCommand(
                m_swerve_pid_controller::getNearestReefPose,
                0.2, 
                TargetState.kL4);
        }

        public Command alignSource() {
            return getAlignCommand(
                m_swerve_pid_controller::getNearestSourcePose, 
                0.2, 
                Rotation2d.kPi,
                TargetState.kSource);
        }

        public Command alignProcessor() {
            return getAlignCommand(
                m_swerve_pid_controller::getProcessorPose, 
                0.5,
                TargetState.kProcessor);
        }

        public Command getAlignCommand(Supplier<Pose2d> target_pose, double target_distance, TargetState elevator_state) {
            return getAlignCommand(target_pose, target_distance, Rotation2d.kZero, elevator_state);
        }

        public Command getAlignCommand(
            Supplier<Pose2d> target_pose, 
            double target_distance, 
            Rotation2d offset, 
            TargetState elevator_state
        ) {
            return new FunctionalCommand(
                () -> {
                    m_robot_state.drive_state = DriveStates.PID_ALIGNING;
                    m_swerve_pid_controller.reset_speeds = true;
                    m_swerve_pid_controller.configure(
                        target_pose.get(), 
                        target_distance,
                        Rotation2d.kZero);
                    m_elevator.setState(elevator_state);
                    m_vision.setCameraPipelines(Vision.CameraMode.kStereoAprilTag);
                },
                () -> {},
                canceled -> {
                    if (canceled) {
                        m_robot_state.drive_state = RobotState.DriveStates.DRIVER_CONTROL;
                        m_vision.setCameraPipelines(Vision.CameraMode.kDefault);
                    }
                },
                () -> m_robot_state.drive_state != RobotState.DriveStates.PID_ALIGNING 
                    || m_elevator.m_current_state != elevator_state
            );
        }

        public Command trackAlgae() {
            return new FunctionalCommand(
                () -> {
                    m_robot_state.drive_state = RobotState.DriveStates.PID_TRACKING;
                    m_vision.m_object_detection.getClosestTarget().ifPresent(
                        target -> tracked_target = target);
                    // Reset pid speeds to real measured for acceleration calculations
                    m_swerve_pid_controller.reset_speeds = true;
                    m_swerve_pid_controller.configure(null, 0.8, Rotation2d.kZero);
                    m_elevator.setState(TargetState.kIntake);
                    m_vision.setCameraPipelines(Vision.CameraMode.kStereoAlgae);
                },
                () -> {},
                canceled -> {
                    m_robot_state.drive_state = RobotState.DriveStates.DRIVER_CONTROL;
                    tracked_target = null;
                    m_vision.setCameraPipelines(Vision.CameraMode.kDefault);
                }, 
                () -> m_robot_state.drive_state != RobotState.DriveStates.PID_TRACKING || tracked_target == null
            );
        }

        public Command pathCmdWrapper(Command path_cmd) {
            return pathCmdWrapper(() -> path_cmd);
        }

        public Command pathCmdWrapper(Supplier<Command> cmd_supplier) {
            return new FunctionalCommand(
                () -> {
                    m_robot_state.drive_state = RobotState.DriveStates.PATHPLANNER_CONTROL;
                    m_pathplanner_controller.current_command = cmd_supplier.get();
                    m_pathplanner_controller.cmdInitialize();
                },
                m_pathplanner_controller::cmdExecute, 
                (canceled) -> {
                    m_robot_state.drive_state = RobotState.DriveStates.DRIVER_CONTROL;
                    m_pathplanner_controller.cmdEnd(canceled);
                },
                () -> m_pathplanner_controller.cmdIsFinished() || 
                    m_robot_state.drive_state != RobotState.DriveStates.PATHPLANNER_CONTROL);
        }

        public Command playMusic(String filename) {
            return playMusic(filename, Integer.MAX_VALUE);
        }

        public Command playMusic(String filename, int tracks) {
            return new FunctionalCommand(
                () -> {
                    setupOrchestra(tracks);
                    orchestra.loadMusic("Music/" + filename + ".chrp");
                    orchestra.play();
                },
                () -> {},
                canceled -> orchestra.stop(),
                () -> !orchestra.isPlaying());
        }

    }
}