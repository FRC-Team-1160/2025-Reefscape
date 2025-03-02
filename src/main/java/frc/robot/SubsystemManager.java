package frc.robot;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.AutoLogOutput;

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
import frc.robot.RobotContainer.JoystickInputs;
import frc.robot.RobotUtils.ArticulatedPose;
import frc.robot.SubsystemManager.RobotState.DriveStates;
import frc.robot.SubsystemManager.RobotState.ElevatorStates;
import frc.robot.Subsystems.Climber.Climber;
import frc.robot.Subsystems.DriveTrain.DriveTrain;
import frc.robot.Subsystems.DriveTrain.DriveTrainRealIO;
import frc.robot.Subsystems.DriveTrain.DriveTrainSimIO;
import frc.robot.Subsystems.Elevator.Elevator;
import frc.robot.Subsystems.Elevator.ElevatorRealIO;
import frc.robot.Subsystems.Elevator.Elevator.TargetState;
import frc.robot.Subsystems.Funnel.Funnel;
import frc.robot.Subsystems.Funnel.FunnelRealIO;
import frc.robot.Subsystems.Vision.Vision;
import frc.robot.Subsystems.Vision.ObjectDetection;
import frc.robot.Subsystems.Vision.VisionTarget;

public class SubsystemManager {

    public static final SubsystemManager instance = new SubsystemManager();

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

    public RobotState m_robot_state = new RobotState();
    public Commands commands = new Commands();

    public Pose2d robot_pose;

    public SwerveDrivePoseEstimator pose_estimator;
    
    public VisionTarget tracked_target;

    StructPublisher<Pose2d> adv_pose_pub;
    StructArrayPublisher<Pose3d> adv_components_pub;

    public Orchestra orchestra;

    /** Creates a new SubsystemManager. */
    private SubsystemManager() {

        robot_pose = new Pose2d();
        pose_estimator = new SwerveDrivePoseEstimator(
            DriveTrain.instance.kinematics, 
            DriveTrain.instance.getGyroAngle(), 
            DriveTrain.instance.getModulePositions(), 
            robot_pose, 
            VecBuilder.fill(0.02, 0.02, 0.02), 
            VecBuilder.fill(0.05, 0.05, 0.05));

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
    @AutoLogOutput
    public Pose2d getPoseEstimate() {
        return pose_estimator.updateWithTime(
            Timer.getTimestamp(), 
            DriveTrain.instance.getGyroAngle(), 
            DriveTrain.instance.getModulePositions());
    }

    /**
     * Returns the current robot speeds for reference for pid control.
     * @return The current robot-relative chassis speeds.
     */
    public ChassisSpeeds getPidReferenceSpeeds() {
        // If starting a new pid, use measured speeds for smooth transition; else, use target speeds for accurate acceleration
        if (SwervePIDController.instance.reset_speeds) {
            SwervePIDController.instance.reset_speeds = false;
            return DriveTrain.instance.getOdomSpeeds();
        } else {
            return DriveTrain.instance.getTargetOdomSpeeds();
        }
    }

    public void update(JoystickInputs stick_inputs) {

        Vision.instance.update();
        // Get odometry readings BEFORE running 
        robot_pose = getPoseEstimate();

        switch (m_robot_state.drive_state) {

            case PATHPLANNER_CONTROL:
                DriveTrain.instance.setSwerveDrive(PathplannerController.instance.generated_speeds);
                break;

            case PID_ALIGNING:
                DriveTrain.instance.setSwerveDrive(SwervePIDController.instance.calculate(true));
                break;
            case PID_TRACKING:
                if (tracked_target != null && tracked_target.timeout < AlgaeParams.DETECTION_LIMIT) {
                    SwervePIDController.instance.target_pose = tracked_target.getPose();

                    if (tracked_target.getDistance(robot_pose) < 1.5)
                        Vision.instance.setCameraPipelines(Vision.CameraMode.kStereoAlgae);

                    DriveTrain.instance.setSwerveDrive(
                        SwervePIDController.instance.calculate(), 
                        false);
                    break;
                } else {
                    tracked_target = null;
                    // No break; control switches back to the driver if no valid target is present
                }

            default:
                double stick_x = MathUtil.applyDeadband(-stick_inputs.drive_x(), 0.1, 1)
                     * SwerveConstants.DRIVE_SPEED;
                double stick_y = MathUtil.applyDeadband(-stick_inputs.drive_y(), 0.1, 1)
                     * SwerveConstants.DRIVE_SPEED;
                double stick_a = MathUtil.applyDeadband(-stick_inputs.drive_a(), 0.1, 1)
                     * SwerveConstants.TURN_SPEED;

                double stick_el = stick_inputs.elevator();

                double stick_speed = RobotUtils.hypot(stick_x, stick_y);
                // Renormalize movement if combined vector is overspeed
                stick_x *= Math.min(SwerveConstants.DRIVE_SPEED / stick_speed, 1);
                stick_y *= Math.min(SwerveConstants.DRIVE_SPEED / stick_speed, 1);

                DriveTrain.instance.setSwerveDrive(stick_x, stick_y, stick_a);
        }
        publishAdv();
    }

    public void publishAdv() {
        new ArticulatedPose(getPoseEstimate(), 
            Elevator.instance.getElevatorHeight(), 
            Elevator.instance.getWristAngle().getRadians()
        ).publish(adv_pose_pub, adv_components_pub);
    }

    public void setupOrchestra() {
        setupOrchestra(Integer.MAX_VALUE);
    }

    public void setupOrchestra(int tracks) {

        if (Robot.isSimulation()) return; // Abort if robot is in simulation

        orchestra.clearInstruments();

        List<TalonFX> instruments = new ArrayList<TalonFX>();

        try {
            instruments.addAll(((DriveTrainRealIO) DriveTrain.instance).getTalons());
        } catch (Exception e) {
            e.printStackTrace();
        }
        try {
            instruments.addAll(((ElevatorRealIO) Elevator.instance).getTalons());
        } catch (Exception e) {
            e.printStackTrace();
        }

        // Increment track number by 1 when adding; reset when max tracks reached
        for (int i = 0; i < instruments.size(); i++) { 
            orchestra.addInstrument(instruments.get(i), i % tracks);
        }
    }
    
    public class Commands {

        public Command alignReef(boolean with_elevator) {
            return getAlignCommand(
                SwervePIDController.instance::getNearestReefPose,
                0.2, 
                with_elevator ? TargetState.kL4 : null);
        }

        public Command alignSource(boolean with_elevator) {
            return getAlignCommand(
                SwervePIDController.instance::getNearestSourcePose, 
                0.2, 
                Rotation2d.kPi,
                with_elevator ? TargetState.kSource : null);
        }

        public Command alignProcessor(boolean with_elevator) {
            return getAlignCommand(
                SwervePIDController.instance::getProcessorPose, 
                0.5,
                with_elevator ? TargetState.kProcessor : null);
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
                    SwervePIDController.instance.reset_speeds = true;
                    SwervePIDController.instance.configure(
                        target_pose.get(), 
                        target_distance,
                        offset);
                    Elevator.instance.setState(elevator_state);
                    Vision.instance.setCameraPipelines(Vision.CameraMode.kStereoAprilTag);
                },
                () -> {},
                canceled -> {
                    if (canceled) {
                        m_robot_state.drive_state = RobotState.DriveStates.DRIVER_CONTROL;
                        Vision.instance.setCameraPipelines(Vision.CameraMode.kDefault);
                    }
                },
                () -> m_robot_state.drive_state != RobotState.DriveStates.PID_ALIGNING 
                || Elevator.instance.m_current_state != elevator_state
            );
        }

        public Command trackAlgae() {
            return new FunctionalCommand(
                () -> {
                    m_robot_state.drive_state = RobotState.DriveStates.PID_TRACKING;
                    ObjectDetection.instance.getClosestTarget().ifPresent(
                        target -> tracked_target = target);
                    // Reset pid speeds to real measured for acceleration calculations
                    SwervePIDController.instance.reset_speeds = true;
                    SwervePIDController.instance.configure(null, 0.8, Rotation2d.kZero);
                    Elevator.instance.setState(TargetState.kIntakePrepare);
                    Vision.instance.setCameraPipelines(Vision.CameraMode.kStereoAlgae);
                },
                () -> {},
                canceled -> {
                    m_robot_state.drive_state = RobotState.DriveStates.DRIVER_CONTROL;
                    tracked_target = null;
                    Vision.instance.setCameraPipelines(Vision.CameraMode.kDefault);
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
                    PathplannerController.instance.current_command = cmd_supplier.get();
                    PathplannerController.instance.cmdInitialize();
                },
                PathplannerController.instance::cmdExecute, 
                (canceled) -> {
                    m_robot_state.drive_state = RobotState.DriveStates.DRIVER_CONTROL;
                    PathplannerController.instance.cmdEnd(canceled);
                },
                () -> PathplannerController.instance.cmdIsFinished() || 
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