package frc.robot;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.VisionConstants.AlgaeParams;
import frc.robot.RobotContainer.JoystickInputs;
import frc.robot.RobotUtils.ArticulatedPose;
import frc.robot.SubsystemManager.RobotState.DriveStates;
import frc.robot.Subsystems.Climber.Climber;
import frc.robot.Subsystems.DriveTrain.DriveTrain;
import frc.robot.Subsystems.Elevator.Elevator;
import frc.robot.Subsystems.Vision.Vision;
import frc.robot.Subsystems.Vision.ObjectDetection;
import frc.robot.Subsystems.Vision.VisionTarget;

/** 
 * The SubsystemManager coordinates all of the classes and ensures only one sequence is run at at time.
 */
public class SubsystemManager {

    public static final SubsystemManager instance = new SubsystemManager();

    public class RobotState {
        public enum DriveStates {
            DRIVER_CONTROL,
            PID_TRACKING,
            PID_ALIGNING,
            PATHPLANNER_CONTROL
        }
        public DriveStates drive_state = DriveStates.DRIVER_CONTROL;
    }

    public RobotState m_robot_state = new RobotState();
    public final RobotCommands commands = new RobotCommands();

    public SwerveDrivePoseEstimator pose_estimator;
    
    public VisionTarget tracked_target;

    public Orchestra orchestra;

    /** Stores a set of chassis speeds and acceleration feedforwards to accept PathPlanner output. */
    public record PathplannerSpeeds(ChassisSpeeds chassis_speeds, double[] acceleration_feedforwards) {
        public static PathplannerSpeeds kZero = 
            new PathplannerSpeeds(new ChassisSpeeds(0, 0, 0), new double[4]);
    };

    public PathplannerSpeeds m_pathplanner_speeds;

    /** Creates a new SubsystemManager. */
    private SubsystemManager() {

        pose_estimator = new SwerveDrivePoseEstimator(
            DriveTrain.instance.kinematics, 
            DriveTrain.instance.getGyroAngle(), 
            DriveTrain.instance.getModulePositions(), 
            Pose2d.kZero, 
            VecBuilder.fill(0.01, 0.01, 0.01), 
            VecBuilder.fill(0.05, 0.05, 0.05));

        m_pathplanner_speeds = PathplannerSpeeds.kZero;

        orchestra = new Orchestra();
        setupOrchestra();

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

    @AutoLogOutput
    public Pose3d[] getComponentPoses() {
        return new ArticulatedPose(
            getPoseEstimate(), 
            Elevator.instance.getElevatorHeight(), 
            0).component_poses();
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
            return DriveTrain.instance.getTargetOdomSpeeds().plus(DriveTrain.instance.getOdomSpeeds()).div(2);
        }
    }

    public void update(JoystickInputs stick_inputs) {

        Vision.instance.update();

        switch (m_robot_state.drive_state) {

            case PATHPLANNER_CONTROL:
                DriveTrain.instance.setSwerveDrive(m_pathplanner_speeds.chassis_speeds, false);
                DriveTrain.instance.setAccelerationFeedforwards(m_pathplanner_speeds.acceleration_feedforwards);
                Logger.recordOutput("DriveTrain/FFs", m_pathplanner_speeds.acceleration_feedforwards);
                break;

            case PID_ALIGNING:
                DriveTrain.instance.setSwerveDrive(
                    DriveTrain.instance.discretize_chassis_speeds(
                        SwervePIDController.instance.calculate(true), 8));
                break;

            case PID_TRACKING:
                if (tracked_target != null && tracked_target.timeout < AlgaeParams.DETECTION_LIMIT) {
                    SwervePIDController.instance.target_pose = tracked_target.getPose();

                    if (tracked_target.getDistance(getPoseEstimate()) < 1.5)
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
                double stick_x = MathUtil.applyDeadband(-stick_inputs.drive_x(), 0.2, 1)
                     * SwerveConstants.DRIVE_SPEED;
                double stick_y = MathUtil.applyDeadband(-stick_inputs.drive_y(), 0.2, 1)
                     * SwerveConstants.DRIVE_SPEED;
                double stick_a = MathUtil.applyDeadband(-stick_inputs.drive_a(), 0.2, 1)
                     * SwerveConstants.TURN_SPEED;

                double stick_speed = RobotUtils.hypot(stick_x, stick_y);
                // Renormalize movement if combined vector is overspeed
                stick_x *= Math.min(SwerveConstants.DRIVE_SPEED / stick_speed, 1);
                stick_y *= Math.min(SwerveConstants.DRIVE_SPEED / stick_speed, 1);

                DriveTrain.instance.setSwerveDrive(stick_x, stick_y, stick_a);
        }

        Logger.recordOutput("SubsystemManager/DriveState", m_robot_state.drive_state.toString());
    }

    /**
     * Clears and refills the orchestra to play MIDI sequences. Each track is assigned one motor.
     */
    public void setupOrchestra() {
        setupOrchestra(Integer.MAX_VALUE);
    }

    /**
     * Clears and refills the orchestra to play MIDI sequences.
     * @param tracks The number of tracks the midi file contains.
     */
    public void setupOrchestra(int tracks) {

        if (Robot.isSimulation()) return; // Abort if robot is in simulation

        orchestra.clearInstruments();

        List<TalonFX> instruments = new ArrayList<TalonFX>();

        instruments.addAll(DriveTrain.instance.getTalons());
        instruments.addAll(Elevator.instance.getTalons());
        instruments.addAll(Climber.instance.getTalons());

        // Increment track number by 1 when adding; reset when max tracks reached
        for (int i = 0; i < instruments.size(); i++) { 
            orchestra.addInstrument(instruments.get(i), i % tracks);
        }
    }
    
    /**
     * A class separating all of the command getters.
     */
    public class RobotCommands {

        /**
         * Selects an alignment or tracking command based on the current elevator state.
         * @return A command to align with a game element or piece.
         */
        public Command selectCommand() {
            return new DeferredCommand(
                () -> Elevator.instance.m_current_state.target_position.command_supplier.get(), new HashSet<>());
        }

        public Command alignReef(Integer index, double distance) {
            return getAlignCommand(
                () -> index == null ? SwervePIDController.instance.getNearestReefPose()
                     : SwervePIDController.instance.getReefPose(index), 
                distance
            ).withName("Align Reef");
        }

        public Command alignReef(Integer index) { return alignReef(index, 0.05); }

        public Command alignReef() { return alignReef(null); }

        public Command alignReefClose(Integer index) { return alignReef(index, 0.02); }

        public Command alignReefClose() { return alignReefClose(null); }

        public Command alignReefAlgae() {
            return getAlignCommand(
                () -> SwervePIDController.instance.getNearestReefPose(0), 0.15, null);
        }

        public Command alignSource() {
            return getAlignCommand( 
                SwervePIDController.instance::getNearestSourcePose, 
                0.05,
                Rotation2d.kPi
            ).withName("Align Source");
        }

        public Command getAlignCommand(Supplier<Pose2d> target_pose, double target_distance) {
            return getAlignCommand(target_pose, target_distance, Rotation2d.kZero);
        }

        public Command getAlignCommand(
            Supplier<Pose2d> target_pose, 
            double target_distance, 
            Rotation2d offset
        ) {
            return new FunctionalCommand(() -> {
                    m_robot_state.drive_state = DriveStates.PID_ALIGNING;
                    SwervePIDController.instance.reset_speeds = true;
                    SwervePIDController.instance.done = false;
                    SwervePIDController.instance.configure(target_pose.get(), target_distance, offset);
                    // Vision.instance.setCameraPipelines(Vision.CameraMode.kStereoAprilTag);
                },
                () -> {},
                canceled -> {
                    m_robot_state.drive_state = RobotState.DriveStates.DRIVER_CONTROL;
                },
                () -> SwervePIDController.instance.done
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

        public Command decoratePathplannerCmd(Command cmd) {
            return cmd.beforeStarting(
                () -> m_robot_state.drive_state = RobotState.DriveStates.PATHPLANNER_CONTROL);
        }

        public Command decorateAutoCmd(Supplier<Command> cmd) {
            return Commands.deferredProxy(cmd)
                .beforeStarting(() -> {
                    m_robot_state.drive_state = RobotState.DriveStates.PATHPLANNER_CONTROL;
                })
                .andThen(() -> {
                    m_robot_state.drive_state = RobotState.DriveStates.DRIVER_CONTROL;
                    m_pathplanner_speeds = PathplannerSpeeds.kZero;
                    DriveTrain.instance.setGyroAngle(-SubsystemManager.instance.getPoseEstimate().getRotation().getDegrees());
                });
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