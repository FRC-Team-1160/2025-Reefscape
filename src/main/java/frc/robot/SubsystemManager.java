package frc.robot;

import java.io.IOException;
import java.util.function.Supplier;

import org.json.simple.parser.ParseException;

import com.ctre.phoenix6.Orchestra;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Subsystems.DriveTrain.DriveTrain;
import frc.robot.Subsystems.DriveTrain.DriveTrainRealIO;
import frc.robot.Subsystems.DriveTrain.DriveTrainSimIO; //Simulation can't identify sim class without this for some reason
import frc.robot.Subsystems.DriveTrain.SwerveModule;
import frc.robot.Subsystems.DriveTrain.SwerveModuleRealIO;
import frc.robot.Subsystems.Elevator.Elevator;
import frc.robot.Subsystems.Elevator.ElevatorRealIO;
import frc.robot.Subsystems.Elevator.ElevatorSimIO;
import frc.robot.Subsystems.Vision.Vision;
import frc.robot.Subsystems.Vision.ObjectDetection;
import frc.robot.Subsystems.Vision.Target;

public class SubsystemManager {

    public class RobotState {
        enum DriveStates {
            DRIVER_CONTROL,
            PID_CONTROL,
            PATHPLANNER_CONTROL
        }

        enum ElevatorStates {
            FULL_CONTROL // control via setpoint
        }
    
        public DriveStates drive_state = DriveStates.DRIVER_CONTROL;
        public ElevatorStates elevator_state = ElevatorStates.FULL_CONTROL;
    }

    public DriveTrain m_drive;
    public Elevator m_elevator;
    public Vision m_vision;
    public ObjectDetection m_object_detection;

    public SwervePIDController m_swerve_pid_controller;

    public PathplannerController m_pathplanner_controller;

    public RobotState m_robot_state = new RobotState();

    public Commands commands = new Commands();

    public Supplier<Double> getStickX, getStickY, getStickA, getStickEl;

    public Pose2d robot_pose;

    public SwerveDrivePoseEstimator pose_estimator;
    
    public Target tracked_target;

    StructPublisher<Pose2d> adv_pose_pub;

    public Orchestra orchestra;

    /** Creates a new SubsystemManager. */
    public SubsystemManager(
        Supplier<Double> getStickX, 
        Supplier<Double> getStickY, 
        Supplier<Double> getStickA, 
        Supplier<Double> getStickEl) {

        // Initialize subsystems
        if (Robot.isReal()) {
            this.m_drive = new DriveTrainRealIO();
        } else {
            this.m_drive = new DriveTrainSimIO();
            this.m_elevator = new ElevatorSimIO();
        }

        robot_pose = new Pose2d();
        pose_estimator = new SwerveDrivePoseEstimator(
            m_drive.kinematics,
            m_drive.getGyroAngle(),
            m_drive.getModulePositions(),
            robot_pose);

        // Vision needs to be initialized afterwards to have access to pose_estimator
        m_vision = new Vision(pose_estimator::addVisionMeasurement, pose_estimator::getEstimatedPosition);

        this.getStickX = getStickX;
        this.getStickY = getStickY;
        this.getStickA = getStickA;
        this.getStickEl = getStickEl;

        m_swerve_pid_controller = new SwervePIDController(
            pose_estimator::getEstimatedPosition, 
            this::getPidReferenceSpeeds, 
            1);

        m_pathplanner_controller = new PathplannerController(pose_estimator::getEstimatedPosition);

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
                new PIDConstants(AutoConstants.translation_kP, AutoConstants.translation_kI,
                    AutoConstants.translation_kD),
                new PIDConstants(AutoConstants.rotation_kP, AutoConstants.rotation_kI, AutoConstants.rotation_kD)),
            config,
            () -> { 
            if (true) return false;
            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) return alliance.get() == DriverStation.Alliance.Red;
            return false;
            },
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

    public void periodic() {

        robot_pose = pose_estimator.updateWithTime(Timer.getTimestamp(), m_drive.getGyroAngle(), m_drive.getModulePositions());

        switch (m_robot_state.drive_state) {

            case PATHPLANNER_CONTROL:
                m_drive.setSwerveDrive(m_pathplanner_controller.generated_speeds);
                break;

            case PID_CONTROL:
                if (tracked_target != null && tracked_target.marked < VisionConstants.DETECTION_LIMIT) {
                    m_drive.setSwerveDrive(m_swerve_pid_controller.calculate(tracked_target.getPose()), false);
                    break;
                } else {
                    tracked_target = null;
                    // No break; control switches back to the driver if no valid target is present
                }

            default:
                double stick_x = Utils.threshold(-getStickX.get(), 0.1) * SwerveConstants.DRIVE_SPEED;
                double stick_y = Utils.threshold(-getStickY.get(), 0.1) * SwerveConstants.DRIVE_SPEED;
                double stick_a = Utils.threshold(-getStickA.get(), 0.1) * SwerveConstants.TURN_SPEED;

                double stick_el = getStickEl.get();

                double stick_speed = Utils.hypot(stick_x, stick_y);
                // Renormalize movement if combined vector is overspeed
                stick_x *= Math.min(SwerveConstants.DRIVE_SPEED / stick_speed, 1);
                stick_y *= Math.min(SwerveConstants.DRIVE_SPEED / stick_speed, 1);

                m_drive.setSwerveDrive(stick_x, stick_y, stick_a);
        }
        SmartDashboard.putNumber("reef face 2", m_pathplanner_controller.getNearestReefFace());
        publishAdv();
    }

    public void publishAdv() {
        adv_pose_pub.set(pose_estimator.getEstimatedPosition());
    }


    public void setupOrchestra() {
        setupOrchestra(Integer.MAX_VALUE);
    }

    public void setupOrchestra(int tracks) {
        if (Robot.isSimulation()) return; // Abort if robot is simulated

        orchestra.clearInstruments();

        int t = 0;

        for (SwerveModule module : m_drive.modules) {
            // Increment track number by 1 when adding; reset when max tracks reached
            orchestra.addInstrument(((SwerveModuleRealIO)module).drive_motor, t++ % tracks); 
            orchestra.addInstrument(((SwerveModuleRealIO)module).steer_motor, t++ % tracks);
        }
        
        orchestra.addInstrument(((ElevatorRealIO)m_elevator).left_motor, t++ % tracks);
        orchestra.addInstrument(((ElevatorRealIO)m_elevator).right_motor, t++ % tracks);
    }
 
    
    public class Commands {

        public Command trackAlgae() {
            return new FunctionalCommand(
                () -> {m_robot_state.drive_state = RobotState.DriveStates.PID_CONTROL;
                    tracked_target = m_vision.m_object_detection.getClosestTarget();
                    m_swerve_pid_controller.reset_speeds = true; // Reset pid speeds to real measured for acceleration calculations
                },
                () -> {}, 
                canceled -> {m_robot_state.drive_state = RobotState.DriveStates.DRIVER_CONTROL;}, 
                () -> m_robot_state.drive_state != RobotState.DriveStates.PID_CONTROL || tracked_target == null);
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
                    orchestra.play();},
                () -> {},
                canceled -> orchestra.stop(),
                () -> !orchestra.isPlaying());
        }

    }
}