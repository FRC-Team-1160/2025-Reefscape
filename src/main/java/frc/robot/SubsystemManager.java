package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.Subsystems.Claw.Claw;
import frc.robot.Commands.AlgaeAlignmentPID;
import frc.robot.Subsystems.Elevator;
import frc.robot.Subsystems.DriveTrain.DriveTrain;
import frc.robot.Subsystems.DriveTrain.DriveTrainRealIO;
import frc.robot.Subsystems.DriveTrain.DriveTrainSimIO; //Simulation can't identify sim class without this for some reason
import frc.robot.Subsystems.Vision.Vision;
import frc.robot.Subsystems.Vision.ObjectDetection;

public class SubsystemManager {

    public class RobotState {
        enum DriveStates {
            FULL_CONTROL,
            PID_CONTROL
        }

        enum ElevatorStates {
            FULL_CONTROL // control via setpoint
        }
    
        public DriveStates drive_state = DriveStates.FULL_CONTROL;
        public ElevatorStates elevator_state = ElevatorStates.FULL_CONTROL;
    }

    public DriveTrain m_drive;
    public Elevator m_elevator;
    public Vision m_vision;
    public ObjectDetection m_object_detection;
    public Claw m_claw;

    public AlgaeAlignmentPID algae_alignment_PID;


    public RobotState robot_state = new RobotState();

    public Supplier<Double> getStickX, getStickY, getStickA, getStickEl;

    Pose2d robot_pose;
    SwerveDrivePoseEstimator m_pose_estimator;

    StructPublisher<Pose2d> adv_pose_pub;

    public SubsystemManager(Supplier<Double> getStickX, Supplier<Double> getStickY, Supplier<Double> getStickA, Supplier<Double> getStickEl) {

        if (Robot.isSimulation()) {
            this.m_drive = new DriveTrainSimIO();
        } else {
            this.m_drive = new DriveTrainRealIO();
            this.m_claw = new Claw();
            this.m_elevator = new Elevator();
            // m_vision = new Vision();
            m_object_detection = new ObjectDetection();
        }

        this.getStickX = getStickX;
        this.getStickY = getStickY;
        this.getStickA = getStickA;
        this.getStickEl = getStickEl;

        algae_alignment_PID = new AlgaeAlignmentPID(m_object_detection, m_drive);

        // robot_pose = new Pose2d();
        // m_pose_estimator = new SwerveDrivePoseEstimator(
        // m_drive.m_kinematics,
        // m_drive.getGyroAngle(),
        // m_drive.getModulePositions(),
        // robot_pose);

        setupDashboard();

    }

    public void setupDashboard() {
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable adv_swerve = inst.getTable("adv_swerve");
        adv_pose_pub = adv_swerve.getStructTopic("Pose", Pose2d.struct).publish();
    }

    public void periodic_disabled() {

    }

    public void periodic() {
        double stick_x = getStickX.get();
        double stick_y = getStickY.get();
        double stick_a = getStickA.get();
        double stick_el = getStickEl.get();


        stick_x = (Math.abs(stick_x) < 0.1) ? 0 : stick_x;
        stick_y = (Math.abs(stick_y) < 0.1) ? 0 : stick_y;
        stick_a = (Math.abs(stick_a) < 0.1) ? 0 : stick_a;

        // stick_x = stick_x/Math.sqrt(stick_x*stick_x + stick_y*stick_y);
        // stick_y = stick_y/Math.sqrt(stick_x*stick_x + stick_y*stick_y);

        double drive_x = -stick_x * Constants.Swerve.DRIVE_SPEED;
        double drive_y = -stick_y * Constants.Swerve.DRIVE_SPEED;
        double drive_a = -stick_a * Constants.Swerve.TURN_SPEED;

        if (!algae_alignment_PID.isScheduled()) {
            m_drive.setSwerveDrive(drive_x, drive_y, drive_a);
        }

        m_elevator.setpoint += Constants.Elevator.MAX_SPEED*stick_el;

        publishAdv();
    }

    public void publishAdv() {
        adv_pose_pub.set(m_drive.odom_pose);
    }
}