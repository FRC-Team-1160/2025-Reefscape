package frc.robot;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Subsystems.DriveTrain.DriveTrain;
import frc.robot.Subsystems.DriveTrain.DriveTrainRealIO;
import frc.robot.Subsystems.DriveTrain.DriveTrainSimIO; //Simulation can't identify sim class without this for some reason

import frc.robot.Subsystems.Vision.*;

public class SubsystemManager {

    public DriveTrain m_drive;
    public Vision m_vision;

    Pose2d robot_pose;
    SwerveDrivePoseEstimator m_pose_estimator;

    StructPublisher<Pose2d> adv_pose_pub;

    public SubsystemManager(){

        if (Robot.isSimulation()){
            this.m_drive = new DriveTrainSimIO();
            
        } else {
            this.m_drive = new DriveTrainRealIO();
            m_vision = new Vision();
        }

        // robot_pose = new Pose2d();
        // m_pose_estimator = new SwerveDrivePoseEstimator(
        //     m_drive.m_kinematics, 
        //     m_drive.getGyroAngle(), 
        //     m_drive.getModulePositions(), 
        //     robot_pose);

        setupDashboard();

    }

    public void setupDashboard(){
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable adv_swerve = inst.getTable("adv_swerve");
        adv_pose_pub = adv_swerve.getStructTopic("Pose", Pose2d.struct).publish();
    }

    public void periodic(double stick_x, double stick_y, double stick_a){

        // robot_pose = m_pose_estimator.update(m_drive.getGyroAngle(), m_drive.getModulePositions());

        // LimelightHelpers.SetRobotOrientation("", m_drive.getGyroAngle().getDegrees(), 0.0, 0.0, 0.0, 0.0, 0.0);
        // LimelightHelpers.PoseEstimate vision_estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue("");
        // m_pose_estimator.addVisionMeasurement(vision_estimate.pose, vision_estimate.timestampSeconds);

        stick_x = (Math.abs(stick_x) < 0.1) ? 0 : stick_x;
        stick_y = (Math.abs(stick_y) < 0.1) ? 0 : stick_y;
        stick_a = (Math.abs(stick_a) < 0.1) ? 0 : stick_a;

        double drive_x = -stick_x * 0.5; //* SwerveConstants.MAX_SPEED;
        double drive_y = -stick_y * 0.5; //* SwerveConstants.MAX_SPEED;
        double drive_a = -stick_a * 0.5;

        SmartDashboard.putNumber("in_x", drive_x);
        SmartDashboard.putNumber("in_y", drive_y);
        SmartDashboard.putNumber("in_a", drive_a);

        m_drive.setSwerveDrive(drive_x, drive_y, drive_a);

        publishAdv();
    }

    public void publishAdv(){
        adv_pose_pub.set(m_drive.m_odom_pose);
    }

}