package frc.robot;

import java.util.function.BooleanSupplier;

import org.ejml.equation.Function;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Subsystems.DriveTrain.*;
import frc.robot.Subsystems.Vision.*;

public class SubsystemManager {

    DriveTrain m_drive;
    Vision m_vision;
    ObjectDetection m_ObjectDetection;

    Pose2d robot_pose;
    SwerveDrivePoseEstimator m_pose_estimator;

    StructPublisher<Pose2d> adv_pose_pub;

    Commands commands;


    public SubsystemManager(){

        if (Robot.isSimulation()){
            
        } else {
            this.m_drive = new DriveTrainRealIO();
            m_vision = new Vision();
            m_ObjectDetection = new ObjectDetection();
        }

        // robot_pose = new Pose2d();
        // m_pose_estimator = new SwerveDrivePoseEstimator(
        //     m_drive.m_kinematics, 
        //     m_drive.getGyroAngle(), 
        //     m_drive.getModulePositions(), 
        //     robot_pose);

        commands = new Commands();

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

        double drive_x = stick_x * 0.25;//* SwerveConstants.MAX_SPEED;
        double drive_y = stick_y * 0.25; //* SwerveConstants.MAX_SPEED;
        double drive_a = stick_a * 0.25;

        m_drive.setSwerveDrive(drive_x, drive_y, drive_a);

        publishAdv();
    }

    public void publishAdv(){
        adv_pose_pub.set(m_drive.m_odom_pose);
    }

    class Commands {

    }
}