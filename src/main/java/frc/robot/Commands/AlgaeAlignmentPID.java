// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.DriveTrain.DriveTrain;
import frc.robot.Subsystems.Vision.ObjectDetection;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlgaeAlignmentPID extends Command {
  /** Creates a new AlgaeAlignmentPID. */
  Pose2d target_pose;

  StructPublisher<Pose2d> adv_pose_pub;

  TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 3);
  TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 3);

  TrapezoidProfile.Constraints OMEGA_CONSTRAINTS = new TrapezoidProfile.Constraints(Math.toRadians(40),
      Math.toRadians(20));

  ObjectDetection m_object_detection;
  DriveTrain m_drivetrain;

  ProfiledPIDController x_controller = new ProfiledPIDController(1, 0, 0, X_CONSTRAINTS);
  ProfiledPIDController y_controller = new ProfiledPIDController(1, 0, 0, Y_CONSTRAINTS);
  ProfiledPIDController omega_controller = new ProfiledPIDController(1, 0, 0, OMEGA_CONSTRAINTS);

  ProfiledPIDController x_controller = new ProfiledPIDController(1, 0, 0, X_CONSTRAINTS);
  ProfiledPIDController y_controller = new ProfiledPIDController(1, 0, 0, Y_CONSTRAINTS);
  ProfiledPIDController omega_controller = new ProfiledPIDController(1, 0, 0, OMEGA_CONSTRAINTS);

  public AlgaeAlignmentPID(
      ObjectDetection m_object_detection,
      DriveTrain m_drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_object_detection = m_object_detection;
    this.m_drivetrain = m_drivetrain;

    x_controller.setTolerance(0.1);
    y_controller.setTolerance(0.1);
    omega_controller.setTolerance(Units.degreesToRadians(5));

    omega_controller.enableContinuousInput(-Math.PI, Math.PI);


    addRequirements(m_object_detection, m_drivetrain);


    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable adv_vision = inst.getTable("adv_vision");
    adv_pose_pub = adv_vision.getStructTopic("Goal Pose", Pose2d.struct).publish();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    x_controller.reset(m_drivetrain.odom_pose.getX());
    y_controller.reset(m_drivetrain.odom_pose.getY());
    omega_controller.reset(m_drivetrain.odom_pose.getRotation().getRadians());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (m_object_detection.closest_pose != null) {
      Pose3d object_pose = m_object_detection.closest_pose;
      double x_dist = object_pose.getX() - m_drivetrain.odom_pose.getX();
      double y_dist = object_pose.getY() - m_drivetrain.odom_pose.getY();

      double target_angle = Math.atan2(y_dist, x_dist);

      double dist = Math.sqrt(Math.pow(x_dist, 2) + Math.pow(y_dist, 2));

      SmartDashboard.putNumber("x_dist", x_dist);
      SmartDashboard.putNumber("y_dist", y_dist);
      SmartDashboard.putNumber("dist", dist);
      SmartDashboard.putNumber("t_ang", target_angle);

      Pose2d goal_pose = new Pose2d( // no likey transform2d
          object_pose.getX() - 0.8 * x_dist / dist,
          object_pose.getY() - 0.8 * y_dist / dist,
          Rotation2d.fromRadians(target_angle));

      adv_pose_pub.set(goal_pose);

      x_controller.setGoal(goal_pose.getX());
      y_controller.setGoal(goal_pose.getY());
      omega_controller.setGoal(goal_pose.getRotation().getRadians());

      double x_speed = x_controller.calculate(m_drivetrain.odom_pose.getX());
      if (x_controller.atSetpoint()) {
        x_speed = 0;
      }

      double y_speed = y_controller.calculate(m_drivetrain.odom_pose.getY());
      if (y_controller.atSetpoint()) {
        y_speed = 0;
      } 

      double omega_speed = omega_controller.calculate(m_drivetrain.odom_pose.getRotation().getRadians());
      if (omega_controller.atSetpoint()) {
        omega_speed = 0;
      }

      // normalize speed
      double max_speed = 3;
      double max_accel = 3;

      double req_speed = pythag(x_speed, y_speed);
      if (req_speed > max_speed) {
        x_speed *= max_speed / req_speed;
        y_speed *= max_speed / req_speed;
      }
      Translation2d current_velocity = new Translation2d(x_speed, y_speed);

      double x_accel = x_speed - current_velocity.getX();
      double y_accel = y_speed - current_velocity.getY();
      double req_accel = pythag(x_accel, y_accel);
      if (req_accel > max_accel * 0.02) {
        x_accel *= max_accel / req_accel;
        y_accel *= max_accel / req_accel;
      }
      x_speed = current_velocity.getX() + x_accel;
      y_speed = current_velocity.getY() + y_accel;

      m_drivetrain.setSwerveDrive(x_speed, y_speed, omega_speed);
    } else {
      m_drivetrain.setSwerveDrive(0, 0, 0);
    }
  }

  public double pythag(double a, double b) {
    return Math.sqrt(Math.pow(a, 2) + Math.pow(b, 2));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.setSwerveDrive(0, 0, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}