// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.SwerveConstants.Tracking;
import frc.robot.Constants.SwerveConstants.Tracking.PIDConstants.Angle;
import frc.robot.Constants.SwerveConstants.Tracking.PIDConstants.Distance;


public class SwervePIDController {

  Pose2d target_pose;

  Pose2d robot_pose;

  StructPublisher<Pose2d> adv_goal_pose_pub;

  PIDController dist_pid_controller, ang_pid_controller;

  Supplier<Pose2d> robot_pose_supplier;

  Supplier<ChassisSpeeds> robot_speeds_supplier;

  double DISTANCE;

  public boolean reset_speeds;

  public SwervePIDController(Supplier<Pose2d> robot_pose_supplier, Supplier<ChassisSpeeds> robot_speeds_supplier) {
    this(robot_pose_supplier, robot_speeds_supplier, 0.5);
  }

  public SwervePIDController(Supplier<Pose2d> robot_pose_supplier, Supplier<ChassisSpeeds> robot_speeds_supplier, double target_distance) {

    target_pose = new Pose2d();
    robot_pose = robot_pose_supplier.get();

    dist_pid_controller = new PIDController(Distance.kP, Distance.kI, Distance.kD);
    dist_pid_controller.setTolerance(Tracking.DISTANCE_TOLERANCE);

    ang_pid_controller = new PIDController(Angle.kP, Angle.kI, Angle.kD);
    ang_pid_controller.setTolerance(Tracking.ANGLE_TOLERANCE);
    ang_pid_controller.enableContinuousInput(-Math.PI, Math.PI);

    this.robot_pose_supplier = robot_pose_supplier;
    this.robot_speeds_supplier = robot_speeds_supplier;

    DISTANCE = target_distance;

    NetworkTable adv_vision = NetworkTableInstance.getDefault().getTable("adv_vision");
    adv_goal_pose_pub = adv_vision.getStructTopic("Goal Pose", Pose2d.struct).publish();

    reset_speeds = true;
    
  }

  public ChassisSpeeds calculate(Pose2d target_pose, double target_distance) {
    DISTANCE = target_distance;
    return calculate(target_pose);
  }

  public ChassisSpeeds calculate(Pose2d target_pose) {
    this.target_pose = target_pose;
    return calculate();
  }

  public ChassisSpeeds calculate() {

    robot_pose = robot_pose_supplier.get();
    ChassisSpeeds current_speeds = robot_speeds_supplier.get();

    // Find robot to target translation
    Translation2d target_off = target_pose.getTranslation().minus(robot_pose.getTranslation());
    Rotation2d target_angle = target_off.getAngle();
    // Add extra space for turning to goal distance if robot is not pointing towards target
    double goal_dist = DISTANCE + Math.min(0, 
      Tracking.MAX_ALIGN_SEPARATION * (Math.abs(target_angle.minus(robot_pose.getRotation()).getRotations()) - Tracking.ALIGN_SEPARATION_TOLERANCE));

    adv_goal_pose_pub.set(new Pose2d(
        target_pose.getTranslation().minus(target_off.times(goal_dist/target_off.getNorm())),
        target_angle));

    dist_pid_controller.setSetpoint(goal_dist);
    ang_pid_controller.setSetpoint(target_angle.getRadians());

    // Distance pid controller returns desired change in distance (away being positive); take negative to convert to towards being positive
    double desired_velocity = -Math.min(dist_pid_controller.calculate(target_off.getNorm()), Tracking.MAX_SPEED);

    Translation2d desired_speeds_vector = new Translation2d(
      desired_velocity * target_angle.getCos(),
      desired_velocity * target_angle.getSin());

    Translation2d current_speeds_vector = new Translation2d(
      current_speeds.vxMetersPerSecond,
      current_speeds.vyMetersPerSecond)
        .rotateBy(robot_pose.getRotation());

    // Calculate acceleration and reset requested speeds based on max acceleration
    Translation2d accel_vector = desired_speeds_vector.minus(current_speeds_vector);
    accel_vector = accel_vector.times(Math.min(Tracking.MAX_ACCEL * RobotConstants.LOOP_TIME_SECONDS / accel_vector.getNorm(), 1));
    desired_speeds_vector = current_speeds_vector.plus(accel_vector).rotateBy(robot_pose.getRotation().unaryMinus());
    // Calculate angular velocity and acceleration
    double desired_ang_speed = ang_pid_controller.atSetpoint() ? 0 : Utils.clamp(ang_pid_controller.calculate(robot_pose.getRotation().getRadians()), Tracking.MAX_ANG_SPEED);
    desired_ang_speed = current_speeds.omegaRadiansPerSecond + 
      Utils.clamp(desired_ang_speed - current_speeds.omegaRadiansPerSecond, Tracking.MAX_ANG_ACCEL * RobotConstants.LOOP_TIME_SECONDS);

    return new ChassisSpeeds(desired_speeds_vector.getX(), desired_speeds_vector.getY(), desired_ang_speed);

  }

}