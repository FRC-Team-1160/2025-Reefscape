// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.DriveTrain.DriveTrain;
import frc.robot.Subsystems.Vision.ObjectDetection;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlgaeAlignmentPID extends Command {
  /** Creates a new AlgaeAlignmentPID. */
  Pose2d target_pose;
  Pose2d robot_pose;

  // TrapezoidProfile.Constraints X_CONSTRAINTS = 
  //     new TrapezoidProfile.Constraints(0.5, 1);
  // TrapezoidProfile.Constraints Y_CONSTRAINTS = 
  //     new TrapezoidProfile.Constraints(0.5,1);
  // TrapezoidProfile.Constraints OMEGA_CONSTRAINTS = 
  //     new TrapezoidProfile.Constraints(Math.toRadians(90), Math.toRadians(180));

  // final??? maybe
  // edge case to be added to style guide
  TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(2, 2);
  TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(2, 2);
  TrapezoidProfile.Constraints OMEGA_CONSTRAINTS = new TrapezoidProfile.Constraints(Math.toRadians(90), Math.toRadians(180));

  ObjectDetection m_object_detection;
  DriveTrain m_drivetrain;
  
  ProfiledPIDController x_controller
      = new ProfiledPIDController(1, 0, 0, X_CONSTRAINTS);
  ProfiledPIDController y_controller
      = new ProfiledPIDController(1, 0, 0, Y_CONSTRAINTS);
  ProfiledPIDController omega_controller
      = new ProfiledPIDController(1, 0, 0, OMEGA_CONSTRAINTS);

  Transform3d OBJECT_TO_GOAL = new Transform3d(
      // 0.5 m behind
      new Translation3d(0.5, 0.0, 0.0),
      new Rotation3d(0.0, 0.0, Math.PI)
  );

  public static boolean running = false;
  

  public AlgaeAlignmentPID(ObjectDetection m_object_detection, DriveTrain m_drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_object_detection = m_object_detection;
    this.m_drivetrain = m_drivetrain;

    x_controller.setTolerance(0.2);
    y_controller.setTolerance(0.2);
    omega_controller.setTolerance(Units.degreesToRadians(10));
    
    omega_controller.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(m_object_detection, m_drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    robot_pose = m_drivetrain.odom_pose;
    x_controller.reset(robot_pose.getX());
    y_controller.reset(robot_pose.getY());
    omega_controller.reset(robot_pose.getRotation().getRadians());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    running = true;
    robot_pose = m_drivetrain.odom_pose;

    Pose3d currentRobotPose3d = new Pose3d(
        robot_pose.getX(),
        robot_pose.getY(),
        0.0,
        new Rotation3d(0, 0, robot_pose.getRotation().getRadians())
    );

    if (m_object_detection.closest_pose != null) {
      Pose3d objectPose = m_object_detection.closest_pose;
      
      // offset from the algae
      var goalPose = objectPose.transformBy(OBJECT_TO_GOAL).toPose2d();

      x_controller.setGoal(goalPose.getX());
      y_controller.setGoal(goalPose.getY());
      // omegaController.setGoal(goalPose.getRotation().getRadians());
      omega_controller.setGoal(Math.atan2((objectPose.getY()-robot_pose.getY()),(objectPose.getX() - robot_pose.getX())));
      SmartDashboard.putNumber("targetAngle", omega_controller.getSetpoint().position);

      double xSpeed = x_controller.calculate(currentRobotPose3d.getX());
      if (x_controller.atGoal()) {
        xSpeed = 0;
      }

      double ySpeed = y_controller.calculate(currentRobotPose3d.getY());
      if (y_controller.atGoal()) {
        ySpeed = 0;
      }

      double omegaSpeed = omega_controller.calculate(robot_pose.getRotation().getRadians());
      SmartDashboard.putNumber("givenAngle", robot_pose.getRotation().getRadians());
      if (omega_controller.atGoal()) {
        omegaSpeed = 0;
      }
      System.out.println(xSpeed);

      
      m_drivetrain.setSwerveDrive(xSpeed, ySpeed, omegaSpeed);
    } else {
      m_drivetrain.setSwerveDrive(0, 0, 0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_drivetrain.setSwerveDrive(0,0,0);
    running = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
