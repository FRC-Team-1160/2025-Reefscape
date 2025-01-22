// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.DriveTrain.DriveTrain;
import frc.robot.Subsystems.Vision.ObjectDetection;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlgaeAlignmentPID extends Command {
  /** Creates a new AlgaeAlignmentPID. */
  Pose2d targetPose;
  Pose2d robotPose;

  TrapezoidProfile.Constraints X_CONSTRAINTS = 
      new TrapezoidProfile.Constraints(0.5, 1);
  TrapezoidProfile.Constraints Y_CONSTRAINTS = 
      new TrapezoidProfile.Constraints(0.5,1);
  TrapezoidProfile.Constraints OMEGA_CONSTRAINTS = 
      new TrapezoidProfile.Constraints(Math.toRadians(90), Math.toRadians(180));

  ObjectDetection m_ObjectDetection;
  DriveTrain m_DriveTrain;

  ProfiledPIDController xController 
      = new ProfiledPIDController(1, 0, 0, X_CONSTRAINTS);
  ProfiledPIDController yController 
      = new ProfiledPIDController(1, 0, 0, Y_CONSTRAINTS);
  ProfiledPIDController omegaController 
      = new ProfiledPIDController(1, 0, 0, OMEGA_CONSTRAINTS);

  Transform3d OBJECT_TO_GOAL = new Transform3d(
      // 0.5 m behind
      new Translation3d(0.8, 0.0, 0.0),

      new Rotation3d(0.0, 0.0, Math.PI)
  );
  

  public AlgaeAlignmentPID(
    ObjectDetection m_ObjectDetection,
    DriveTrain m_DriveTrain
  ) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_ObjectDetection = m_ObjectDetection;
    this.m_DriveTrain = m_DriveTrain;

    xController.setTolerance(0.2);
    yController.setTolerance(0.2);
    omegaController.setTolerance(Units.degreesToRadians(3));
    
    omegaController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(m_ObjectDetection, m_DriveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    robotPose = m_DriveTrain.m_odom_pose;
    xController.reset(robotPose.getX());
    yController.reset(robotPose.getY());
    omegaController.reset(robotPose.getRotation().getRadians());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    robotPose = m_DriveTrain.m_odom_pose;

    Pose3d currentRobotPose3d = new Pose3d(
        robotPose.getX(),
        robotPose.getY(),
        0.0,
        new Rotation3d(0, 0, robotPose.getRotation().getRadians())
    );

    if (m_ObjectDetection.closestPose != null) {
      Pose3d objectPose = m_ObjectDetection.closestPose;
      
      // offset from the algae
      var goalPose = objectPose.transformBy(OBJECT_TO_GOAL).toPose2d();

      xController.setGoal(goalPose.getX());
      yController.setGoal(goalPose.getY());
      omegaController.setGoal(goalPose.getRotation().getRadians());

      double xSpeed = xController.calculate(currentRobotPose3d.getX());
      if (xController.atGoal()) {
        xSpeed = 0;
      }

      double ySpeed = yController.calculate(currentRobotPose3d.getY());
      if (yController.atGoal()) {
        ySpeed = 0;
      }

      double omegaSpeed = omegaController.calculate(robotPose.getRotation().getRadians());
      if (omegaController.atGoal()) {
        omegaSpeed = 0;
      }
      System.out.println(xSpeed);

      
      m_DriveTrain.setSwerveDrive(xSpeed, ySpeed, omegaSpeed);
    } else {
      m_DriveTrain.setSwerveDrive(0, 0, 0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_DriveTrain.setSwerveDrive(0,0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
