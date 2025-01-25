// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
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
  Pose2d targetPose;
  Pose2d robotPose;

  StructPublisher<Pose2d> adv_posePub;

  public static boolean running = false;

  TrapezoidProfile.Constraints X_CONSTRAINTS = 
      new TrapezoidProfile.Constraints(3, 3);
  TrapezoidProfile.Constraints Y_CONSTRAINTS = 
      new TrapezoidProfile.Constraints(3,3);
  TrapezoidProfile.Constraints OMEGA_CONSTRAINTS = 
      new TrapezoidProfile.Constraints(Math.toRadians(40), Math.toRadians(20));

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
      new Translation3d(-0.5, 0.0, 0.0),

      new Rotation3d(0.0, 0.0, -Math.PI)
  );
  

  public AlgaeAlignmentPID(
    ObjectDetection m_ObjectDetection,
    DriveTrain m_DriveTrain
  ) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_ObjectDetection = m_ObjectDetection;
    this.m_DriveTrain = m_DriveTrain;

    xController.setTolerance(0.1);
    yController.setTolerance(0.1);
    omegaController.setTolerance(Units.degreesToRadians(5));
    
    omegaController.enableContinuousInput(-Math.PI, Math.PI);

    addRequirements(m_ObjectDetection, m_DriveTrain);

    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable adv_vision = inst.getTable("adv_vision");
    adv_posePub = adv_vision.getStructTopic("Goal Pose", Pose2d.struct).publish();
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
    // System.out.println(m_ObjectDetection.closestTarget.distance);
    // if (m_ObjectDetection.closestTarget.distance >= 0.5 || (m_ObjectDetection.closestTarget.offset >= 0.1 || m_ObjectDetection.closestTarget.offset <= -0.1)){
      if (true){
      running = true;
      robotPose = m_DriveTrain.m_odom_pose;
  
      Pose3d currentRobotPose3d = new Pose3d(
          robotPose.getX(),
          robotPose.getY(),
          0.0,
          new Rotation3d(0, 0, robotPose.getRotation().getRadians())
      );
  
      if (m_ObjectDetection.closestPose != null) {
        Pose3d objectPose = m_ObjectDetection.closestPose;
        double x_dist = objectPose.getX() - robotPose.getX();
        double y_dist = objectPose.getY() - robotPose.getY();

        double target_angle = Math.atan2(y_dist, x_dist);

        double dist = Math.sqrt(Math.pow(x_dist, 2) + Math.pow(y_dist, 2));

        SmartDashboard.putNumber("x_dist", x_dist);
        SmartDashboard.putNumber("y_dist", y_dist);
        SmartDashboard.putNumber("dist", dist);
        SmartDashboard.putNumber("t_ang", target_angle);
        // offset from the algae
        // Pose2d goalPose = objectPose.toPose2d().transformBy(new Transform2d(
        //   -x_dist,
        //   -y_dist,
        //   new Rotation2d()
        // ));

        Pose2d goalPose = new Pose2d( //no likey transform2d
          objectPose.getX() - 0.8 * x_dist / dist,
          objectPose.getY() - 0.8 * y_dist / dist,
          Rotation2d.fromRadians(target_angle)
        );

        adv_posePub.set(goalPose);
  
        xController.setGoal(goalPose.getX());
        yController.setGoal(goalPose.getY());
        // omegaController.setGoal(goalPose.getRotation().getRadians());
        omegaController.setGoal(goalPose.getRotation().getRadians());
  
        double xSpeed = xController.calculate(currentRobotPose3d.getX());
        if (xController.atSetpoint()) {
          xSpeed = 0;
        }
  
        double ySpeed = yController.calculate(currentRobotPose3d.getY());
        if (yController.atSetpoint()) {
          ySpeed = 0;
        }
  
        double omegaSpeed = omegaController.calculate(robotPose.getRotation().getRadians());
        if (omegaController.atSetpoint()) {
          omegaSpeed = 0;
        }
        // System.out.println(xSpeed);
  
        
        m_DriveTrain.setSwerveDrive(xSpeed, ySpeed, omegaSpeed);
      } else {
        m_DriveTrain.setSwerveDrive(0, 0, 0);
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_DriveTrain.setSwerveDrive(0,0,0);
    running = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
