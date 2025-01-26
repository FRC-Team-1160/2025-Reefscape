// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import java.util.List;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Subsystems.DriveTrain.DriveTrain;
import frc.robot.Subsystems.Vision.ObjectDetection;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AlgaeAlignment extends Command {
  /** Creates a new AlgaeAlignment. */
  Pose2d target_pose;
  Pose2d robot_pose;

  ObjectDetection m_object_detection;
  DriveTrain m_drivetrain;

  PathPlannerPath path;

  public AlgaeAlignment(ObjectDetection m_object_detection, DriveTrain m_drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_object_detection, m_drivetrain);
    this.m_object_detection = m_object_detection;
    this.m_drivetrain = m_drivetrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (m_object_detection.has_target == false) {
      System.out.println("no target pose");
      this.cancel();
      return;
    }

    RobotConfig config;

    try {
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      config = null;
    }

    FollowPathCommand cmd = new FollowPathCommand(
      path, 
      () -> m_drivetrain.odom_pose, 
      () -> m_drivetrain.kinematics.toChassisSpeeds(m_drivetrain.getModuleStates()),
      null, 
      new PPHolonomicDriveController(null, null), 
      config, 
      () -> false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    target_pose = m_object_detection.closest_pose.toPose2d();

    List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
        target_pose
    );
    PathConstraints constraints = new PathConstraints(0.5, 1, 0.5 * Math.PI, Math.PI); // The constraints for this path.
    path = new PathPlannerPath(
        waypoints,
        constraints,
        null, // The ideal starting state, this is only relevant for pre-planned paths, so can be null for on-the-fly paths.
        new GoalEndState(0.0, Rotation2d.fromDegrees(-90)) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
    );

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
