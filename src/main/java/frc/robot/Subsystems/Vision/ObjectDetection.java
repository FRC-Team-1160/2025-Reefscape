// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Vision;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.networktables.StructSubscriber;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Vision;
import frc.robot.Robot;

public class ObjectDetection extends SubsystemBase {
  private PhotonCamera detector;
  
  public boolean has_target;
  public Pose3d closest_pose;
  private ArrayList<Pose3d> target_poses = new ArrayList<>();
  private ArrayList<Target> target_distances = new ArrayList<>();
  public Target closest_target = new Target();

  private Pose2d robot_pose;

  StructPublisher<Pose3d> adv_closest_pub;
  StructArrayPublisher<Pose3d> adv_targets_pub;
  StructSubscriber<Pose2d> adv_robot_pose_sub;

  StructArrayPublisher<Translation2d> adv_corners_pub; // jank

  // TODO: potentially unneeded?
  /** in meters */
  double closest_distance;

  /** Creates a new ObjectDetection. */
  public ObjectDetection() {
    if (!Robot.isSimulation()) {
      detector = new PhotonCamera("OV9782");
    }
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable adv_vision = inst.getTable("adv_vision");
    NetworkTable adv_swerve = inst.getTable("adv_swerve");
    adv_targets_pub = adv_vision.getStructArrayTopic("Target", Pose3d.struct).publish();
    adv_closest_pub = adv_vision.getStructTopic("Closest", Pose3d.struct).publish();
    adv_corners_pub = adv_vision.getStructArrayTopic("Target Corners", Translation2d.struct).publish();

    adv_robot_pose_sub = adv_swerve.getStructTopic("Pose", Pose2d.struct).subscribe(new Pose2d(),
        PubSubOption.keepDuplicates(true));
  }

  public Pose3d getObjectPose3D(Pose2d robot_pose, double distance, double angle_to_target) {
    // Robot's global heading
    double robot_theta = robot_pose.getRotation().getRadians();

    // Convert distance + angle to object coords in the field frame:
    double global_angle = robot_theta + angle_to_target; // field heading to the target
    double x = robot_pose.getX() + (distance * Math.cos(global_angle));
    double y = robot_pose.getY() + (distance * Math.sin(global_angle));

    // If you don't know the object's actual heading, just store globalAngle or 0.
    // We'll just store the object's "facing" as globalAngle here for demonstration.
    return new Pose3d(x, y, 0.21, new Rotation3d(0, 0, global_angle));
  }

  // public static double[] getDistance(double targetWidthPixel, double
  // targetXPixel){
  // double c = targetWidthPixel / (2 * Math.tanh(cameraHFOV));
  // double distance = targetWidthM / targetWidthPixel * c;

  // double offset = (targetXPixel - targetWidthPixel/2) * c * distance;
  // double[] result = {distance, offset};
  // return result;
  // }

  /**
   * 
   * @param image_width in pixels
   * @param x_fov in radians
   * @return
   */
  private double getFocalLength(int image_width, double x_fov) {
    return (image_width / 2.0d) / Math.tan(x_fov / 2.0d);
  }

  
  /**
   * @param target_width in pixels
   * @param target_height in pixels
   * @param target_x in pixels
   * @return [distance, offset] in meters
   */
  public double[] getDistance(double target_width, double target_height, double target_x) {
    // 1) Distance: d = (focalLengthPx * realWidthMeters) / boundingBoxWidthPx
    double distance;
    // in pixels
    double focal_length = getFocalLength(Vision.SCREEN_WIDTH, Vision.CAMERA_X_FOV);
    if (target_width > 0) {
      distance = 250.0d / target_width; // (focalLengthPx * targetWidthM) / targetWidthPixel;
      SmartDashboard.putNumber("Vison Distance", distance);

    } else {
      distance = Double.POSITIVE_INFINITY;
    }

    distance = 261/(Math.max(target_width, target_height)) - 0.04 + 0.35;

    double image_center = Vision.SCREEN_WIDTH / 2.0;
    double fraction_off_center = (target_x - image_center) / image_center;
    double offset_angle = fraction_off_center * (Vision.CAMERA_X_FOV / 2.0); // in radians
    double horizontal_offset = distance * Math.sin(offset_angle);

    return new double[] { distance, horizontal_offset };
  }

  public Pose3d getClosestTarget(ArrayList<Target> targets) {
    double min_dist = Double.MAX_VALUE;
    Target result = new Target();
    for (Target target : targets) {
      if (target.direct_distance < min_dist) {
        min_dist = target.direct_distance;
        result = target;
      }
    }
    double angle_to_target = Math.atan(result.offset / result.distance);
    return getObjectPose3D(robot_pose, result.distance, angle_to_target);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // closestDistance = getDistance(413.0, 2.4, 413.0);

    if (Robot.isSimulation()) {
      closest_pose = new Pose3d(1, 7, 0, new Rotation3d(0, 0, Math.PI));
      adv_closest_pub.set(closest_pose);
      return;
    }

      var result = detector.getLatestResult();
      List<PhotonTrackedTarget> targets = result.getTargets();
      for (PhotonTrackedTarget target : targets) {
        double minX = Double.MIN_VALUE; double minY = Double.MIN_VALUE;
        double maxX = Double.MAX_VALUE; double maxY = Double.MAX_VALUE;
        
        for (TargetCorner corner : target.getMinAreaRectCorners()) {
          minX = Math.min(minX, corner.x);
          maxX = Math.max(maxX, corner.x);
          minY = Math.min(minY, corner.y);
          maxY = Math.max(maxY, corner.y);
        }
        
        int midpoint = (maxX + minX) / 2;
        double width = maxX - minX;
        double height = maxY - minY;

        SmartDashboard.putNumber("minY", minY);
        SmartDashboard.putNumber("maxY", maxY);
        SmartDashboard.putNumber("tempHeight", height);


        double[] temp_dist = getDistance(width, height, midpoint);
        // double distance = tempDistance[0] + 0.55;
        double distance = test_dist[0];

        double offset = - temp_dist[1] + 0.24;
        // double distance = 230 / tempWidthPixel;
        // double offset = -(tempMidpoint - (horizontalScreenPixel/2))*0.012 - 0.28;
        double angle_to_target = Math.atan(offset/distance);
        double direct_distance = Math.sqrt(Math.pow(distance, 2) + Math.pow(offset, 2));
        SmartDashboard.putNumber("tempHeight2", height);
        System.out.println(distance);

        target_poses.add(getObjectPose3D(robot_pose, distance, angle_to_target));
        target_distances.add(new Target(distance, offset, direct_distance));
      }

      if (target_poses.size() >= 1) {
        // adv_targetsPub.set(targetPoses);
        adv_closest_pub.set(target_poses.get(0));
        closest_pose = getClosestTarget(target_distances);
      }
    }
  }
}
