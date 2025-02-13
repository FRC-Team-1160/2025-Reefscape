// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Vision;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import com.fasterxml.jackson.core.util.DefaultPrettyPrinter.FixedSpaceIndenter;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;

import edu.wpi.first.math.geometry.Pose2d; 
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.networktables.StructSubscriber;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Vision;
import frc.robot.Constants.Vision.CameraDistortion;
import frc.robot.Constants.Vision.CameraIntrinsics;
import frc.robot.Robot;

public class ObjectDetection extends SubsystemBase {
  public PhotonCamera detector;

  public Pose2d closest_pose;
  private Pose2d robot_pose;

  StructPublisher<Pose3d> adv_closest_pub;
  StructSubscriber<Pose2d> adv_robot_pose_sub;

  StructArrayPublisher<Translation2d> adv_corners_pub; // jank

  Transform2d robot_to_camera;

  MatOfPoint2f temp_mat, dest_mat;

  Mat camera_instrinsics_mat, dist_coeffs_mat;

  // TODO: potentially unneeded?
  /** in meters */
  double closest_distance;

  // Load OpenCV
  static {
    System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
  }

  /** Creates a new ObjectDetection. */
  public ObjectDetection() {
    if (!Robot.isSimulation()) {
      detector = new PhotonCamera("OV9782");
    }
    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable adv_vision = inst.getTable("adv_vision");
    NetworkTable adv_swerve = inst.getTable("adv_swerve");
    adv_closest_pub = adv_vision.getStructTopic("Closest", Pose3d.struct).publish();
    adv_corners_pub = adv_vision.getStructArrayTopic("Target Corners", Translation2d.struct).publish();

    adv_robot_pose_sub = adv_swerve.getStructTopic("Pose", Pose2d.struct).subscribe(new Pose2d(),
        PubSubOption.keepDuplicates(true));

    robot_to_camera = new Transform2d(Units.inchesToMeters(2.5), Units.inchesToMeters(11), Rotation2d.fromDegrees(-20));

    // Initialize mats as member fields because creating mats is expensive
    temp_mat = new MatOfPoint2f();
    dest_mat = new MatOfPoint2f();

    // Camera intrinsics matrix
    camera_instrinsics_mat = Mat.zeros(3, 3, CvType.CV_64F);
    camera_instrinsics_mat.put(0, 0, CameraIntrinsics.f_x);
    camera_instrinsics_mat.put(0, 2, CameraIntrinsics.c_x);
    camera_instrinsics_mat.put(1, 1, CameraIntrinsics.f_y);
    camera_instrinsics_mat.put(1, 2, CameraIntrinsics.f_y);
    camera_instrinsics_mat.put(2, 2, 1);

    
    dist_coeffs_mat = new Mat(1, 8, CvType.CV_64F);
    dist_coeffs_mat.put(0, 0, 
        CameraDistortion.k1, 
        CameraDistortion.k2, 
        CameraDistortion.k3, 
        CameraDistortion.k4, 
        CameraDistortion.k5, 
        CameraDistortion.k6, 
        CameraDistortion.k7, 
        CameraDistortion.k8);
  }

  public Pose2d getObjectPose(Pose2d robot_pose, double distance, double angle_to_target) {
    // Robot's global heading
    double robot_theta = robot_pose.getRotation().getRadians();

    // Convert distance + angle to object coords in the field frame:
    // double global_angle = robot_theta + angle_to_target; // field heading to the target
    // double x = robot_pose.getX() + (distance * Math.cos(global_angle));
    // double y = robot_pose.getY() + (distance * Math.sin(global_angle));

    Transform2d camera_to_target = new Transform2d(
        distance * Math.cos(angle_to_target),
        distance * Math.sin(angle_to_target),
        new Rotation2d());

    Pose2d final_pose = robot_pose.transformBy(robot_to_camera).transformBy(camera_to_target);

    // If you don't know the object's actual heading, just store globalAngle or 0.
    // We'll just store the object's "facing" as globalAngle here for demonstration.
    // return new Pose2d(x, y, new Rotation2d(global_angle));
    return final_pose;
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
   * @param x_fov       in radians
   * @param x_fov       in radians
   * @return
   */
  private double getFocalLength(int image_width, double x_fov) {
    return (image_width / 2.0d) / Math.tan(x_fov / 2.0d);
  }


  /**
   * @param target_width  in pixels
   * @param target_width  in pixels
   * @param target_height in pixels
   * @param target_x      in pixels
   * @param target_x      in pixels
   * @return [distance, offset] in meters
   */
  public double[] getDistance(double target_width, double target_height, double target_x) {
    // 1) Distance: d = (focalLengthPx * realWidthMeters) / boundingBoxWidthPx
    double distance;
    // in pixels
    // double focal_length = getFocalLength(Vision.SCREEN_WIDTH, Vision.CAMERA_X_FOV);

    distance = 261 / (Math.max(target_width, target_height)) - 0.04 + 0.35;
    SmartDashboard.putNumber("target_height", target_height);

    // adjust with offset
    // distance += 2.5 * 0.0254;
    double image_center = Vision.SCREEN_WIDTH / 2.0;
    double fraction_off_center = (target_x - image_center) / image_center;

    // Angle from camera centerline
    double offset_angle_camera = fraction_off_center * (Vision.CAMERA_X_FOV / 2.0);

    // Because camera is turned 20° clockwise, subtract 20° to get the robot-relative angle
    double cameraMountAngleRad = Math.toRadians(-20.0);
    double offset_angle_robot = offset_angle_camera - cameraMountAngleRad;

    // Horizontal offset in meters from the robot’s forward axis
    double horizontal_offset = distance * Math.sin(offset_angle_camera);
    // distance = distance * Math.sin(offset_angle_camera);

    return new double[] { distance, horizontal_offset };
  }

  // public Pose2d getClosestTarget(ArrayList<Target> targets) {
  //   double min_dist = Double.MAX_VALUE;
  //   Target result = new Target();
  //   for (Target target : targets) {
  //     if (target.direct_distance < min_dist) {
  //       min_dist = target.direct_distance;
  //       result = target;
  //     }
  //   }
  //   double angle_to_target = Math.atan(result.offset / result.distance);
  //   return getObjectPose3D(robot_pose, result.distance, angle_to_target);
  // }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // closestDistance = getDistance(413.0, 2.4, 413.0);

    robot_pose = adv_robot_pose_sub.get();

    if (Robot.isSimulation()) {
      closest_pose = new Pose2d(1, 7, new Rotation2d(Math.PI));
      adv_closest_pub.set(new Pose3d(closest_pose));
      return;
    }

    var result = detector.getLatestResult();
    // var result = detector.getAllUnreadResults();
    List<PhotonTrackedTarget> targets = result.getTargets();
    ArrayList<Pose2d> target_poses = new ArrayList<Pose2d>();
    ArrayList<Target> target_distances = new ArrayList<Target>();

    for (PhotonTrackedTarget target : targets) {

      double minX = Double.MAX_VALUE;
      double minY = Double.MAX_VALUE;
      double maxX = Double.MIN_VALUE;
      double maxY = Double.MIN_VALUE;

      Point[] corner_points = new Point[4];

      for (int i = 0; i < 4; i++) {
        TargetCorner c = target.getMinAreaRectCorners().get(i);
        corner_points[i] = new Point(c.x, c.y);
      }

      temp_mat.fromArray(corner_points);

      for (TargetCorner corner : target.getMinAreaRectCorners()) {
        minX = Math.min(minX, corner.x);
        maxX = Math.max(maxX, corner.x);
        minY = Math.min(minY, corner.y);
        maxY = Math.max(maxY, corner.y);
      }

      SmartDashboard.putNumber("minXo", minX);
      SmartDashboard.putNumber("maxXo", maxX);
      SmartDashboard.putNumber("minYo", minY);
      SmartDashboard.putNumber("maxYo", maxY);

      SmartDashboard.putNumber("heighto", maxY - minY);

      SmartDashboard.putNumber("k1", dist_coeffs_mat.get(0, 0)[0]);

      Calib3d.undistortImagePoints(
        temp_mat,
        dest_mat,
        camera_instrinsics_mat,
        dist_coeffs_mat);

      for (Point corner : dest_mat.toArray()) {
        minX = Math.min(minX, corner.x);
        maxX = Math.max(maxX, corner.x);
        minY = Math.min(minY, corner.y);
        maxY = Math.max(maxY, corner.y);
      }

      SmartDashboard.putNumber("minX", minX);
      SmartDashboard.putNumber("maxX", maxX);
      SmartDashboard.putNumber("minY", minY);
      SmartDashboard.putNumber("maxY", maxY);

      double midpoint = (maxX + minX)/2;
      double width = maxX - minX;
      double height = maxY - minY;

      SmartDashboard.putNumber("height", height);

      double[] temp_dist = getDistance(width, height, midpoint);
      double distance = temp_dist[0];
      double offset = -temp_dist[1] + 0.24;

      double angle_to_target = Math.atan(offset / distance);
      double direct_distance = Math.sqrt(Math.pow(distance, 2) + Math.pow(offset, 2));

      target_poses.add(getObjectPose(robot_pose, distance, angle_to_target));
      target_distances.add(new Target(distance, offset, direct_distance));
    }

    if (target_poses.size() >= 1) {
      adv_closest_pub.set(new Pose3d(target_poses.get(0)));
      closest_pose = target_poses.get(0);
    }
  }
}