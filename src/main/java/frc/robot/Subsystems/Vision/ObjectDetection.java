package frc.robot.Subsystems.Vision;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.function.Supplier;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.fasterxml.jackson.core.util.DefaultPrettyPrinter.FixedSpaceIndenter;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;

import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d; 
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.Kinematics;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;

import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.VisionConstants.AlgaeParams;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.VisionConstants.EstimationParams;
import frc.robot.Constants.VisionConstants.CameraTransforms.LeftCamera;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants.CameraTransforms.RightCamera;
import frc.robot.Constants.VisionConstants.CameraDistortion;
import frc.robot.Constants.VisionConstants.CameraIntrinsics;
import frc.robot.Subsystems.DriveTrain.DriveTrain;
import frc.robot.Robot;
import frc.robot.RobotUtils;

public class ObjectDetection {
    /** The OV9782 instance. */
    private PhotonCamera left_camera;
    /** The robot-to-camera transform. */
    private Transform2d left_camera_transform;
    /** The current robot pose. */
    public Pose2d robot_pose;
    /** Supplier for the robot pose. */
    Supplier<Pose2d> robot_pose_supplier;
    /** AdvantageScope publisher. */
    StructPublisher<Pose2d> adv_closest_pub;
    /** AdvantageScope publisher. */
    StructArrayPublisher<Pose3d> adv_tracked_pub;
    /** List of targets that are being tracked. */
    public List<VisionTarget> tracked_targets;

    MatOfPoint2f temp_mat, dest_mat;

    Mat camera_instrinsics_mat, dist_coeffs_mat;

    /** Creates a new ObjectDetection. */
    public ObjectDetection(Supplier<Pose2d> robot_pose_supplier) {
        if (Robot.isReal()) {
            left_camera = new PhotonCamera("OV9782");
        }

        left_camera_transform = new Transform2d(LeftCamera.X, LeftCamera.Y, Rotation2d.fromRadians(LeftCamera.YAW));

        NetworkTable adv_vision = NetworkTableInstance.getDefault().getTable("adv_vision");

        adv_closest_pub = adv_vision.getStructTopic("Closest", Pose2d.struct).publish();
        adv_tracked_pub = adv_vision.getStructArrayTopic("Tracked", Pose3d.struct).publish();

        this.robot_pose_supplier = robot_pose_supplier;

        tracked_targets = new ArrayList<VisionTarget>();

        // Generate 3 random vision targets for testing
        if (Robot.isSimulation()) {
            for (int i = 0; i < 3; i++) {
                tracked_targets.add(new VisionTarget(
                    new Translation2d(Math.random() * 12.0 + 2.0, Math.random() * 4.0 + 2.0)
                ));
            }
        }

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

    /**
     * Uses an inverse regression function to calculate the normal distance of the target.
     * @param width The width of the target's bounding box in pixels.
     * @param height The height of the target's bounding box in pixels.
     * @return The normal distance of the target in meters.
     */
    public double getNormalDistance(double width, double height) {
        return EstimationParams.a / (Math.max(width, height)) - EstimationParams.b;
    }

    /**
     * Calculates the distance from the camera horizontal ray to the target center.
     * @param normal_distance The normal distance to the target in meters.
     * @param x_center The horizontal center of the target's bounding box in pixels.
     * @return The lateral distance to the target in meters.
     */
    public double getLateralDistance(double normal_distance, int x_center) {
        return normal_distance 
            * Math.tan(VisionConstants.CAMERA_X_FOV/2) 
            * (x_center / VisionConstants.SCREEN_WIDTH - 0.5);
    }

    /**
     * Calculates the diagonal distance from the camera to the target.
     * @param width The width of the target's bounding box in pixels.
     * @param height The height of the target's bounding box in pixels.
     * @param x_center The normal distance of the target in meters.
     * @return The distance to the target in meters.
     */
    public double getDistance(double width, double height, int x_center) {
        double n_dist = getNormalDistance(width, height);
        return RobotUtils.hypot(n_dist, getLateralDistance(n_dist, x_center));
    }

    /**
     * Calculates the target's field-relative pose.
     * @param width The width of the target's bounding box in pixels.
     * @param height The height of the target's bounding box in pixels.
     * @param x_center The normal distance of the target in meters.
     * @return The calculated pose.
     */
    public Pose2d getTargetPose(double width, double height, int x_center, Transform2d camera_transform) {
        double n_dist = getNormalDistance(width, height);
        Transform2d robot_transform = new Transform2d(
            robot_pose.getTranslation(),
            robot_pose.getRotation());
        Transform2d target_transform = new Transform2d(
            n_dist,
            getLateralDistance(n_dist, x_center),
            new Rotation2d()
        );
        target_transform = robot_transform.plus(camera_transform).plus(target_transform);
        return new Pose2d(target_transform.getTranslation(), target_transform.getRotation());
    }


    /**
     * Returns the closest target to the robot. Accounts for robot orientation.
     * @return The best target object.
     */
    public Optional<VisionTarget> getClosestTarget() {
        robot_pose = robot_pose_supplier.get();
        double min_dist = AlgaeParams.MAX_TRACKING_DISTANCE; // Only return if closest target is within range
        Optional<VisionTarget> closest = Optional.empty();
        for (VisionTarget target : tracked_targets) {
            if (target.getDistance(robot_pose) < min_dist) {
                // Take into account both distance and rotational distance
                min_dist = target.getDistance(robot_pose) + target.getAngle(robot_pose).getRotations();
                closest = Optional.of(target);
            }
        }
        return closest;
    }
    
    /** @hidden */
    public void publishAdv() {
        getClosestTarget().ifPresent(
            target -> adv_closest_pub.set(target.getPose()));

        Pose3d[] target_poses = new Pose3d[tracked_targets.size()];
        for (int i = 0; i < tracked_targets.size(); i++) {
            target_poses[i] = new Pose3d(tracked_targets.get(i).getPose());
        }
        adv_tracked_pub.set(target_poses);
    }

    public void updateTrackedObjects(PhotonCamera camera, Transform2d camera_transform) {

        List<PhotonTrackedTarget> targets = camera.getLatestResult().getTargets();
        // Make a temporary list to keep track of which targets have been matched
        List<VisionTarget> temp_tracked_targets = tracked_targets;

        // calculate poses for each target
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

            // Corners are returned in an unordered list; get extrema
                for (TargetCorner corner : target.getMinAreaRectCorners()) {
                    minX = Math.min(minX, corner.x);
                    maxX = Math.max(maxX, corner.x);
                    minY = Math.min(minY, corner.y);
                    maxY = Math.max(maxY, corner.y);
                }

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

            int tol = AlgaeParams.EDGE_TOLERANCE;

            Pose2d target_pose = getTargetPose(maxX - minX, maxY - minY, (int)(maxX + minX)/2, camera_transform);
            // Keep track of whether current target has been matched with existing object
            boolean matched = false; 

            // Check that object isnt cut off
            if (minX > tol && maxX < VisionConstants.SCREEN_WIDTH - tol 
                || minY > tol && maxX < VisionConstants.SCREEN_HEIGHT - tol) {
                // Attempt to match the current pose with each stored target
                for (VisionTarget t : temp_tracked_targets) {
                    if (matched = t.match(target_pose, robot_pose, true)) {
                        // Remove targets from the temporary list if they have been matched
                        temp_tracked_targets.remove(t);
                        break;
                    }
                }
                // If current target hasnt been matched, start tracking as new target
                if (!matched) tracked_targets.add(new VisionTarget(target_pose)); 

            } else {
                // If target bounding box is cut off at camera edges
                for (VisionTarget tracked : temp_tracked_targets) {
                    if (tracked.inAngularTolerance(target_pose, robot_pose)) {
                        // Confirm that the target still exists, but calculated position may be inaccurate
                        tracked.resetTimer(); 
                        // Remove targets from the temporary list of they have been matched
                        temp_tracked_targets.remove(tracked);
                        break;
                    }
                }
            }

        }
        // If target was expected in fov and not seen, or has not been seen in too long, stop tracking it
        tracked_targets.removeIf(target -> 
            target.timeout >= AlgaeParams.DETECTION_LIMIT 
                || target.timer.get() > AlgaeParams.TRACKING_TIMEOUT);

    }

    /** @hidden */
    public void update() {

        robot_pose = robot_pose_supplier.get();
        // Publish and exit now if robot is in sim; else, publish later
        if (Robot.isSimulation()) {
            publishAdv();
            return;
        }
        if (left_camera.getPipelineIndex() != 0) {
            publishAdv();
            return;
        }

        // Increase timeouts
        for (VisionTarget t : tracked_targets) {
            if (Math.abs(t.getAngle(robot_pose).getRadians()) < AlgaeParams.EXPECTED_RANGE / 2) t.timeout++;
            else t.timeout = 0;
        }

        updateTrackedObjects(left_camera, left_camera_transform);

        // DEPRECATED, REPLACE WITH getAllUnreadResults()
        
        publishAdv();

    }
}