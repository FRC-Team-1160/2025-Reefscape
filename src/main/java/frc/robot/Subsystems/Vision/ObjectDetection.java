package frc.robot.Subsystems.Vision;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;
import java.util.stream.DoubleStream;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;
import org.littletonrobotics.junction.Logger;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;

import edu.wpi.first.math.geometry.Pose2d; 
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;

import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.VisionConstants.AlgaeParams;
import frc.robot.Constants.VisionConstants.EstimationParams;
import frc.robot.Constants.VisionConstants.CameraTransforms.LeftCamera;
import frc.robot.Constants.VisionConstants.CameraTransforms.RightCamera;
import frc.robot.Constants.VisionConstants.CameraDistortion;
import frc.robot.Constants.VisionConstants.CameraIntrinsics;
import frc.robot.Robot;
import frc.robot.RobotUtils;
import frc.robot.SubsystemManager;

public class ObjectDetection {

    public static final ObjectDetection instance = new ObjectDetection();
    /** The camera instance. */
    private PhotonCamera camera_left, camera_right;
    /** The robot-to-camera transform. */
    private Transform2d camera_transform_left, camera_transform_right;
    /** The current robot pose. */
    public Pose2d robot_pose;
    /** List of targets that are being tracked. */
    public List<VisionTarget> tracked_targets;

    MatOfPoint2f temp_mat, dest_mat;
    // Load OpenCV
    static {
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
    }

    Mat camera_instrinsics_mat, dist_coeffs_mat;

    /** Creates a new ObjectDetection. */
    private ObjectDetection() {
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);

        if (Robot.isReal()) {
            // right one is color cam
            camera_right = new PhotonCamera("OV9782");
            camera_left = new PhotonCamera("OV9281");
        }

        camera_transform_left = new Transform2d(LeftCamera.X, LeftCamera.Y, Rotation2d.fromRadians(LeftCamera.YAW));
        camera_transform_right = new Transform2d(RightCamera.X, RightCamera.Y, Rotation2d.fromRadians(RightCamera.YAW));
        
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
            * Math.tan(VisionConstants.CAMERA_X_FOV/2) * 2
            * -((double) x_center / VisionConstants.SCREEN_WIDTH - 0.5);
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
        robot_pose = SubsystemManager.instance.getPoseEstimate();
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
            target -> Logger.recordOutput("ObjectDetection/Closest Target", target.getPose3d()));


        Pose3d[] target_poses = Arrays.stream(tracked_targets.toArray())
            .map(t -> ((VisionTarget) t).getPose3d()).toArray(Pose3d[]::new);

        Logger.recordOutput("ObjectDetection/Tracked Targets", target_poses);
    }

    public void updateTrackedObjects(PhotonPipelineResult pipeline_result, Transform2d camera_transform) {

        if (!pipeline_result.hasTargets()) return;

        // Make a temporary list to keep track of which targets have been matched
        List<VisionTarget> temp_tracked_targets = new ArrayList<VisionTarget>(tracked_targets);

        // calculate poses for each target
        for (PhotonTrackedTarget target : pipeline_result.getTargets()) {

            Point[] corner_points = Arrays.stream(target.getMinAreaRectCorners().toArray())
                .map(c -> new Point(((TargetCorner) c).x, ((TargetCorner) c).y)).toArray(Point[]::new);

            temp_mat.fromArray(corner_points);

            Calib3d.undistortImagePoints(temp_mat, dest_mat, camera_instrinsics_mat, dist_coeffs_mat);

            // Corners are returned in an unordered list; get extrema
            DoubleStream x_stream = Arrays.stream(dest_mat.toArray()).mapToDouble(corner -> corner.x);
            DoubleStream y_stream = Arrays.stream(dest_mat.toArray()).mapToDouble(corner -> corner.x);

            double max_x = x_stream.max().orElse(Double.NaN);
            double min_x = x_stream.min().orElse(Double.NaN);

            double width = max_x - min_x;
            double height = y_stream.max().orElse(Double.NaN) - y_stream.min().orElse(Double.NaN);

            Pose2d target_pose = getTargetPose(width, height, (int)(min_x + max_x)/2, camera_transform);
            // Keep track of whether current target has been matched with existing object
            boolean matched = false; 

            // Check that object isnt cut off using the height:width ratio
            if (Math.min(width / height, height / width) >= AlgaeParams.MIN_BOUNDING_RATIO) {
                // Attempt to match the current pose with each stored target
                for (VisionTarget t : temp_tracked_targets) {
                    if (matched = t.match(target_pose, robot_pose, target.objDetectConf / 2)) {
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

    }

    /** @hidden */
    public void update() {

        robot_pose = SubsystemManager.instance.getPoseEstimate();
        // Publish and exit now if robot is in sim; else, publish later
        if (Robot.isSimulation()) {
            publishAdv();
            return;
        }

        // Increase timeouts
        for (VisionTarget t : tracked_targets) {
            if (Math.abs(t.getAngle(robot_pose.transformBy(camera_transform_left)).getRadians()) < AlgaeParams.EXPECTED_RANGE / 2
                || t.getDistance(robot_pose) < 1) t.timeout++;
            else t.timeout = 0;
        }

        if (camera_left.getPipelineIndex() == 1)
            for (PhotonPipelineResult result : camera_left.getAllUnreadResults())
                updateTrackedObjects(result, camera_transform_left);
        
        if (camera_right.getPipelineIndex() == 1)
            for (PhotonPipelineResult result : camera_right.getAllUnreadResults())
                updateTrackedObjects(result, camera_transform_right);

        // If any target was expected in fov and not seen, or has not been seen in too long, stop tracking it
        tracked_targets.removeIf(target -> 
        target.timeout >= AlgaeParams.DETECTION_LIMIT 
            || target.timer.hasElapsed(AlgaeParams.TRACKING_TIMEOUT));
        
        publishAdv();

    }
}