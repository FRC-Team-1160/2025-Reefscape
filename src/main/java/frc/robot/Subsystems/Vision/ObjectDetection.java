package frc.robot.Subsystems.Vision;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.VisionConstants.AlgaeParams;
import frc.robot.Constants.VisionConstants.EstimationParameters;
import frc.robot.Robot;
import frc.robot.RobotUtils;

public class ObjectDetection {
    /** The OV9782 instance. */
    public PhotonCamera camera;
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

    /** Creates a new ObjectDetection. */
    public ObjectDetection(Supplier<Pose2d> robot_pose_supplier) {
        if (Robot.isReal()) {
            camera = new PhotonCamera("OV9782");
        }

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
    }

    /**
     * Uses an inverse regression function to calculate the normal distance of the target.
     * @param width The width of the target's bounding box in pixels.
     * @param height The height of the target's bounding box in pixels.
     * @return The normal distance of the target in meters.
     */
    public double getNormalDistance(double width, double height) {
        return EstimationParameters.a / (Math.max(width, height)) - EstimationParameters.b;
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
    public Pose2d getTargetPose(double width, double height, int x_center) {
        double n_dist = getNormalDistance(width, height);
        Transform2d robot_transform = new Transform2d(
            robot_pose.getTranslation(),
            robot_pose.getRotation());
        Transform2d target_transform = new Transform2d(
            n_dist,
            getLateralDistance(n_dist, x_center),
            new Rotation2d()
        );
        return new Pose2d(
            robot_transform.plus(target_transform).getTranslation(), 
            target_transform.getTranslation().getAngle());
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

    /** @hidden */
    public void update() {

        robot_pose = robot_pose_supplier.get();
        // Publish and exit now if robot is in sim; else, publish later
        if (Robot.isSimulation()) {
            publishAdv();
            return;
        }
        if (camera.getPipelineIndex() != 0) {
            publishAdv();
            return;
        }
        // Increase marked counter
        for (VisionTarget t : tracked_targets) {
            if (Math.abs(t.getAngle(robot_pose).getRadians()) < AlgaeParams.EXPECTED_RANGE / 2)  {
                t.marked++;
            } else {
                t.marked = 0;
            }
        }

        // DEPRECATED, REPLACE WITH getAllUnreadResults()
        List<PhotonTrackedTarget> targets = camera.getLatestResult().getTargets();

        for (PhotonTrackedTarget target : targets) {

            double minX = Double.MAX_VALUE;
            double minY = Double.MAX_VALUE;
            double maxX = Double.MIN_VALUE;
            double maxY = Double.MIN_VALUE;

            // Corners are returned in an unordered list; get extrema
            for (TargetCorner corner : target.getMinAreaRectCorners()) {
                minX = Math.min(minX, corner.x);
                maxX = Math.max(maxX, corner.x);
                minY = Math.min(minY, corner.y);
                maxY = Math.max(maxY, corner.y);
            }

            int tol = AlgaeParams.EDGE_TOLERANCE;

            Pose2d target_pose = getTargetPose(maxX - minX, maxY - minY, (int)(maxX + minX)/2);
            // Keep track of whether current target has been matched with existing object
            boolean matched = false; 

            // Check that object isnt cut off
            if (minX > tol && maxX < VisionConstants.SCREEN_WIDTH - tol 
                || minY > tol && maxX < VisionConstants.SCREEN_HEIGHT - tol) {
                // Attempt to match the current pose with each stored target
                for (VisionTarget t : tracked_targets) {
                    if (matched = t.match(target_pose, robot_pose, true)) break;
                }
                // If current target hasnt been matched, start tracking as new target
                if (!matched) tracked_targets.add(new VisionTarget(target_pose)); 

            } else {
                // If target bounding box is cut off at camera edges
                for (VisionTarget t : tracked_targets) {
                    if (t.getAngle(robot_pose.getTranslation()).minus(target_pose.getRotation()).getDegrees() < 10) {
                        // Confirm that the target still exists, but calculated position may be inaccurate
                        t.resetTimer(); 
                        break;
                    }
                }
            }

        }
        // If target was expected in fov and not seen, or has not been seen in too long, stop tracking it
        tracked_targets.removeIf(target -> 
            target.marked >= AlgaeParams.DETECTION_LIMIT 
                || target.timer.get() > AlgaeParams.TRACKING_TIMEOUT);
        
        publishAdv();

    }
}