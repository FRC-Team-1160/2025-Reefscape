package frc.robot.Subsystems.Vision;
import java.util.HashSet;
import java.util.List;
import java.util.Set;
import java.util.function.BiConsumer;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import frc.robot.Robot;
import frc.robot.Constants.VisionConstants.CameraTransforms.LeftCamera;
import frc.robot.Constants.VisionConstants.CameraTransforms.RightCamera;

public class Vision {

    public ObjectDetection m_object_detection;

    private record CameraResults(Pose2d pose, Set<Integer> ids) {}

    StructArrayPublisher<Pose2d> adv_poses_pub;
    StructArrayPublisher<Pose3d> adv_tags_pub;

    PhotonCamera camera_left, camera_right;
    PhotonPoseEstimator pose_estimator_left, pose_estimator_right;
    VisionPoseCache pose_cache_left, pose_cache_right;

    BiConsumer<Pose2d, Double> update_pose_estimator;
    Supplier<Pose2d> robot_pose_supplier;

    /** Creates a new Vision. */
    public Vision(BiConsumer<Pose2d, Double> update_pose_estimator, Supplier<Pose2d> robot_pose_supplier) {

        this.robot_pose_supplier = robot_pose_supplier;
        this.update_pose_estimator = update_pose_estimator;

        m_object_detection = new ObjectDetection(robot_pose_supplier);

        if (Robot.isSimulation()) return; // Exit if in simulation

        NetworkTable adv_vision = NetworkTableInstance.getDefault().getTable("adv_vision");
        adv_poses_pub = adv_vision.getStructArrayTopic("Poses", Pose2d.struct).publish();
        adv_tags_pub = adv_vision.getStructArrayTopic("Used Tags", Pose3d.struct).publish();

        camera_left = new PhotonCamera("OV9782");
        camera_right = new PhotonCamera("OV9281");

        AprilTagFieldLayout APRILTAG_FIELD_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

        pose_estimator_left = new PhotonPoseEstimator(
            APRILTAG_FIELD_LAYOUT, 
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, 
            new Transform3d(LeftCamera.X, LeftCamera.Y, LeftCamera.Z,
                new Rotation3d(LeftCamera.ROLL, LeftCamera.PITCH, LeftCamera.YAW)
            )
        );
        pose_estimator_left.setReferencePose(new Pose2d());

        pose_estimator_right = new PhotonPoseEstimator(
            APRILTAG_FIELD_LAYOUT, 
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, 
            new Transform3d(RightCamera.X, RightCamera.Y, RightCamera.Z,
                new Rotation3d(RightCamera.ROLL, RightCamera.PITCH, RightCamera.YAW)
            )
        );
        pose_estimator_right.setReferencePose(new Pose2d());

        pose_cache_left = new VisionPoseCache();
        pose_cache_right = new VisionPoseCache();

    }

    public CameraResults readPhotonResults(PhotonCamera camera, PhotonPoseEstimator pose_estimator, VisionPoseCache cache) {

        Pose2d pose = null;
        List<PhotonPipelineResult> photon_results = camera_left.getAllUnreadResults();
        Set<Integer> fiducials = new HashSet<Integer>();

        for (PhotonPipelineResult result : photon_results) {
            // Exit if result has no targets or pose is empty
            if (!result.hasTargets()) continue;
            var opt_pose = pose_estimator.update(result);
            if (opt_pose.isEmpty()) continue;

            EstimatedRobotPose estimate = opt_pose.get();

            pose = estimate.estimatedPose.toPose2d();
            // cache pose for fluctuation calculations
            cache.addPose(
                pose,
                robot_pose_supplier.get(), 
                estimate.timestampSeconds);
            // store ambiguity and read used apriltag IDs
            if (result.multitagResult.isPresent()) {
                cache.updateAmbiguity(result.getMultiTagResult().get().estimatedPose.ambiguity);
                for (Short id : result.getMultiTagResult().get().fiducialIDsUsed) {
                    fiducials.add(id.intValue());
                }
            } else {
                cache.updateAmbiguity(result.getBestTarget().poseAmbiguity);
                fiducials.add(result.getBestTarget().fiducialId);
            }
        }

        return new CameraResults(pose, fiducials);
    }

    public void update() {
        // Object detection is still updated during simulation
        m_object_detection.update(); 

        if (Robot.isSimulation()) return;
        
    }
}
