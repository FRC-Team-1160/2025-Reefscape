package frc.robot.Subsystems.Vision;

import java.util.ArrayList;
import java.util.Collection;
import java.util.HashSet;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.LimelightHelpers.RawFiducial;
import frc.robot.Robot;
import frc.robot.SubsystemManager;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.VisionConstants.CameraTransforms.LeftCamera;
import frc.robot.Constants.VisionConstants.CameraTransforms.RightCamera;

public class Vision {

    public static final Vision instance = new Vision();

    // Convenience class for organizing readings from cameras
    private record CameraResults(Pose2d pose, Collection<Integer> ids) {}

    StructArrayPublisher<Pose2d> adv_poses_pub;
    StructArrayPublisher<Pose3d> adv_tags_pub;

    PhotonCamera camera_left, camera_right;
    PhotonPoseEstimator pose_estimator_left, pose_estimator_right;
    VisionPoseCache pose_cache_left, pose_cache_right, pose_cache_limelight;

    Pose3d[] apriltags_map;

    @AutoLogOutput
    CameraMode m_camera_mode;

    Pose2d robot_pose;

    public enum CameraMode {
        kDefault(0, 1),
        kStereoAprilTag(0, 0),
        kStereoAlgae(1, 1);
        
        public int left, right;
        private CameraMode(int left, int right) {
            this.left = left;
            this.right = right;
        }
    }

    /** Creates a new Vision. */
    private Vision() {

        pose_cache_left = new VisionPoseCache();
        pose_cache_right = new VisionPoseCache();
        pose_cache_limelight = new VisionPoseCache();

        if (Robot.isSimulation()) return; // Exit if in simulation

        NetworkTable adv_vision = NetworkTableInstance.getDefault().getTable("adv_vision");
        adv_poses_pub = adv_vision.getStructArrayTopic("Poses", Pose2d.struct).publish();
        adv_tags_pub = adv_vision.getStructArrayTopic("Used Tags", Pose3d.struct).publish();

        camera_left = new PhotonCamera("OV9281");
        camera_right = new PhotonCamera("OV9782");

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

        LimelightHelpers.setPipelineIndex("", 0);

        // apriltag sightline stuff
        apriltags_map = new Pose3d[VisionConstants.APRILTAG_POSES.length + 1];
        apriltags_map[0] = null;
        for(int i = 0; i < VisionConstants.APRILTAG_POSES.length; i++) {
            apriltags_map[i + 1] = new Pose3d(
                VisionConstants.APRILTAG_POSES[i][0],
                VisionConstants.APRILTAG_POSES[i][1],
                VisionConstants.APRILTAG_POSES[i][2],
                new Rotation3d(
                    0,
                    VisionConstants.APRILTAG_POSES[i][4],
                    VisionConstants.APRILTAG_POSES[i][3]
                )
            );
        }

        setCameraPipelines(CameraMode.kDefault);
    }

    public Optional<CameraResults> readPhotonResults(
        PhotonCamera camera, 
        PhotonPoseEstimator photon_pose_estimator, 
        VisionPoseCache cache) {

        Pose2d pose = null;
        // Using a set prevents repetition
        ArrayList<Integer> fiducials = new ArrayList<Integer>();
        List<PhotonPipelineResult> photon_results = camera.getAllUnreadResults();

        // 2025: getAllUnreadResults() returns all estimations since the last check
        for (PhotonPipelineResult result : photon_results) {
            // Move on to next result if this has no targets or pose is empty
            if (!result.hasTargets()) continue;
            var opt_pose = photon_pose_estimator.update(result);
            if (opt_pose.isEmpty()) continue;

            EstimatedRobotPose estimate = opt_pose.get();
            pose = estimate.estimatedPose.toPose2d();

            // Cache pose for fluctuation calculations
            cache.addPose(pose, SubsystemManager.instance.getPoseEstimate(), estimate.timestampSeconds);

            // Check if result has multiple tags; store ambiguity and read used apriltag IDs
            if (result.multitagResult.isPresent()) {
                cache.last_ambiguity = result.multitagResult.get().estimatedPose.ambiguity;
                for (Short id : result.multitagResult.get().fiducialIDsUsed) {
                    fiducials.add(id.intValue());
                }
            } else {
                cache.last_ambiguity = result.getBestTarget().poseAmbiguity;
                fiducials.add(result.getBestTarget().fiducialId);
            }
            // Directly add our vision estimates to the PoseEstimator; there are too many arguments for a Consumer
            if (pose != null) SubsystemManager.instance.pose_estimator.addVisionMeasurement(
                pose, 
                estimate.timestampSeconds, 
                cache.getWeightedStdevs());
        }
        return pose == null ? Optional.empty() : Optional.of(new CameraResults(pose, fiducials));
    }

    public Optional<CameraResults> readLimelightResults() {
        // Limelight MegaTag2 uses the robot yaw
        LimelightHelpers.SetRobotOrientation(
            "", robot_pose.getRotation().getDegrees(), 0, 0, 0, 0, 0);
        // Run MegaTag 2; always use blue origin
        PoseEstimate mt2_estimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("");

        Logger.recordOutput("Vision/Limelight Pose", mt2_estimate.pose);

        if (mt2_estimate == null) return Optional.empty();
        if (mt2_estimate.tagCount == 0) return Optional.empty();
        
        pose_cache_limelight.addPose(
            mt2_estimate.pose, 
            SubsystemManager.instance.getPoseEstimate(), 
            mt2_estimate.timestampSeconds);

        SubsystemManager.instance.pose_estimator.addVisionMeasurement(
            mt2_estimate.pose, 
            mt2_estimate.timestampSeconds,
            pose_cache_limelight.getWeightedStdevs());

        ArrayList<Integer> ids = new ArrayList<Integer>();

        for (RawFiducial tag : mt2_estimate.rawFiducials) {
            ids.add(tag.id);
        }

        return Optional.of(new CameraResults(mt2_estimate.pose, ids));
    }

    /**
     * Switch the pipelines of the two OV cameras.
     * @param mode The mode for the two cameras.
     */
    public void setCameraPipelines(CameraMode mode) {
        if (mode == m_camera_mode || Robot.isSimulation()) return;
        // Don't reset if already on this pipeline.
        if (camera_left.getPipelineIndex() != mode.left) camera_left.setPipelineIndex(mode.left);
        if (camera_right.getPipelineIndex() != mode.right) camera_right.setPipelineIndex(mode.right);
        m_camera_mode = mode;
    }

    public CameraMode getCameraMode() {
        return m_camera_mode;
    }

    // Vision is marked as a non-subsystem to allow it to run before subsystem periodic methods
    public void update() {
        robot_pose = SubsystemManager.instance.getPoseEstimate();
        // Object detection is still updated during simulation
        ObjectDetection.instance.update();

        if (Robot.isSimulation()) return;

        List<Pose2d> vision_poses = new ArrayList<Pose2d>();
        HashSet<Integer> used_ids = new HashSet<Integer>();

        readPhotonResults(camera_left, pose_estimator_left, pose_cache_left).ifPresent(
            result -> {
                vision_poses.add(result.pose);
                used_ids.addAll(result.ids);
            }
        );

        readPhotonResults(camera_right, pose_estimator_right, pose_cache_right).ifPresent(
            result -> {
                vision_poses.add(result.pose);
                used_ids.addAll(result.ids);
            }
        );

        readLimelightResults().ifPresent(
            result -> {
                vision_poses.add(result.pose);
                used_ids.addAll(used_ids);
            }
        );

        Pose3d[] used_tag_poses = new Pose3d[used_ids.size()];

        int i = 0;
        for (int id : used_ids) {
            used_tag_poses[i++] = apriltags_map[id];
        }

        adv_poses_pub.set(vision_poses.toArray(Pose2d[]::new));
        adv_tags_pub.set(used_tag_poses);
    }
}
