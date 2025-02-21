package frc.robot.Subsystems.Vision;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;
import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Optional;
import java.util.Set;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.LimelightHelpers.RawFiducial;
import frc.robot.Robot;
import frc.robot.RobotUtils;
import frc.robot.Constants.VisionConstants.CameraTransforms.LeftCamera;
import frc.robot.Constants.VisionConstants.CameraTransforms.RightCamera;

public class Vision {

    public ObjectDetection m_object_detection;
    // Convenience class for organizing readings from cameras
    private record CameraResults(Pose2d pose, Set<Integer> ids) {}

    StructArrayPublisher<Pose2d> adv_poses_pub;
    StructArrayPublisher<Pose3d> adv_tags_pub;

    PhotonCamera camera_left, camera_right;
    PhotonPoseEstimator pose_estimator_left, pose_estimator_right;
    VisionPoseCache pose_cache_left, pose_cache_right, pose_cache_limelight;
    // A REFERENCE to the SwerveDrivePoseEstimator contained in SubsystemManager so that vision can update with 3 args
    SwerveDrivePoseEstimator main_pose_estimator;
    Supplier<Pose2d> robot_pose_supplier;

    // apriltags sightline stuff
    HashMap<Integer, Pose3d>  apriltags_map;
    List<Pose3d> apriltag_poses;

    // Limelight stuff
    int count;
    double limelight_pose_timestamp;
    Pose2d limelight_pose;

    /** Creates a new Vision. */
    public Vision(SwerveDrivePoseEstimator main_pose_estimator, Supplier<Pose2d> robot_pose_supplier) {

        this.main_pose_estimator = main_pose_estimator;
        this.robot_pose_supplier = robot_pose_supplier;

        m_object_detection = new ObjectDetection(robot_pose_supplier);

        pose_cache_left = new VisionPoseCache();
        pose_cache_right = new VisionPoseCache();

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

        // apriltag sightline stuff
        apriltags_map = new HashMap<Integer, Pose3d>();
        apriltag_poses = new ArrayList<Pose3d>();
        for(double[] i:Constants.VisionConstants.tags_map){
            apriltags_map.put((int)i[0], new Pose3d(i[1] * 0.0254, i[2] * 0.0254, i[3] * 0.0254, new Rotation3d(0, i[5], i[4])));
        }
    }

    public Optional<CameraResults> readPhotonResults(
        PhotonCamera camera, 
        PhotonPoseEstimator photon_pose_estimator, 
        VisionPoseCache cache) {

        Pose2d pose = null;
        // Using a set prevents repetition
        Set<Integer> fiducials = new HashSet<Integer>();
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
            cache.addPose(
                pose,
                robot_pose_supplier.get(), 
                estimate.timestampSeconds);

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
            if (pose != null) main_pose_estimator.addVisionMeasurement(
                pose, 
                estimate.timestampSeconds, 
                cache.getWeightedStdevs());
        }
        return pose == null ? Optional.empty() : Optional.of(new CameraResults(pose, fiducials));
    }

    // Vision is marked as a non-subsystem to allow it to run before subsystem periodic methods
    public void update() {
        // Object detection is still updated during simulation
        m_object_detection.update();

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

        // limelight stuff
        // smart cropping:
        LimelightResults limelightResult = LimelightHelpers.getLatestResults("");
        if(limelightResult.valid){
            // non-dynamic
            LimelightHelpers.setPipelineIndex("", 1);
            count = 0;
        }
        if(!limelightResult.valid && count >=25){
            LimelightHelpers.setPipelineIndex("", 0);
        }


        if (limelightResult != null && limelightResult.valid){
            limelight_pose_timestamp = limelightResult.timestamp_LIMELIGHT_publish;
            if (RobotUtils.isRedAlliance()){
                limelight_pose = LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2("").pose;
                limelight_pose_timestamp = LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2("").timestampSeconds;
            } else {
                limelight_pose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("").pose;
                limelight_pose_timestamp = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("").timestampSeconds;
            }
            pose_cache_limelight.addPose(limelight_pose, main_pose_estimator.getEstimatedPosition(), limelight_pose_timestamp);

             RawFiducial[] fiducials = LimelightHelpers.getRawFiducials("");
            // if (fiducials != null){
            //     limelight_pose_stddev.times(fiducials[0].ambiguity);
            // }
            // store it in apriltag 
            for (RawFiducial fudicial:fiducials){
                apriltag_poses.add(apriltags_map.get(fudicial));
            }
        }
        if ( pose_cache_limelight != null){
            main_pose_estimator.addVisionMeasurement(limelight_pose, limelight_pose_timestamp, pose_cache_limelight.getStdevs());
        }

        adv_poses_pub.set(vision_poses.toArray(Pose2d[]::new));
    }
}
