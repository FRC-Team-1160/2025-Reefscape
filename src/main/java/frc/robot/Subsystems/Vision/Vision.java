package frc.robot.Subsystems.Vision;
import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.function.Supplier;

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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.LimelightHelpers.RawFiducial;
import frc.robot.Robot;
import frc.robot.Subsystems.DriveTrain.DriveTrain;
import frc.robot.Subsystems.DriveTrain.DriveTrainRealIO;

public class Vision extends SubsystemBase {
    public Pose2d pose;
    public Pose2d photon_pose;
    public Pose2d photon_pose2;
    public Pose2d limelight_pose;

    public double photon_pose_timestamp;
    public double photon_pose2_timestamp;
    public double limelight_pose_timestamp;

    public PoseCaching photon_pose_chaching;
    public PoseCaching photon_pose2_chaching;
    public PoseCaching limelight_pose_chaching;

    public Matrix<N3, N1> photon_pose_stddev;
    public Matrix<N3, N1> photon_pose2_stddev;
    public Matrix<N3, N1> limelight_pose_stddev;

    public PoseEstimator odom_estimator;

    public int count;

    // DriveTrain drive_train;

    StructPublisher<Pose2d> adv_pose_pub;
    StructArrayPublisher<Pose2d> adv_target_pub;
    StructPublisher<Pose2d> adv_tracked_pub;

    PhotonCamera photon_tag_camera;
    PhotonCamera photon_tag_camera2;
    PhotonPoseEstimator photon_pose_estimator;
    PhotonPoseEstimator photon_pose_estimator2;

    List<PhotonPipelineResult> photon_result;
    List<PhotonPipelineResult> photon_result2;

    Supplier<Pose2d> getOdomPose;
    Supplier<PoseEstimator> getPoseEstimator;

    StructArrayPublisher<Pose2d> vision_pose_array_pub;
    StructPublisher<Transform2d> diff_pub;

    HashMap<Integer, Pose3d>  apriltags_map;
    StructArrayPublisher<Pose3d> apriltags_array_pub;
    List<Pose3d> apriltag_poses;


    public boolean stero = true;

    public Vision(Supplier<Pose2d> getOdomPose, Supplier<PoseEstimator> getPoseEstimator) {
        this.getOdomPose = getOdomPose;
        this.getPoseEstimator = getPoseEstimator;

        photon_pose_chaching = new PoseCaching(0.25);
        photon_pose2_chaching = new PoseCaching(0.25);
        limelight_pose_chaching = new PoseCaching(0.25);

        apriltags_map = new HashMap<Integer, Pose3d>();
        apriltag_poses = new ArrayList<Pose3d>();
        for(double[] i:Constants.Vision.tags_map){
            apriltags_map.put((int)i[0], new Pose3d(i[1] * 0.0254, i[2] * 0.0254, i[3] * 0.0254, new Rotation3d(0, i[5], i[4])));
        }

        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable adv_vision = inst.getTable("adv_vision");
        adv_pose_pub = adv_vision.getStructTopic("Pose", Pose2d.struct).publish();
        vision_pose_array_pub = adv_vision.getStructArrayTopic("photon poses", Pose2d.struct).publish();
        diff_pub = adv_vision.getStructTopic("diff pose", Transform2d.struct).publish();

        apriltags_array_pub = adv_vision.getStructArrayTopic("apriltags", Pose3d.struct).publish();
        

        pose = new Pose2d(0.12, 0, new Rotation2d());

        AprilTagFieldLayout APRILTAG_FIELD_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

        // 1 in = 0.0254 m
        photon_pose_estimator = new PhotonPoseEstimator(APRILTAG_FIELD_LAYOUT, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, new Transform3d(new Translation3d(2.5 * 0.0254, -0.305, 9 * 0.0254), new Rotation3d(0, 0, Math.toRadians(20))));
        // correct: 10.3 
        // object detection camera (on the left of the robot)
        photon_pose_estimator2 = new PhotonPoseEstimator(APRILTAG_FIELD_LAYOUT, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, new Transform3d(new Translation3d(2.5 * 0.0254, 0.305 + 0.052, 9 * 0.0254), new Rotation3d(0, 0,-Math.toRadians(20))));

        photon_pose_estimator.setReferencePose(pose);
        photon_pose_estimator2.setReferencePose(pose);

        if (!Robot.isReal()) return;

        photon_tag_camera = new PhotonCamera("OV9281");
        photon_tag_camera2 = new PhotonCamera("OV9782");
    }

    @Override 
    public void periodic() {
        pose = getOdomPose.get();
        odom_estimator = getPoseEstimator.get();
        apriltag_poses.clear();

        // photon_tag_camera2.setPipelineIndex(1);

        if (Robot.isReal()) {
            Pose2d[] photon_poses_list = new Pose2d[2];
            photon_result = photon_tag_camera.getAllUnreadResults();
            for (var result : photon_result){
                if (result.hasTargets()) {
                    var update = photon_pose_estimator.update(result);
                    if (update.isPresent()) {
                        photon_pose = update.get().estimatedPose.toPose2d();
                        photon_poses_list[0] = photon_pose;
                        photon_pose_timestamp = result.getTimestampSeconds();
                        photon_pose_chaching.addPose(photon_pose, getOdomPose.get(), photon_pose_timestamp);
                        photon_pose_stddev = photon_pose_chaching.getStdevs();
                        // if (result.multitagResult.isPresent()){
                        //     photon_pose_stddev.div(result.multitagResult.get().estimatedPose.ambiguity);
                        // }else{
                        //     photon_pose_stddev.div(result.targets.get(0).poseAmbiguity);
                        // }
                        // if (Math.abs(pose.getRotation().getRadians()) < 1) photon_pose_estimator.setReferencePose(photon_pose);
                    }
                    System.out.println(photon_pose_stddev);
                    // store it in apriltag 
                    for (PhotonTrackedTarget target:result.getTargets()){
                        apriltag_poses.add(apriltags_map.get(target.fiducialId));
                    }
                }
            }

            if (stero){
                var photon_result2 = photon_tag_camera2.getAllUnreadResults();
                for (var result : photon_result2){
                    if (result.hasTargets()) {
                        var update = photon_pose_estimator2.update(result);
                        if (update.isPresent()) {
                            photon_pose2 = update.get().estimatedPose.toPose2d();
                            photon_poses_list[1] = photon_pose2;
                            photon_pose2_timestamp = result.getTimestampSeconds();
                            photon_pose2_chaching.addPose(photon_pose2, getOdomPose.get(), photon_pose2_timestamp);
                            photon_pose2_stddev = photon_pose_chaching.getStdevs();
                            // if (result.multitagResult.isPresent()){
                            //     photon_pose2_stddev.div(result.multitagResult.get().estimatedPose.ambiguity);
                            // }else{
                            //     System.out.println(photon_pose2_stddev);
                            //     photon_pose2_stddev.div(result.targets.get(0).poseAmbiguity);
                            // }
                            // if (Math.abs(pose.getRotation().getRadians()) < 1) photon_pose_estimator2.setReferencePose(photon_pose);
                        }
                        // System.out.println(photon_pose2_stddev);
                        // store it in apriltag 
                        for (PhotonTrackedTarget target:result.getTargets()){
                            apriltag_poses.add(apriltags_map.get(target.fiducialId));
                        }
                    }
                }
            }
            vision_pose_array_pub.set(photon_poses_list);
            if (photon_poses_list[0] != null && photon_poses_list[1] != null) {
                diff_pub.set(photon_poses_list[0].minus(photon_poses_list[1]));

            }

        }

        
        
        // smart cropping:
        LimelightResults limelightResult = LimelightHelpers.getLatestResults("");
        if(limelightResult.valid){
            // non-dynamic
            LimelightHelpers.setPipelineIndex("", 6);
            count = 0;
        }
        if(!limelightResult.valid && count >=25){
            LimelightHelpers.setPipelineIndex("", 0);
        }


        if (limelightResult != null && limelightResult.valid){
            limelight_pose_timestamp = limelightResult.timestamp_LIMELIGHT_publish;
            if (DriverStation.getAlliance().get() == Alliance.Red){
                limelight_pose = LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2("").pose;
                limelight_pose_timestamp = LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2("").timestampSeconds;
            } else {
                limelight_pose = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("").pose;
                limelight_pose_timestamp = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("").timestampSeconds;
            }
            limelight_pose_chaching.addPose(limelight_pose, getOdomPose.get(), limelight_pose_timestamp);
            limelight_pose_stddev = limelight_pose_chaching.getStdevs();

             RawFiducial[] fiducials = LimelightHelpers.getRawFiducials("");
            // if (fiducials != null){
            //     limelight_pose_stddev.times(fiducials[0].ambiguity);
            // }
            // store it in apriltag 
            for (RawFiducial fudicial:fiducials){
                apriltag_poses.add(apriltags_map.get(fudicial));
            }
        }


        // publish aprlitag poses
        if (apriltag_poses != null){
            apriltags_array_pub.set((new HashSet<Pose3d>(apriltag_poses)).toArray(new Pose3d[0]));
        }

        if (limelight_pose_stddev != null){
            odom_estimator.addVisionMeasurement(limelight_pose, limelight_pose_timestamp, limelight_pose_stddev);
        }else{
            odom_estimator.addVisionMeasurement(limelight_pose, limelight_pose_timestamp);
        }

        if (photon_pose_stddev != null){
            odom_estimator.addVisionMeasurement(photon_pose, photon_pose_timestamp, photon_pose_stddev);
        }else{
            odom_estimator.addVisionMeasurement(photon_pose, photon_pose_timestamp);
        }

        if (photon_pose2_stddev != null){
            odom_estimator.addVisionMeasurement(photon_pose2, photon_pose2_timestamp, photon_pose2_stddev);
        }else{
            odom_estimator.addVisionMeasurement(photon_pose2, photon_pose2_timestamp);
        }
        // System.out.println(photon_pose2_stddev);



        pose = getOdomPose.get();
        adv_pose_pub.set(pose);
    }
}
