package frc.robot.Subsystems.Vision;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class Vision extends SubsystemBase {
    public Pose2d pose;
    public Pose2d photon_pose;
    public Pose2d limelight_pose;

    public int count;

    StructPublisher<Pose2d> adv_pose_pub;
    StructArrayPublisher<Pose2d> adv_target_pub;
    StructPublisher<Pose2d> adv_tracked_pub;

    PhotonCamera photon_tag_camera;
    PhotonPoseEstimator photon_pose_estimator;

    public Vision() {
        if (Robot.isReal()){
            NetworkTableInstance inst = NetworkTableInstance.getDefault();
            NetworkTable adv_vision = inst.getTable("adv_vision");
            adv_pose_pub = adv_vision.getStructTopic("Pose", Pose2d.struct).publish();

            photon_tag_camera = new PhotonCamera("OV9281");

            pose = new Pose2d(0.12, 0, new Rotation2d());

            AprilTagFieldLayout APRILTAG_FIELD_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

            photon_pose_estimator = new PhotonPoseEstimator(APRILTAG_FIELD_LAYOUT, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, new Transform3d(new Translation3d(-0.15 * 0.0254, 0.10 * 0.0254, 0.2), new Rotation3d(0, 20.0 * Math.PI / 180, 0)));
            photon_pose_estimator.setReferencePose(pose);
        }
    }

    public Pose2d combinePoses(Pose2d photon_pose, double photon_weight, Pose2d limelight_pose, double limelight_weight) {
        double total_weight = photon_weight + limelight_weight;
        photon_weight /= total_weight;
        limelight_weight /= total_weight;

        // combine xy
        double combinedX = photon_pose.getX() * photon_weight + limelight_pose.getX() * limelight_weight;
        double combinedY = photon_pose.getY() * photon_weight + limelight_pose.getY() * limelight_weight;
        // combine rotation
        double photon_theta = photon_pose.getRotation().getRadians();
        double limelight_theta = limelight_pose.getRotation().getRadians();
        double combined_theta = photon_theta * photon_weight + limelight_theta * limelight_weight;

        return new Pose2d(combinedX, combinedY, new Rotation2d(combined_theta));
    }


    @Override 
    public void periodic() {
        if (Robot.isReal()) {
            var photon_result = photon_tag_camera.getLatestResult();
            if (photon_result.hasTargets()){
                var update = photon_pose_estimator.update(photon_result);
                if (update.isPresent()){
                    photon_pose = update.get().estimatedPose.toPose2d();
                    if (Math.abs(pose.getRotation().getRadians()) < 1){
                    photon_pose_estimator.setReferencePose(photon_pose);
                    }
                }
            }
        }

        var photon_result = photon_tag_camera.getLatestResult();
        if (photon_result.hasTargets()) {
            var update = photon_pose_estimator.update(photon_result);
            if (update.isPresent()) {
                photon_pose = update.get().estimatedPose.toPose2d();
                if (Math.abs(pose.getRotation().getRadians()) < 1) photon_pose_estimator.setReferencePose(photon_pose);
            }
        }
        
        // smart cropping:
        // LimelightResults limelightResult = LimelightHelpers.getLatestResults("");
        // if(limelightResult.valid){
        //     double tag_x = LimelightHelpers.getTX("");
        //     double tag_y = LimelightHelpers.getTY("");
        //     // dynamic cropping
        //     // if(tag_x >=-0.83 && tag_x <=0.07){
        //     //   LimelightHelpers.setPipelineIndex("", 1);
        //     // }else if (tag_x >=-0.63 && tag_x <=0.27) {
        //     //   LimelightHelpers.setPipelineIndex("", 2);
        //     // }else if (tag_x >=-0.43 && tag_x <=0.47) {
        //     //   LimelightHelpers.setPipelineIndex("", 3);
        //     // }else if (tag_x >=-0.23 && tag_x <=0.67) {
        //     //   LimelightHelpers.setPipelineIndex("", 4);
        //     // }else if (tag_x >=-0.03 && tag_x <=0.87) {
        //     //   LimelightHelpers.setPipelineIndex("", 5);
        //     // }
        // 
        // 
        // 
        // if (limelightResult != null && limelightResult.valid){
        //     if (DriverStation.getAlliance().get() == Alliance.Red){
        //         m_limelightPose = LimelightHelpers.getBotPose3d_wpiRed("");
        //     } else {
        //         m_limelightPose = LimelightHelpers.getBotPose3d_wpiBlue("");
        //     }
        // }
        //
        //     // non-dynamic
        //     LimelightHelpers.setPipelineIndex("", 6);
        //     count = 0;
        // }
        // if(!limelightResult.valid && count >=25){
        //     LimelightHelpers.setPipelineIndex("", 0);
        // }
        // 
        //
        // if (m_limelightPose != null && m_photonPose != null){
        //     m_pose = combinePoses(m_photonPose, 0.5, m_limelightPose, 0.5);
        // }else if(limelightResult.valid){
        //     m_pose = m_limelightPose;
        // }else if(photonResult.hasTargets()){
        //     m_pose = m_photonPose;
        //     System.out.println("HELSDLASKJDLKDJ");
        // }
        // if (photonResult.hasTargets()){
        //     // m_pose = m_photonPose;
        // }

        // testing object detection
        pose = new Pose2d(13, 7, new Rotation2d(Math.PI));
        adv_pose_pub.set(pose);
    }
}
