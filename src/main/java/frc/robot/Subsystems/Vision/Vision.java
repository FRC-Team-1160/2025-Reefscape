package frc.robot.Subsystems.Vision;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
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
    public Pose3d pose;
    public Pose3d photon_pose;
    public Pose3d limelight_pose;

    public int count;

    StructPublisher<Pose3d> adv_pose_pub;
    StructArrayPublisher<Pose3d> adv_target_pub;
    StructPublisher<Pose3d> adv_tracked_pub;

    PhotonCamera photon_tag_camera;
    PhotonPoseEstimator photon_pose_estimator;

    public Vision() {
        if (Robot.isReal()){
            NetworkTableInstance inst = NetworkTableInstance.getDefault();
            NetworkTable adv_vision = inst.getTable("adv_vision");
            adv_pose_pub = adv_vision.getStructTopic("Pose", Pose3d.struct).publish();

            photon_tag_camera = new PhotonCamera("Arducam_OV9281_USB_Camera (1)");

            pose = new Pose3d(0.12, 0, 0, new Rotation3d());

            AprilTagFieldLayout APRILTAG_FIELD_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

            photon_pose_estimator = new PhotonPoseEstimator(APRILTAG_FIELD_LAYOUT, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, new Transform3d(new Translation3d(-0.15 * 0.0254, 0.10 * 0.0254, 0.2), new Rotation3d(0, 20.0 * Math.PI / 180, 0)));
            photon_pose_estimator.setReferencePose(pose);
        }
    }

    public Pose3d combinePoses(Pose3d photon_pose, double photon_weight, Pose3d limelight_pose, double limelight_weight) {
        double total_weight = photon_weight + limelight_weight;
        photon_weight /= total_weight;
        limelight_weight /= total_weight;

        // combine xy
        double combinedX = photon_pose.getX() * photon_weight + limelight_pose.getX() * limelight_weight;
        double combinedY = photon_pose.getY() * photon_weight + limelight_pose.getY() * limelight_weight;
        // combine rotation
        double photon_theta = photon_pose.getRotation().getZ();
        double limelight_theta = limelight_pose.getRotation().getZ();
        double combined_theta = photon_theta * photon_weight + limelight_theta * limelight_weight;

        return new Pose3d(combinedX, combinedY, photon_pose.getZ(), new Rotation3d(0, 0, combined_theta));
    }


    @Override 
    public void periodic() {
        if (Robot.isReal()) {
            var photon_result = photon_tag_camera.getLatestResult();
            if (photon_result.hasTargets()){
                var update = photon_pose_estimator.update(photon_result);
                if (update.isPresent()){
                    photon_pose = update.get().estimatedPose;
                    if (Math.abs(pose.getZ()) < 1){
                    photon_pose_estimator.setReferencePose(photon_pose);
                    }
                }
            }
        }

        var photon_result = photon_tag_camera.getLatestResult();
        if (photon_result.hasTargets()) {
            var update = photon_pose_estimator.update();
            if (update.isPresent()) {
                photon_pose = update.get().estimatedPose;
                if (Math.abs(pose.getZ()) < 1) photon_pose_estimator.setReferencePose(photon_pose);
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
        pose = new Pose3d(13, 7, 0, new Rotation3d(0.0, 0.0, Math.PI));
        adv_pose_pub.set(pose);
    }
}
