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

// limelight lib
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightResults;

public class Vision extends SubsystemBase {
        private static final AprilTagFieldLayout AprilTagFieldLayout = null;
        private static Vision m_instance;
        public Pose3d m_pose;
        // public DriveTrain m_drive;

        public int count;
    
        StructPublisher<Pose3d> adv_posePub;
        StructArrayPublisher<Pose3d> adv_targetPub;
        StructPublisher<Pose3d> adv_trackedPub;
    
        PhotonCamera m_photonTagCamera;
    
        PhotonPoseEstimator m_photonPoseEstimator;
    
        public static Vision getInstance(){
            if (m_instance == null){
              m_instance = new Vision();
            }
            return m_instance;
        }
    
        public Vision() {
            NetworkTableInstance inst = NetworkTableInstance.getDefault();
            NetworkTable adv_vision = inst.getTable("adv_vision");
            adv_posePub = adv_vision.getStructTopic("Pose", Pose3d.struct).publish();
            adv_targetPub = adv_vision.getStructArrayTopic("Target", Pose3d.struct).publish();
            adv_trackedPub = adv_vision.getStructTopic("Tracked", Pose3d.struct).publish();
    
            m_photonTagCamera = new PhotonCamera("OV9281");
    
            m_pose = new Pose3d(12.5, 0, 0, new Rotation3d());
    
            AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2025.loadAprilTagLayoutField();
            m_photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, new Transform3d(new Translation3d(0.15 * 0.0254, 0.10 * 0.0254, 0.1), new Rotation3d(0, 20.0 * Math.PI / 180, 0)));
            m_photonPoseEstimator.setReferencePose(m_pose);
        }

    @Override 
    public void periodic() {
        count++;

        var photonResult = m_photonTagCamera.getLatestResult();
        if (photonResult.hasTargets()){
        var update = m_photonPoseEstimator.update(photonResult);
        if (update.isPresent()){
            m_pose = update.get().estimatedPose;
            if (Math.abs(m_pose.getZ()) < 1){
            m_photonPoseEstimator.setReferencePose(m_pose);
            }
            // if (m_drive != null){
            //     m_drive.m_poseEstimator.addVisionMeasurement(m_pose.toPose2d(), Timer.getFPGATimestamp());
            //     // System.out.println(m_pose.getX());
            // }
        }
        }
        
        // smart cropping:
        LimelightResults limelightResult = LimelightHelpers.getLatestResults("");
        if(limelightResult.valid){
            double tag_x = LimelightHelpers.getTX("");
            double tag_y = LimelightHelpers.getTY("");
            // dynamic cropping
            // if(tag_x >=-0.83 && tag_x <=0.07){
            //   LimelightHelpers.setPipelineIndex("", 1);
            // }else if (tag_x >=-0.63 && tag_x <=0.27) {
            //   LimelightHelpers.setPipelineIndex("", 2);
            // }else if (tag_x >=-0.43 && tag_x <=0.47) {
            //   LimelightHelpers.setPipelineIndex("", 3);
            // }else if (tag_x >=-0.23 && tag_x <=0.67) {
            //   LimelightHelpers.setPipelineIndex("", 4);
            // }else if (tag_x >=-0.03 && tag_x <=0.87) {
            //   LimelightHelpers.setPipelineIndex("", 5);
            // }

            // non-dynamic
            LimelightHelpers.setPipelineIndex("", 6);
            count = 0;
        }

    }
}
