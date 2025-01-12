package frc.robot.Subsystems.Vision;
import java.util.ArrayList;

import org.opencv.core.Point;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// limelight lib
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.Subsystems.DriveTrain.DriveTrain;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.LimelightResults;

public class Vision extends SubsystemBase {
        private static Vision m_instance;
        public Pose3d m_pose;
        public DriveTrain m_drive;

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
    
            m_photonTagCamera = new PhotonCamera("Arducam_OV9281_USB_Camera");
    
            m_pose = new Pose3d(12.5, 0, 0, new Rotation3d());
    
            AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

            m_photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, new Transform3d(new Translation3d(0.15 * 0.0254, 0.10 * 0.0254, 0.1), new Rotation3d(0, 20.0 * Math.PI / 180, 0)));
            m_photonPoseEstimator.setReferencePose(m_pose);
        }

    @Override 
    public void periodic() {
        count++;

        var photonResult = m_photonTagCamera.getAllUnreadResults().get(0);
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
        if(!limelightResult.valid && count >=25){
            LimelightHelpers.setPipelineIndex("", 0);
        }


        // if (limelightResult != null && limelightResult.valid){
        //     if (DriverStation.getAlliance().get() == Alliance.Red){
        //         m_pose = LimelightHelpers.getBotPose3d_wpiRed("");
        //     }else{
        //         m_pose = LimelightHelpers.getBotPose3d_wpiBlue("");
        //     }

        //     System.out.println(m_pose.getX());

        //     // if (m_drive != null){
        //     //     m_drive.m_poseEstimator.addVisionMeasurement(m_pose.toPose2d(), Timer.getFPGATimestamp());
        //     // }
        // }

        adv_posePub.set(m_pose);

        // double min_dist = 99;

        // if (m_drive.odomPose != null && noteCenters != null){
        //     Pose2d odomPose = m_drive.odomPose;
        //     double rot = odomPose.getRotation().getRadians();
        //     ArrayList<Pose3d> poses3d = new ArrayList<Pose3d>();
        //     for (Point p : noteCenters){
        //         double f = 0.76 / (Math.tan(38 / 2 * Math.PI/180) * (p.y - 60)/60);
        //         double h = -f * Math.tan(63 / 2 * Math.PI/180) * (p.x - 80)/80;
        //         double x = f * Math.cos(rot) - h * Math.sin(rot);
        //         double y = f * Math.sin(rot) + h * Math.cos(rot);
        //         double dist = Math.sqrt(f*f + h*h);
        //         Pose3d pose = new Pose3d(
        //         x + odomPose.getX(),
        //         y + odomPose.getY(),
        //         0.04,
        //         new Rotation3d()
        //         );
        //         poses3d.add(pose);
        //         if (dist < min_dist){
        //         min_dist = dist;
        //         tracked_note = pose;
        //         tracking_timeout.reset();
        //         }
        //     }
        //     adv_targetPub.set(poses3d.toArray(new Pose3d[0]));
        //     adv_trackedPub.set(tracked_note);
        //     } else {
        //     if (tracking_timeout.hasElapsed(0.5)){
        //         tracked_note = null;
        //     }
        // }
    }
}
