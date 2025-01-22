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

import frc.robot.Subsystems.DriveTrain.DriveTrain;

public class Vision extends SubsystemBase {
    private static Vision m_instance;

    public Pose3d m_pose;
    public Pose3d m_photonPose;
    public Pose3d m_limelightPose;

    public DriveTrain m_drive;

    public int count;

    StructPublisher<Pose3d> adv_posePub;
    StructArrayPublisher<Pose3d> adv_targetPub;
    StructPublisher<Pose3d> adv_trackedPub;

    PhotonCamera m_photonTagCamera;

    PhotonPoseEstimator m_photonPoseEstimator;

    boolean sim = false;

    public static Vision getInstance(){
        if (m_instance == null){
            m_instance = new Vision();
        }
        return m_instance;
    }

    public Vision() {
        if (!sim){
            NetworkTableInstance inst = NetworkTableInstance.getDefault();
            NetworkTable adv_vision = inst.getTable("adv_vision");
            adv_posePub = adv_vision.getStructTopic("Pose", Pose3d.struct).publish();

            m_photonTagCamera = new PhotonCamera("Arducam_OV9281_USB_Camera (1)");

            m_pose = new Pose3d(0.12, 0, 0, new Rotation3d());

            AprilTagFieldLayout aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

            m_photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, new Transform3d(new Translation3d(-0.15 * 0.0254, 0.10 * 0.0254, 0.2), new Rotation3d(0, 20.0 * Math.PI / 180, 0)));
            m_photonPoseEstimator.setReferencePose(m_pose);
        }
    }

    public Pose3d combinePoses(Pose3d photonPose, double photonWeight, Pose3d limelightPose, double limelightWeight){
        double totalWeight = photonWeight + limelightWeight;
        photonWeight /= totalWeight;
        limelightWeight /= totalWeight;

        // combine xy
        double combinedX = photonPose.getX() * photonWeight + limelightPose.getX() * limelightWeight;
        double combinedY = photonPose.getY() * photonWeight + limelightPose.getY() * limelightWeight;
        // combine rotation
        double photonTheta = photonPose.getRotation().getZ();
        double limelightTheta = limelightPose.getRotation().getZ();
        double combinedTheta = photonTheta * photonWeight + limelightTheta * limelightWeight;

        return new Pose3d(combinedX, combinedY, photonPose.getZ(), new Rotation3d(0.0,0.0,combinedTheta));
    }


    @Override 
    public void periodic() {
        if (!sim){
            var photonResult = m_photonTagCamera.getLatestResult();
            if (photonResult.hasTargets()){
                var update = m_photonPoseEstimator.update(photonResult);
                if (update.isPresent()){
                    m_photonPose = update.get().estimatedPose;
                    if (Math.abs(m_pose.getZ()) < 1){
                    m_photonPoseEstimator.setReferencePose(m_photonPose);
                    }
                    // if (m_drive != null){
                    //     m_drive.m_poseEstimator.addVisionMeasurement(m_pose.toPose2d(), Timer.getFPGATimestamp());
                    //     // System.out.println(m_pose.getX());
                    // }
                }
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

        //     // non-dynamic
        //     LimelightHelpers.setPipelineIndex("", 6);
        //     count = 0;
        // }
        // if(!limelightResult.valid && count >=25){
        //     LimelightHelpers.setPipelineIndex("", 0);
        // }


        // if (limelightResult != null && limelightResult.valid){
        //     if (DriverStation.getAlliance().get() == Alliance.Red){
        //         m_limelightPose = LimelightHelpers.getBotPose3d_wpiRed("");
        //     }else{
        //         m_limelightPose = LimelightHelpers.getBotPose3d_wpiBlue("");
        //     }
        // }

        //     System.out.println(m_pose.getX());

        //     // if (m_drive != null){
        //     //     m_drive.m_poseEstimator.addVisionMeasurement(m_pose.toPose2d(), Timer.getFPGATimestamp());
        //     // }

        //     // non-dynamic
        //     LimelightHelpers.setPipelineIndex("", 6);
        //     count = 0;
        // }
        // if(!limelightResult.valid && count >=25){
        //     LimelightHelpers.setPipelineIndex("", 0);
        // }


        // if (limelightResult != null && limelightResult.valid){
        //     if (DriverStation.getAlliance().get() == Alliance.Red){
        //         m_limelightPose = LimelightHelpers.getBotPose3d_wpiRed("");
        //     }else{
        //         m_limelightPose = LimelightHelpers.getBotPose3d_wpiBlue("");
        //     }
        // }

        // //     System.out.println(m_pose.getX());

        // //     // if (m_drive != null){
        // //     //     m_drive.m_poseEstimator.addVisionMeasurement(m_pose.toPose2d(), Timer.getFPGATimestamp());
        // //     // }
        // // }
        // if (m_limelightPose != null && m_photonPose != null){
        //     m_pose = combinePoses(m_photonPose, 0.5, m_limelightPose, 0.5);
        // }else if(limelightResult.valid){
        //     m_pose = m_limelightPose;
        // }else if(photonResult.hasTargets()){
        //     m_pose = m_photonPose;
        //     System.out.println("HELSDLASKJDLKDJ");
        // }

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
        m_pose = new Pose3d(13, 7, 0, new Rotation3d(0.0, 0.0, Math.PI));
        
        adv_posePub.set(m_pose);
    }
}
