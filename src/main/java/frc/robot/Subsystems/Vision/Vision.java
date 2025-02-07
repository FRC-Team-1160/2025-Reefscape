package frc.robot.Subsystems.Vision;
import java.util.function.BiConsumer;
import java.util.function.Supplier;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class Vision extends SubsystemBase {

    public int count;

    StructPublisher<Pose2d> adv_pose_pub;
    StructArrayPublisher<Pose2d> adv_target_pub;
    StructPublisher<Pose2d> adv_tracked_pub;

    PhotonCamera photon_tag_camera;
    PhotonPoseEstimator photon_pose_estimator;

    public ObjectDetection m_object_detection;

    BiConsumer<Pose2d, Double> update_pose_estimator;

    Supplier<Pose2d> robot_pose_supplier;

    Transform2d camera_1_offset;

    /** Creates a new Vision. */
    public Vision(BiConsumer<Pose2d, Double> update_pose_estimator, Supplier<Pose2d> robot_pose_supplier) {

        this.robot_pose_supplier = robot_pose_supplier;
        this.update_pose_estimator = update_pose_estimator;

        m_object_detection = new ObjectDetection(robot_pose_supplier);

        if (Robot.isSimulation()) return; // Exit if in simulation

        NetworkTable adv_vision = NetworkTableInstance.getDefault().getTable("adv_vision");
        adv_pose_pub = adv_vision.getStructTopic("Pose", Pose2d.struct).publish();

        photon_tag_camera = new PhotonCamera("OV9281");

        AprilTagFieldLayout APRILTAG_FIELD_LAYOUT = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

        photon_pose_estimator = new PhotonPoseEstimator(
            APRILTAG_FIELD_LAYOUT, 
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, 
            new Transform3d());
        photon_pose_estimator.setReferencePose(new Pose2d());

    }

    @Override 
    public void periodic() {
        // Object detection is still updated during simulation
        m_object_detection.update(); 

        if (Robot.isSimulation()) return;

        var photon_result = photon_tag_camera.getLatestResult();
        if (photon_result.hasTargets()) {
            var update = photon_pose_estimator.update(photon_result);
            if (update.isPresent()) {
                Pose2d photon_pose = update.get().estimatedPose.toPose2d();
                update_pose_estimator.accept(photon_pose.plus(camera_1_offset), update.get().timestampSeconds);
            }
        }
         
    }
}
