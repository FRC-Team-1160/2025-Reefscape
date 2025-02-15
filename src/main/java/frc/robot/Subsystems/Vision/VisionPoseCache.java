package frc.robot.Subsystems.Vision;



import java.util.LinkedList;

import edu.wpi.first.math.geometry.Pose2d;

public class VisionPoseCache {

    public final double POSE_TIMEOUT;

    private final LinkedList<CachedPose> cache;

    public class CachedPose {
        public Pose2d robot_pose;
        public double time;

        public CachedPose(Pose2d pose, double timestampSeconds) {
            this.robot_pose = pose;
            this.time = timestampSeconds;
        }
    }

    public VisionPoseCache() {
        this.POSE_TIMEOUT = 0.25;
        this.cache = new LinkedList<CachedPose>();
    }

    public void addPose(Pose2d pose, double timestampSeconds) {
        if (cache.isEmpty() || Math.abs(timestampSeconds - cache.getFirst().time) < 1e-4) {
            cache.add(new CachedPose(pose, timestampSeconds));
        }
    }

    public void clearOldPoses(double timestampSeconds) {
        while (!cache.isEmpty()) {
            if (timestampSeconds - cache.getFirst().time > pose_timeout) {
                cache.removeFirst();
            } else {
                break;
            }
        }
    }
}
