package frc.robot.Subsystems.Vision;

import java.util.ArrayDeque;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class VisionPoseCache {

    private final double POSE_TIMEOUT;
    // A convenience class for storing poses with their timestamp
    private record CachedPose(double x, double y, double a, double timestamp) {}
    // A cache for the stored poses. ArrayDeque is used because we only need to access the first and last element.
    private final ArrayDeque<CachedPose> cache;
    // Take a rolling sum and sum of squares to calculate stdevs
    private double[] sums, sum_squares;

    private double last_ambiguity;

    /** Creates a new VisionPoseCache. */
    public VisionPoseCache() {
        POSE_TIMEOUT = 0.25;
        cache = new ArrayDeque<CachedPose>();
        sums = new double[3];
        sum_squares = new double[3];
    }

    /**
     * Updates the cache with a new vision estimate. Clears out old datapoints.
     * @param pose The vision-estimated pose.
     * @param referencePose The predicted pose of the robot from odometry.
     * @param timestampSeconds The time of the vision estimate.
     */
    public void addPose(Pose2d pose, Pose2d referencePose, double timestampSeconds) {
        if (cache.isEmpty() || Math.abs(timestampSeconds - cache.getLast().timestamp) > 1e-4) {
            // We take the difference between the actual (best combined estimate) pose and the vision estimate
            // to account for robot motion
            Transform2d diff = pose.minus(referencePose);
            // Take rolling sum and sum of squares
            sums[0] += diff.getX();
            sums[1] += diff.getY();
            sums[2] += diff.getRotation().getRadians();

            sum_squares[0] += Math.pow(diff.getX(), 2);
            sum_squares[1] += Math.pow(diff.getY(), 2);
            sum_squares[2] += Math.pow(diff.getRotation().getRadians(), 2);

            cache.add(new CachedPose(
                diff.getX(), 
                diff.getY(), 
                diff.getRotation().getRadians(), 
                timestampSeconds));
        }
        clearOldPoses(timestampSeconds);

    }

    public void updateAmbiguity(double ambiguity) {
        this.last_ambiguity = ambiguity;
    }

    public void clearOldPoses(double timestampSeconds) {
        while (!cache.isEmpty()) {
            CachedPose first = cache.getFirst();
            // Remove data from cache if too old
            if (timestampSeconds - first.timestamp > POSE_TIMEOUT) {
                // Remove this pose from the rolling sums
                sums[0] -= first.x;
                sums[1] -= first.y;
                sums[2] -= first.a;

                sum_squares[0] -= first.x * first.x;
                sum_squares[1] -= first.y * first.y;
                sum_squares[2] -= first.a * first.a;
                
                cache.removeFirst();
            } else {
                break;
            }
        }
    }

    // The standard deviation can be calculated with no iterative passes using the sum and sum of squares
    private double calcStdev(int i) {
        return Math.sqrt((sum_squares[i] - (sums[i] * sums[i] / cache.size())) / cache.size());
    }

    /**
     * Returns the standard deviations for the associated camera based on the cached measurements.
     * @return The x, y, and angular standard deviations.
     */
    public Matrix<N3, N1> getStdevs() {
        return new Matrix<>(Nat.N3(), Nat.N1(), new double[] {
                calcStdev(0),
                calcStdev(1),
                calcStdev(2)
            }).plus(last_ambiguity);
    }
}
