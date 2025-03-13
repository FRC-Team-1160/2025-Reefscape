package frc.robot.Subsystems.Vision;

import java.util.LinkedList;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class VisionPoseCache {

    private final double POSE_TIMEOUT;
    // A convenience class for storing poses with their timestamp
    private record CachedPose(double x, double y, double a, double timestamp) {}
    // LinkedList is usually suboptimal, but we only need to access the first and last element
    private final LinkedList<CachedPose> cache;
    // Take a rolling sum and sum of squares to calculate stdevs
    private double[] sums, sum_squares;

    public double last_ambiguity;

    /** Creates a new VisionPoseCache. */
    public VisionPoseCache() {
        POSE_TIMEOUT = 0.25;
        cache = new LinkedList<CachedPose>();
        sums = new double[3];
        sum_squares = new double[3];
    }

    /**
     * Updates the cache with a new vision estimate. Clears out old datapoints.
     * @param pose The vision-estimated pose.
     * @param referencePose The  predicted pose of the robot from odometry.
     * @param timestampSeconds The time of the vision estimate.
     */
    public void addPose(Pose2d pose, Pose2d referencePose, double timestampSeconds) {
        clearOldPoses(timestampSeconds);
        if (cache.isEmpty() || Math.abs(timestampSeconds - cache.getLast().timestamp) > 1e-3) {
            // We take the difference between the actual pose and the vision estimate to account for robot motion.
            // Theoretically, caching and tracing odometry readings for reference pose would be more correct.
            Transform2d diff = pose.minus(referencePose);
            // Take rolling sum and sum of squares for stdev calculation
            sums[0] += diff.getX();
            sums[1] += diff.getY();
            sums[2] += diff.getRotation().getRadians();

            sum_squares[0] += Math.pow(diff.getX(), 2);
            sum_squares[1] += Math.pow(diff.getY(), 2);
            sum_squares[2] += Math.pow(diff.getRotation().getRadians(), 2);

            // Check if current pose is older than the top of the cache; if so, insert it one index deep
            // This assumes that no more than 2 results are provided per batch, which may occur during a loop overrun.
            cache.add(cache.size() - (cache.isEmpty() || timestampSeconds > cache.getLast().timestamp ? 0 : 1),
                new CachedPose(diff.getX(), diff.getY(), diff.getRotation().getRadians(), timestampSeconds));

        }

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
        // Reset the sums if the cache is empty to mitigate floating point error accumulation
        if (cache.isEmpty()) {
            sums = new double[3];
            sum_squares = new double[3];
        }
    }

    // The standard deviation can be calculated with no iterative passes using the sum and sum of squares.
    /* Theoretically, taking a linear regression would account for change over time,
       giving marginal time range and accuracy improvements */
    private double calcStdev(int i) {
        return cache.isEmpty() ? 0 : Math.sqrt((sum_squares[i] - (sums[i] * sums[i] / cache.size())) / cache.size());
    }

    /**
     * Returns the standard deviations for the associated camera based on the cached measurements.
     * @return The x, y, and angular standard deviations in meters and radians.
     */
    public Matrix<N3, N1> getStdevs() {
        return VecBuilder.fill(calcStdev(0), calcStdev(1), calcStdev(2));
    }

    /**
     * Returns standard deviations in combination with the last recorded ambiguity rating.
     * @return The weighted x, y, and angular standard deviations.
     * 
     */
    public Matrix<N3, N1> getWeightedStdevs() {
        return getStdevs().plus(last_ambiguity / 2);
    }
}
