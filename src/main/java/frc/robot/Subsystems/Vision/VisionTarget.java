package frc.robot.Subsystems.Vision;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.VisionConstants.AlgaeParams;

public class VisionTarget {

    public Translation2d position;
    public Timer timer;
    public int timeout;

    /** Creates a new Target. */
    public VisionTarget(Pose2d position) {
        this(position.getTranslation());
    }

    /** Creates a new Target. */
    public VisionTarget(Translation2d position) {
        this.position = position;
        timer = new Timer();
        timer.start();
        timeout = 0;
    }

    /**
     * Returns the object's pose as a Pose2d.
     * @return The object's pose.
     */
    public Pose2d getPose() {
        return new Pose2d(position, new Rotation2d());
    }

    /**
     * Returns the object's pose as a Pose3d.
     * @return The object's pose with a small Z offset.
     */
    public Pose3d getPose3d() {
        return new Pose3d(
            position.getX(),
            position.getY(),
            AlgaeParams.TARGET_WIDTH * (0.5 - timer.get() / AlgaeParams.TRACKING_TIMEOUT),
            new Rotation3d());
    }

    /**
     * Calculates the distance from a pose to the object.
     * @param other_pose The reference pose.
     * @return The calculated distance in meters.
     */
    public double getDistance(Pose2d other_pose) {
        return getDistance(other_pose.getTranslation());
    }

    /**
     * Calculates the distance from a position to the object.
     * @param other_pose The reference position.
     * @return The calculated distance in meters.
     */
    public double getDistance(Translation2d other_pose) {
        return position.getDistance(other_pose);
    }

    /**
     * Calculates the angle from the frame of reference of a pose.
     * @param other_pose The reference pose. The rotation of this pose affects the output.
     * @return The calculated angle.
     */
    public Rotation2d getAngle(Pose2d other_pose) {
        return getAngle(other_pose.getTranslation()).minus(other_pose.getRotation());
    }

    /**
     * Calculates the field-relative angle of the object from a position.
     * @param other_pose The reference position.
     * @return The calculated angle.
     */
    public Rotation2d getAngle(Translation2d other_pose) {
        return position.minus(other_pose).getAngle();
    }

    /**
     * Updates the vision-estimated position of the object and resets its timeout.
     * @param new_pose The new pose.
     */
    public void updatePosition(Pose2d new_pose) {
        updatePosition(new_pose, 1);
    }

    /**
     * Returns the vision-estimated positoin of the object and resets its timeout.
     * @param new_pose The estimated pose.
     * @param weight The relative weight to be given to the new pose. 0 is no weight, and 1 overrides the stored pose.
     * @return The new calculated pose of the target.
     */
    public Pose2d updatePosition(Pose2d new_pose, double weight) {
        weight = MathUtil.clamp(weight, 0, 1);
        position = new Translation2d(
            new_pose.getX() * weight + position.getX() * (1.0 - weight),
            new_pose.getY() * weight + position.getY() * (1.0 - weight));
        resetTimer();
        return getPose();
    }

    /**
     * Resets the target's deletion marks without updating the pose.
     */
    public void resetTimer() {
        timer.restart();
        timeout = 0;
    }

    public boolean inAngularTolerance(Pose2d observed_pose, Pose2d robot_pose) {
        Translation2d robot_pos = robot_pose.getTranslation();
        return observed_pose.getTranslation().minus(robot_pos).getAngle()
            .minus(position.minus(robot_pos).getAngle()).getRadians() <= AlgaeParams.ANGULAR_TOLERANCE;
    } 

    public boolean inDistanceTolerance(Pose2d observed_pose, Pose2d robot_pose) {
        Translation2d observed_pos = observed_pose.getTranslation();
        Translation2d robot_pos = robot_pose.getTranslation();
        // Calculate the ratio of the error to the distance; similar to percent error
        double error_ratio = Math.abs((observed_pos.getDistance(robot_pos) - getDistance(robot_pos))
             / getDistance(observed_pos));
        // Check both the ratio and the reciprocal meet the threshold
        return Math.min(error_ratio, 1.0 / error_ratio) >= AlgaeParams.DISTANCE_TOLERANCE;
    }

    public boolean inPositionTolerance(Pose2d observed_pose) {
        return observed_pose.getTranslation().minus(position).getNorm() <= AlgaeParams.POSITION_TOLERANCE;
    }

    /**
     * Returns whether the observed target matches this target's position. Unmarks this target if so.
     * @param observed_pose The observed target pose.
     * @param robot_pose The robot's current pose.
     * @param update_pose Whether or not to update the position of the target with this observation if matched.
     * @return Whether or not the observed target matches this target.
     */
    public boolean match(Pose2d observed_pose, Pose2d robot_pose, boolean update_pose) {
        // Check if the given pose has a close enough angle and distance
        return match(observed_pose, robot_pose, update_pose ? 1 : 0);
    }

    /**
     * Returns whether the observed target matches this target's position. Unmarks this target if so.
     * @param observed_pose The observed target pose.
     * @param robot_pose The robot's current pose.
     * @param update_pose Whether or not to update the position of the target with this observation if matched.
     * @return Whether or not the observed target matches this target.
     */
    public boolean match(Pose2d observed_pose, Pose2d robot_pose, double weight) {
        // Check if the given pose has a close enough angle and distance
        if (inPositionTolerance(observed_pose) ||
            (inAngularTolerance(observed_pose, robot_pose) && inDistanceTolerance(observed_pose, robot_pose))) {

            updatePosition(observed_pose, weight);
            return true;
        }
        return false;
    }
}
