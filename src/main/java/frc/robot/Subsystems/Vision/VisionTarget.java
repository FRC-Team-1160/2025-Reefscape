package frc.robot.Subsystems.Vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
        position = new_pose.getTranslation();
        timer.restart();
        timeout = 0;
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
        double proportional_error = Math.abs((observed_pos.getDistance(robot_pos)
        - getDistance(robot_pos)) / getDistance(observed_pos));
        return Math.abs(proportional_error - 1) <= AlgaeParams.DISTANCE_TOLERANCE;
    }

    public boolean inPositionTolerance(Pose2d observed_pose) {
        return observed_pose.getTranslation().minus(position).getNorm() < 1;
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
        if (inPositionTolerance(observed_pose) || (
            inAngularTolerance(observed_pose, robot_pose) && inDistanceTolerance(observed_pose, robot_pose))) {
            resetTimer();
            if (update_pose) updatePosition(observed_pose);
            return true;
        }
        return false;
    }
}
