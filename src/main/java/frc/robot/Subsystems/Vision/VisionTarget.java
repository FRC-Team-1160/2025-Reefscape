// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems.Vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;


public class VisionTarget {

    public Translation2d position;
    public Timer timer;
    public int marked;

    /** Creates a new Target. */
    public VisionTarget(Pose2d position){
        this(position.getTranslation());
    }

    /** Creates a new Target. */
    public VisionTarget(Translation2d position){
        this.position = position;
        timer = new Timer();
        timer.start();
        marked = 0;
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
        return position.minus(other_pose).getNorm();
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
     * Updates the vision-estimated position of the object and resets its deletion marks.
     * @param new_pose The new pose.
     */
    public void updatePosition(Pose2d new_pose) {
        position = new_pose.getTranslation();
        timer.restart();
        marked = 0;
    }

    /**
     * Resets the target's deletion marks without updating the pose.
     */
    public void resetTimer() {
        timer.restart();
        marked = 0;
    }
}
