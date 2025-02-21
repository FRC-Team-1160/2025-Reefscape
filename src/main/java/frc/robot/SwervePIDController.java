package frc.robot;

import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.FieldConstants.AlgaeProcessor;
import frc.robot.Constants.FieldConstants.CoralStation;
import frc.robot.Constants.FieldConstants.Reef;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.SwerveConstants.Tracking;
import frc.robot.Constants.SwerveConstants.Tracking.PIDConstants.Angle;
import frc.robot.Constants.SwerveConstants.Tracking.PIDConstants.Distance;

public class SwervePIDController {

	StructPublisher<Pose2d> adv_goal_pose_pub;

	PIDController dist_pid_controller, ang_pid_controller;

	Supplier<Pose2d> robot_pose_supplier;
	Supplier<ChassisSpeeds> robot_speeds_supplier;

    public Pose2d target_pose;
	public double target_distance;
    public Rotation2d rotation_offset;

    public boolean reset_speeds;

    public static class FieldPositions {
        static Pose2d[] reef, source;
        static Pose2d processor;
    }

    public SwervePIDController(Supplier<Pose2d> robot_pose_supplier, Supplier<ChassisSpeeds> robot_speeds_supplier) {
        this(robot_pose_supplier, robot_speeds_supplier, 0.5);
    }

    public SwervePIDController(
        Supplier<Pose2d> robot_pose_supplier, 
        Supplier<ChassisSpeeds> robot_speeds_supplier, 
        double target_distance) {

        target_pose = new Pose2d();

        dist_pid_controller = new PIDController(Distance.kP, Distance.kI, Distance.kD);
        dist_pid_controller.setTolerance(Tracking.DISTANCE_TOLERANCE);

        ang_pid_controller = new PIDController(Angle.kP, Angle.kI, Angle.kD);
        ang_pid_controller.setTolerance(Tracking.ANGLE_TOLERANCE);
        ang_pid_controller.enableContinuousInput(-Math.PI, Math.PI);

        this.robot_pose_supplier = robot_pose_supplier;
        this.robot_speeds_supplier = robot_speeds_supplier;

        this.target_distance = target_distance;

        NetworkTable adv_vision = NetworkTableInstance.getDefault().getTable("adv_vision");
        adv_goal_pose_pub = adv_vision.getStructTopic("Goal Pose", Pose2d.struct).publish();

        reset_speeds = true;

        fillFieldPositions();
    }

    private void fillFieldPositions() {
        FieldPositions.reef = new Pose2d[Reef.NUM_SIDES];
        Rotation2d angle = Rotation2d.kPi;
        for (int i = 0; i < Reef.NUM_SIDES; i++) {
            FieldPositions.reef[i] = new Pose2d(
                Reef.CENTER_X,
                Reef.CENTER_Y,
                angle).plus(new Transform2d(
                        Reef.INNER_RADIUS + RobotConstants.BASE_WIDTH / 2,
                        0,
                        Rotation2d.kPi
                    )
                );
            angle = angle.plus(Rotation2d.fromRotations(1.0 / Reef.NUM_SIDES));
        }

        FieldPositions.source = new Pose2d[] {
            // right source
            new Pose2d(
                CoralStation.CENTER_X,
                CoralStation.CENTER_Y,
                Rotation2d.fromRadians(CoralStation.ANGLE_RADIANS).plus(Rotation2d.kPi)
            ),
            // left source
            new Pose2d(
                CoralStation.CENTER_X,
                FieldConstants.WIDTH - CoralStation.CENTER_Y,
                Rotation2d.fromRadians(-CoralStation.ANGLE_RADIANS).plus(Rotation2d.kPi)
            )
        };

        FieldPositions.processor = new Pose2d(
            AlgaeProcessor.CENTER_X,
            AlgaeProcessor.CENTER_Y,
            Rotation2d.fromRadians(AlgaeProcessor.ANGLE_RADIANS)
        );
    }

    /**
     * Returns the closest reef face to the robot. The driverstation-oriented face is 0, and the rest are numbered CCW increasing.
     * @return The number corresponding the closest reef face.
     */
    @AutoLogOutput
    public int getNearestReefFace() {
        Translation2d robot_position = robot_pose_supplier.get().getTranslation();
        Rotation2d reef_angle = new Translation2d(
                Reef.CENTER_X,
                Reef.CENTER_Y
            ).minus(robot_position).getAngle();
        /* Get angle to the reef (in rotations), add 1 to remove negatives and 1/12 to shift by half a face. 
           Round and multiply by 6 to get face # */
        return (int) Math.floor(6 * ((reef_angle.getRotations() + 1 + 0.5 / Reef.NUM_SIDES) % 1));
    }

    public Pose2d getNearestReefPose() {
        SmartDashboard.putNumber("reefclosest", getNearestReefFace());
        return FieldPositions.reef[getNearestReefFace()];
    }

    public Pose2d getNearestSourcePose() {
        if (robot_pose_supplier.get().getY() >= 0 && robot_pose_supplier.get().getY() <= Constants.FieldConstants.WIDTH / 2){
            return FieldPositions.source[0];
        }else{
            return FieldPositions.source[1];
        }
    }

    /**
     * Applies velocity and acceleration constraints to desired speeds.
     * @param target_speeds The desired robot-relative chassis speeds.
     * @return The constrained robot-relative chassis speeds.
     */
    public ChassisSpeeds applyMovementConstraints(ChassisSpeeds target_speeds) {
        ChassisSpeeds current_speeds = robot_speeds_supplier.get();
        // Redefine vectors as Translation2d's for easier math
        Translation2d current_speeds_vector = new Translation2d(
            current_speeds.vxMetersPerSecond,
            current_speeds.vyMetersPerSecond);

        Translation2d target_speeds_vector = new Translation2d(
            target_speeds.vxMetersPerSecond,
            target_speeds.vyMetersPerSecond);

        // Limit velocity to max speed
        current_speeds_vector = current_speeds_vector.times(
            Math.min(1, Tracking.MAX_SPEED / current_speeds_vector.getNorm()));

        // Calculate acceleration and reset requested speeds based on max acceleration
        Translation2d accel_vector = target_speeds_vector.minus(current_speeds_vector);
        accel_vector = accel_vector.times(
            Math.min(1, Tracking.MAX_ACCEL * RobotConstants.LOOP_TIME_SECONDS / accel_vector.getNorm()));
        Translation2d out_speeds_vector = current_speeds_vector.plus(accel_vector);

        // Clamp angular velocity to maximum angular speed
        double target_ang_vel = RobotUtils.clampAbs(target_speeds.omegaRadiansPerSecond, Tracking.MAX_ANG_SPEED);
        // Calculate new angular velocity with acceleration constraint
        double out_ang_vel = current_speeds.omegaRadiansPerSecond
             + RobotUtils.clampAbs(target_ang_vel - current_speeds.omegaRadiansPerSecond,
                Tracking.MAX_ANG_ACCEL * RobotConstants.LOOP_TIME_SECONDS);

        return new ChassisSpeeds(out_speeds_vector.getX(), out_speeds_vector.getY(), out_ang_vel);
    }

    /**
     * Calculate the chassis speeds to position PID to the target pose. Points towards the target.
     * @return The calculated chassis speeds.
     */
    public ChassisSpeeds calculate() {
        return calculate(Rotation2d.kZero, false);
    }

    /**
     * Calculate the chassis speeds to position PID to the target pose.
     * @param useTargetAngle Whether or not to use the rotation value from the stored target pose.
     * @return The calculated chassis speeds.
     */
    public ChassisSpeeds calculate(boolean useTargetAngle) {
        if (useTargetAngle) return calculate(Rotation2d.kZero, true);
        else return calculate();
    }

    /**
     * Calculate the chassis speeds to position PID to the target pose. Accepts a modifier to be applied to the angle.
     * @param angleModifier A function callback to modify the target angle, which points towards the target by default.
     * @return The calculated chassis speeds.
     */
    public ChassisSpeeds calculate(Rotation2d rotationOffset, boolean useTargetAngle) {
        Pose2d robot_pose = robot_pose_supplier.get();

        this.rotation_offset = rotationOffset;

        // Instantiate goal pose at the target pointing in our desired goal-to-target angle
        Pose2d goal_pose = new Pose2d(
            target_pose.getTranslation(),
            useTargetAngle ? target_pose.getRotation() : 
                target_pose.getTranslation().minus(robot_pose.getTranslation()).getAngle());

        // Update the angular PID controller with new measurements and calculate desired angular speed
        double desired_ang_speed = ang_pid_controller.calculate(
            robot_pose.getRotation().getRadians(), 
            goal_pose.getRotation().plus(rotation_offset).getRadians());

        /* Shift the pose backwards by the target distance using a -x transform
           Add extra space for turning to goal distance if robot is not pointing towards target,
           with a small amount of tolerance */
        goal_pose = goal_pose.transformBy(new Transform2d(
                -(target_distance + Math.abs(MathUtil.applyDeadband(
                    // Multiply by two because the maximum error, facing backwards, is 0.5 rotations
                    Units.radiansToRotations(ang_pid_controller.getError()) * 2,
                    Tracking.ALIGN_SEPARATION_TOLERANCE,
                    Tracking.MAX_ALIGN_SEPARATION))),
                0,
                // Apply the rotational offset
                rotationOffset
            )
        );

        adv_goal_pose_pub.set(goal_pose);

        // Find goal to robot translation
        Translation2d goal_off = robot_pose.getTranslation().minus(goal_pose.getTranslation())
            .rotateBy(robot_pose.getRotation().unaryMinus());

        // Get PID forward speed and normalize
        goal_off = goal_off.times(dist_pid_controller.calculate(goal_off.getNorm(), 0) / goal_off.getNorm());

        return applyMovementConstraints(new ChassisSpeeds(
                goal_off.getX(),
                goal_off.getY(),
                desired_ang_speed));

    }

}