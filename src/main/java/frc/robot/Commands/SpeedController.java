package frc.robot.Commands;

import java.util.function.Consumer;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public class SpeedController extends Controller {
    // final??? maybe
    // edge case to be added to style guide
    TrapezoidProfile.Constraints X_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 3);
    TrapezoidProfile.Constraints Y_CONSTRAINTS = new TrapezoidProfile.Constraints(3, 3);
    TrapezoidProfile.Constraints OMEGA_CONSTRAINTS = new TrapezoidProfile.Constraints(Math.toRadians(40),
            Math.toRadians(20));

    ProfiledPIDController x_controller = new ProfiledPIDController(1, 0, 0, X_CONSTRAINTS);
    ProfiledPIDController y_controller = new ProfiledPIDController(1, 0, 0, Y_CONSTRAINTS);
    ProfiledPIDController omega_controller = new ProfiledPIDController(1, 0, 0, OMEGA_CONSTRAINTS);

    Consumer<ChassisSpeeds> setChassisSpeeds;
    Supplier<Pose2d> getPose;
    Supplier<Pose2d> getClosestTarget;

    public SpeedController(Consumer<ChassisSpeeds> setChassisSpeeds, Supplier<Pose2d> getPose,
            Supplier<Pose2d> getClosestTarget) {
        x_controller.setTolerance(0.1);
        y_controller.setTolerance(0.1);
        omega_controller.setTolerance(Units.degreesToRadians(5));
        omega_controller.enableContinuousInput(-Math.PI, Math.PI);

        this.setChassisSpeeds = setChassisSpeeds;
        this.getPose = getPose;
        this.getClosestTarget = getClosestTarget;
    }

    public void init() {
        Pose2d robot_pose = getPose.get();
        x_controller.reset(robot_pose.getX());
        y_controller.reset(robot_pose.getY());
        omega_controller.reset(robot_pose.getRotation().getRadians());
    }

    public void exec() {
        if (getClosestTarget.get() == null) {
            setChassisSpeeds.accept(new ChassisSpeeds());
            return;
        }

        Pose2d robot_pose = getPose.get();
        Pose2d object_pose = getClosestTarget.get();

        double x_dist = object_pose.getX() - robot_pose.getX();
        double y_dist = object_pose.getY() - robot_pose.getY();

        double target_angle = Math.atan2(y_dist, x_dist);

        double dist = Math.sqrt(Math.pow(x_dist, 2) + Math.pow(y_dist, 2));

        Pose2d goal_pose = new Pose2d( // no likey transform2d
            object_pose.getX() - 0.8 * x_dist / dist,
            object_pose.getY() - 0.8 * y_dist / dist,
            Rotation2d.fromRadians(target_angle)
        );

        x_controller.setGoal(goal_pose.getX());
        y_controller.setGoal(goal_pose.getY());
        omega_controller.setGoal(goal_pose.getRotation().getRadians());

        double x_speed = x_controller.calculate(robot_pose.getX());
        double y_speed = y_controller.calculate(robot_pose.getY());
        double omega_speed = omega_controller.calculate(robot_pose.getRotation().getRadians());

        if (x_controller.atSetpoint()) x_speed = 0;
        if (y_controller.atSetpoint()) y_speed = 0;
        if (omega_controller.atSetpoint()) omega_speed = 0;

        setChassisSpeeds.accept(new ChassisSpeeds(x_speed, y_speed, omega_speed));
    }

    public boolean is_fin() {
        return false;
    }

    public void exit(boolean interrupted) {
        setChassisSpeeds.accept(new ChassisSpeeds());
    }
}