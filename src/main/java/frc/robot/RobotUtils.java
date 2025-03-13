package frc.robot;

import java.util.function.Consumer;

import org.littletonrobotics.junction.AutoLogOutput;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.Unit;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.RobotConstants.ComponentZeroPoses;

public final class RobotUtils {
    
    public static double hypot(double a, double b) {
        return Math.sqrt(a * a + b * b);
    }

    public static double hypot_inverse(double c, double a) {
        return Math.sqrt(c * c - a * a);
    }

    public static double clampAbs(double x, double maxAbs) {
        return MathUtil.clamp(x, -maxAbs, maxAbs);
    }

    public static boolean isRedAlliance() {
        if (DriverStation.getAlliance().isEmpty()) return false;
        return DriverStation.getAlliance().get() == Alliance.Red;
    }

    public static double allianceFlipX(double x) {
        return isRedAlliance() ? FieldConstants.LENGTH - x : x;
    }

    public static double allianceFlipY(double y) {
        return isRedAlliance() ? FieldConstants.WIDTH - y : y;
    }

    public static double allianceNegate(double x) {
        return isRedAlliance() ? -x : x;
    }

    public static Command onOffCommand(Consumer<Double> consumer, double on_value, double off_value) {
        return new StartEndCommand(() -> consumer.accept(on_value), () -> consumer.accept(off_value));
    }

    public static Command onOffCommand(Consumer<Double> consumer, double value) {
        return onOffCommand(consumer, value, 0);
    }

    public static Command decorateCommandFeedback(Command command, Runnable feedback) {
        return command.finallyDo(end -> {if (!end) feedback.run();});
    }

    public record ArticulatedPose(Pose2d robot_pose, Pose3d[] component_poses) {

        private final static Pose3d[] zero_poses = new Pose3d[] {
            ComponentZeroPoses.ELEVATOR_STAGE, 
            ComponentZeroPoses.CARRIAGE, 
            ComponentZeroPoses.ALGAE_INTAKE, 
            ComponentZeroPoses.FUNNEL
        };

        public ArticulatedPose(Pose2d robot_pose, Pose3d pose1, Pose3d pose2, Pose3d pose3, Pose3d pose4) {
            this(robot_pose, new Pose3d[] {pose1, pose2, pose3, pose4});
        }

        public ArticulatedPose(Pose2d robot_pose, double elevator_height, double intake_angle) {
            this(
                robot_pose,
                zero_poses[0].plus(new Transform3d(0, 0, elevator_height * ElevatorConstants.GEAR_DIAMETER, Rotation3d.kZero)),
                zero_poses[1].plus(new Transform3d(0, 0, elevator_height * ElevatorConstants.GEAR_DIAMETER * 2, Rotation3d.kZero)),
                // zero_poses[2].plus(new Transform3d(0, 0, 0, new Rotation3d(0, -intake_angle, 0)))
                    // .plus(new Transform3d(0, 0, elevator_height * ElevatorConstants.GEAR_DIAMETER * 2, Rotation3d.kZero)),
                zero_poses[2].plus(new Transform3d(0, 0, elevator_height * ElevatorConstants.GEAR_DIAMETER * 2, new Rotation3d(0, -intake_angle, 0))),
                zero_poses[3]);
        }

        public ArticulatedPose(Pose2d robot_pose) {
            this(robot_pose, zero_poses);
        }

        public void publish(StructPublisher<Pose2d> pose_pub, StructArrayPublisher<Pose3d> components_pub) {
            pose_pub.set(robot_pose);
            components_pub.set(component_poses);
        }
    }

}
