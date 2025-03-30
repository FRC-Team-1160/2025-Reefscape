package frc.robot;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.Map;
import java.util.HashMap;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.AutoConstants.Paths;
import frc.robot.Subsystems.DriveTrain.DriveTrain;
import frc.robot.Subsystems.Elevator.Elevator;
import frc.robot.Subsystems.Elevator.Elevator.TargetState;
import frc.robot.Constants.FieldConstants.AlgaeProcessor;
import frc.robot.Constants.FieldConstants.CoralStation;
import frc.robot.Constants.FieldConstants.Reef;
import frc.robot.Constants.FieldConstants.Barge;
import frc.robot.Constants.RobotConstants;

public class FieldHandler {

    public static FieldHandler instance = new FieldHandler();

    public List<SendableChooser<AutoPos>> auto_menus;

    public Field2d auto_preview;

    /** Stores the poses of alignment targets on the field. */
    public static class FieldPositions {
        static Pose2d[] reef, source;
        static Pose2d processor;
    }

    public enum AutoPos {
        kEnd("End", () -> Pose2d.kZero, Rotation2d.kZero, 0),

        kCage1("Cage 1", () -> new Pose2d(Paths.START_X, Barge.CAGE_1, Rotation2d.kPi),
            Rotation2d.kPi, Paths.BARGE_CONTROL),
        kCage2("Cage 2", () -> new Pose2d(Paths.START_X, Barge.CAGE_2, Rotation2d.kPi),
            Rotation2d.kPi, Paths.BARGE_CONTROL),
        kCage3("Cage 3", () -> new Pose2d(Paths.START_X, Barge.CAGE_3, Rotation2d.kPi), 
            Rotation2d.kPi, Paths.BARGE_CONTROL),
        kBargeMiddle("Barge Middle", () -> new Pose2d(Paths.START_X, Barge.MIDDLE, Rotation2d.kPi), 
            Rotation2d.kPi, Paths.BARGE_CONTROL),
        kCage4("Cage 4", () -> new Pose2d(Paths.START_X, Barge.CAGE_4, Rotation2d.kPi), 
            Rotation2d.kPi, Paths.BARGE_CONTROL),
        kCage5("Cage 5", () -> new Pose2d(Paths.START_X, Barge.CAGE_5, Rotation2d.kPi), 
            Rotation2d.kPi, Paths.BARGE_CONTROL),
        kCage6("Cage 6", () -> new Pose2d(Paths.START_X, Barge.CAGE_6, Rotation2d.kPi), 
            Rotation2d.kPi, Paths.BARGE_CONTROL),

        kReef2L("2 Left", () -> FieldPositions.reef[RobotUtils.isRedAlliance() ? 15 : 6], 
            Rotation2d.fromDegrees(RobotUtils.isRedAlliance() ? 120 : -60), Paths.REEF_CONTROL_CLOSE),
        kReef2R("2 Right", () -> FieldPositions.reef[RobotUtils.isRedAlliance() ? 17 : 8], 
            Rotation2d.fromDegrees(RobotUtils.isRedAlliance() ? 120 : -60), Paths.REEF_CONTROL_CLOSE),
        kReef4L("4 Left", () -> FieldPositions.reef[RobotUtils.isRedAlliance() ? 12 : 3], 
            Rotation2d.fromDegrees(RobotUtils.isRedAlliance() ? 60 : -120), Paths.REEF_CONTROL_FAR),
        kReef4R("4 Right", () -> FieldPositions.reef[RobotUtils.isRedAlliance() ? 14 : 5], 
            Rotation2d.fromDegrees(RobotUtils.isRedAlliance() ? 60 : -120), Paths.REEF_CONTROL_FAR),
        kReef6L("6 Left", () -> FieldPositions.reef[RobotUtils.isRedAlliance() ? 9 : 0], 
            Rotation2d.fromDegrees(RobotUtils.isRedAlliance() ? 0 : 180), Paths.REEF_CONTROL_FAR),
        kReef6R("6 Right", () -> FieldPositions.reef[RobotUtils.isRedAlliance() ? 11 : 2], 
            Rotation2d.fromDegrees(RobotUtils.isRedAlliance() ? 0 : 180), Paths.REEF_CONTROL_FAR),
        kReef8L("8 Left", () -> FieldPositions.reef[RobotUtils.isRedAlliance() ? 6 : 15], 
            Rotation2d.fromDegrees(RobotUtils.isRedAlliance() ? -60 : 120), Paths.REEF_CONTROL_FAR),
        kReef8R("8 Right", () -> FieldPositions.reef[RobotUtils.isRedAlliance() ? 8 : 17], 
            Rotation2d.fromDegrees(RobotUtils.isRedAlliance() ? -60 : 120), Paths.REEF_CONTROL_FAR),
        kReef10L("10 Left", () -> FieldPositions.reef[RobotUtils.isRedAlliance() ? 3 : 12], 
            Rotation2d.fromDegrees(RobotUtils.isRedAlliance() ? -120 : 60), Paths.REEF_CONTROL_CLOSE),
        kReef10R("10 Right", () -> FieldPositions.reef[RobotUtils.isRedAlliance() ? 5 : 14], 
            Rotation2d.fromDegrees(RobotUtils.isRedAlliance() ? -120 : 60), Paths.REEF_CONTROL_CLOSE),
        kReef12L("12 Left", () -> FieldPositions.reef[RobotUtils.isRedAlliance() ? 0 : 9], 
            Rotation2d.fromDegrees(RobotUtils.isRedAlliance() ? 180 : 0), Paths.REEF_CONTROL_CLOSE),
        kReef12R("12 Right", () -> FieldPositions.reef[RobotUtils.isRedAlliance() ? 2 : 11], 
            Rotation2d.fromDegrees(RobotUtils.isRedAlliance() ? 180 : 0), Paths.REEF_CONTROL_CLOSE),

        kSourceL("Source Left", () -> FieldPositions.source[1].transformBy(new Transform2d(0, 0, Rotation2d.kPi)),
            Rotation2d.fromRadians(FieldConstants.CoralStation.ANGLE_RADIANS).unaryMinus(), Paths.SOURCE_CONTROL),      
        kSourceR("Source Right", () -> FieldPositions.source[0].transformBy(new Transform2d(0, 0, Rotation2d.kPi)),
            Rotation2d.fromRadians(FieldConstants.CoralStation.ANGLE_RADIANS), Paths.SOURCE_CONTROL);

        public String name;
        public Pose2d pose;
        public Supplier<Pose2d> pose_getter;
        public Rotation2d rotation;
        public double control_length;
        public Waypoint start_waypoint, end_waypoint;

        private AutoPos(String name, Supplier<Pose2d> pose_getter, Rotation2d heading, double control_length) {
            this.name = name;
            this.pose_getter = pose_getter;
            this.rotation = heading;
            this.control_length = control_length;
        }

        public void compose() {
            pose = new Pose2d(pose_getter.get().getX(), pose_getter.get().getY(), rotation);
            rotation = pose_getter.get().getRotation();
        }

        public Pose2d getActualPose() {
            return new Pose2d(pose.getTranslation(), rotation);
        }

        public Waypoint getWaypoint(boolean start) {
            return new Waypoint(
                start ? null : pose.getTranslation().plus(
                    new Translation2d(Paths.ALIGN_DISTANCE + control_length, 0).rotateBy(pose.getRotation())),
                pose.getTranslation().plus(
                    new Translation2d(start ? 0 : Paths.ALIGN_DISTANCE, 0).rotateBy(pose.getRotation())), 
                start ? pose.getTranslation().plus(
                        new Translation2d(Paths.ALIGN_DISTANCE + control_length, 0).rotateBy(pose.getRotation())) : null
            );
        }
    }

    private static AutoPos[] auto_positions_start;

    private static Map<AutoPos, AutoPos[]> auto_positions_map;

    private FieldHandler() {
        auto_preview = new Field2d();
        auto_menus = new ArrayList<SendableChooser<AutoPos>>(Collections.nCopies(4, new SendableChooser<AutoPos>()));
        fillFieldPositions();
        for (AutoPos pos : AutoPos.values()) pos.compose();
        fillAutoPositions();
        rebuildAutoMenus();
        SmartDashboard.putData("Auto Preview", auto_preview);

    }

    private void fillAutoPositions() {
        auto_positions_start = new AutoPos[] {
            AutoPos.kCage1, 
            AutoPos.kCage2, 
            AutoPos.kCage3, 
            AutoPos.kBargeMiddle, 
            AutoPos.kCage4, 
            AutoPos.kCage5, 
            AutoPos.kCage6
        };

        auto_positions_map = new HashMap<AutoPos, AutoPos[]>(Map.of(
            AutoPos.kCage1, new AutoPos[] {
                AutoPos.kReef10L, AutoPos.kReef10R, AutoPos.kReef8L, AutoPos.kReef8R, AutoPos.kReef6L, AutoPos.kReef6R },
            AutoPos.kCage2, new AutoPos[] {
                AutoPos.kReef10L, AutoPos.kReef10R, AutoPos.kReef8L, AutoPos.kReef8R },
            AutoPos.kCage3, new AutoPos[] {
                AutoPos.kReef10L, AutoPos.kReef10R, AutoPos.kReef12R },
            AutoPos.kBargeMiddle, new AutoPos[] {
                AutoPos.kReef12L, AutoPos.kReef12R },
            AutoPos.kCage4, new AutoPos[] {
                AutoPos.kReef2L, AutoPos.kReef2R, AutoPos.kReef12L },
            AutoPos.kCage5, new AutoPos[] {
                AutoPos.kReef2L, AutoPos.kReef2R, AutoPos.kReef4L, AutoPos.kReef4R },
            AutoPos.kCage6, new AutoPos[] {
                AutoPos.kReef2L, AutoPos.kReef2R, AutoPos.kReef4L, AutoPos.kReef4R, AutoPos.kReef6L, AutoPos.kReef6R },

            AutoPos.kReef12L, new AutoPos[] {},
            AutoPos.kReef12R, new AutoPos[] {},

            AutoPos.kEnd, new AutoPos[] {}
        ));

        AutoPos[] reef_pos_list = new AutoPos[] {
            AutoPos.kReef2L, AutoPos.kReef2R,
            AutoPos.kReef4L, AutoPos.kReef4R,
            AutoPos.kReef6L, AutoPos.kReef6R,
            AutoPos.kReef8L, AutoPos.kReef8R,
            AutoPos.kReef10L, AutoPos.kReef10R,
        };

        for (AutoPos reef_pos : reef_pos_list) 
            auto_positions_map.put(reef_pos, new AutoPos[] {AutoPos.kSourceL, AutoPos.kSourceR});

        auto_positions_map.put(AutoPos.kSourceL, reef_pos_list);
        auto_positions_map.put(AutoPos.kSourceR, reef_pos_list);

        auto_positions_map = Map.copyOf(auto_positions_map);

    }

    private void fillFieldPositions() {
        FieldPositions.reef = new Pose2d[Reef.NUM_SIDES * 3];
        Rotation2d angle = Rotation2d.kPi;
        for (int i = 0; i < Reef.NUM_SIDES; i++) {
            Pose2d center = new Pose2d(
                Reef.CENTER_X,
                Reef.CENTER_Y,
                angle).plus(new Transform2d(
                        Reef.INNER_RADIUS + RobotConstants.BASE_WIDTH / 2,
                        0,
                        Rotation2d.kPi
                    )
                );
            FieldPositions.reef[3*i + 1] = center;
            FieldPositions.reef[3*i] = center.plus(new Transform2d(0, 0.15, Rotation2d.kZero)); //0.18
            FieldPositions.reef[3*i + 2] = center.plus(new Transform2d(0, -0.18, Rotation2d.kZero)); //-0.15
            angle = angle.plus(Rotation2d.fromRotations(1.0 / Reef.NUM_SIDES));
        }

        FieldPositions.source = new Pose2d[] {
            // right source
            new Pose2d(
                CoralStation.CENTER_X,
                CoralStation.CENTER_Y,
                Rotation2d.fromRadians(CoralStation.ANGLE_RADIANS).plus(Rotation2d.kPi)
            ).plus(new Transform2d(-RobotConstants.BASE_WIDTH / 2, 0.5, Rotation2d.kZero)),
            // left source
            new Pose2d(
                CoralStation.CENTER_X,
                FieldConstants.WIDTH - CoralStation.CENTER_Y,
                Rotation2d.fromRadians(-CoralStation.ANGLE_RADIANS)
            ).plus(new Transform2d(RobotConstants.BASE_WIDTH / 2, 0.5, Rotation2d.kPi))
        };

        FieldPositions.processor = new Pose2d(
            AlgaeProcessor.CENTER_X,
            AlgaeProcessor.CENTER_Y,
            Rotation2d.fromRadians(AlgaeProcessor.ANGLE_RADIANS)
        ).plus(new Transform2d(RobotConstants.BASE_WIDTH / 2, 0, Rotation2d.kPi));
    }

    public void autoMenuUpdateCallback(int index) {
        new RunCommand(() -> updateGenericChooser(index)).until(() -> true)
            .ignoringDisable(true).schedule();
    }

    public void rebuildAutoMenus() {
        auto_menus.set(0, new SendableChooser<AutoPos>());

        for (AutoPos pos : auto_positions_start) auto_menus.get(0).addOption(pos.name, pos);

        auto_menus.get(0).setDefaultOption(AutoPos.kBargeMiddle.name, AutoPos.kBargeMiddle);
        auto_menus.get(0).onChange(pos -> autoMenuUpdateCallback(1));

        SmartDashboard.putData("Auto Start", auto_menus.get(0));
    }

    public void updateGenericChooser(int index) {
        auto_menus.set(index, new SendableChooser<AutoPos>());

        if (auto_menus.get(index - 1).getSelected() == null) return;

        AutoPos[] opts = auto_positions_map.get(auto_menus.get(index - 1).getSelected());
        auto_menus.get(index).setDefaultOption(AutoPos.kEnd.name, AutoPos.kEnd);

        for (AutoPos pos : opts) auto_menus.get(index).addOption(pos.name, pos);

        if (index + 1 < auto_menus.size()) auto_menus.get(index).onChange(pos -> autoMenuUpdateCallback(index + 1));
        else auto_menus.get(index).onChange(pos -> updatePreview());

        String key = "Auto " + (index % 2 == 0 ? "Source " : "Reef ") + String.valueOf((index + 1) / 2);
        if (!SmartDashboard.containsKey(key)) SmartDashboard.putData(key, auto_menus.get(index));
    }

    /**
     * Generates a bezier curve path between two positions for autonomous.
     * @param start The starting position.
     * @param end The ending position.
     * @return The generated PathPlannerPath.
     */
    public PathPlannerPath getPath(AutoPos start, AutoPos end) {
        // Generate PathPlanner Waypoints
        Waypoint start_point = start.getWaypoint(true);
        Waypoint end_point = end.getWaypoint(false);

        Translation2d reef_center = new Translation2d(FieldConstants.Reef.CENTER_X, FieldConstants.Reef.CENTER_Y);
        // Angles the reef to each endpoint
        double end_angle = end_point.anchor().minus(reef_center).getAngle().getRotations();
        double start_angle = start_point.anchor().minus(reef_center).getAngle().getRotations();

        Waypoint mid_point = null;
        // Add an extra control point in the middle if the path must go around the reef
        boolean add_midpoint = Math.abs(0.5 - Math.abs(end_angle - start_angle)) < 0.5 - 2.0 / Reef.NUM_SIDES;

        if (add_midpoint) {
            Pose2d mid_pose = new Pose2d(
                reef_center.plus(new Translation2d(
                        /* Position new control point with distance to reef being the harmonic mean of the distances of
                         * other two control points. */
                        Math.sqrt(end_point.anchor().getDistance(reef_center) * start_point.anchor().getDistance(reef_center)), 
                        0
                        // Control point bisects the angle between two points and reef center 
                    ).rotateBy(Rotation2d.fromRotations((start_angle + end_angle) / 2.0
                         + (Math.abs(end_angle - start_angle) > 0.5 ? 0.5 : 0)))),
                // Draw control handles roughly parallel to the straight line between start and end points
                end_point.anchor().plus(end_point.prevControl()).minus(
                    start_point.anchor().plus(start_point.nextControl())).getAngle()
            );

            // Generate control handle points by transform the previously orientated pose forward and backward
            mid_point = new Waypoint(
                mid_pose.transformBy(new Transform2d(
                    -end_point.anchor().getDistance(start_point.anchor()) / 6,
                    0, 
                    Rotation2d.kZero)).getTranslation(), 
                mid_pose.getTranslation(), 
                mid_pose.transformBy(new Transform2d(
                    end_point.anchor().getDistance(start_point.anchor()) / 6, 
                    0,
                    Rotation2d.kZero)).getTranslation());
        }

        return new PathPlannerPath(
            add_midpoint ? Arrays.asList(start_point, mid_point, end_point) : Arrays.asList(start_point, end_point),
            new PathConstraints(
                AutoConstants.MAX_SPEED, 
                AutoConstants.MAX_ACCEL, 
                AutoConstants.MAX_ANG_SPEED, 
                AutoConstants.MAX_ANG_ACCEL), 
            new IdealStartingState(0, start.rotation), 
            new GoalEndState(1.5, end.rotation));
    }

    public void updatePreview() {

        for (SendableChooser<AutoPos> menu : auto_menus) if (menu == null || menu.getSelected() == null) return;

        List<Pose2d> preview_poses = new ArrayList<Pose2d>(); 
        List<PathPlannerTrajectoryState> pp_states = new ArrayList<PathPlannerTrajectoryState>();

        for (int i = 0; i + 1 < auto_menus.size(); i++) {

            if (auto_menus.get(i + 1).getSelected() == AutoPos.kEnd) break;

            var temp = getPath(auto_menus.get(i).getSelected(), auto_menus.get(i + 1).getSelected())
                .generateTrajectory(
                    new ChassisSpeeds(), auto_menus.get(0).getSelected().rotation, 
                    DriveTrain.instance.config
                ).getStates();

            pp_states.addAll(temp);

            for (int d = 1; d <= AutoConstants.PREVIEW_DETAIL; d++) {
                preview_poses.add(temp.get(temp.size() * d / (AutoConstants.PREVIEW_DETAIL + 1)).pose);
            }

            preview_poses.add(auto_menus.get(i + 1).getSelected().getActualPose());
        }

        List<Trajectory.State> traj_states = new ArrayList<Trajectory.State>();

        for (PathPlannerTrajectoryState pp_state : pp_states) {
            traj_states.add(new Trajectory.State(
                pp_state.timeSeconds, 
                pp_state.linearVelocity, 
                0, 
                pp_state.pose, 
                0));
        }

        auto_preview.close();
        auto_preview = new Field2d();

        auto_preview.setRobotPose(auto_menus.get(0).getSelected().getActualPose());
        
        if (traj_states.size() > 0) auto_preview.getObject("auto_traj").setTrajectory(new Trajectory(traj_states));
        else auto_preview.getObject("auto_traj").setTrajectory(new Trajectory());

        for (int i = 0; i < auto_menus.size() * (AutoConstants.PREVIEW_DETAIL + 1); i += 8) {
            auto_preview.getObject("poses_" + String.valueOf(i / 8))
                .setPoses(i < preview_poses.size()
                     ? preview_poses.subList(i, Math.min(i + 8, preview_poses.size()))
                     : new ArrayList<Pose2d>()
                );
        }

        Logger.recordOutput("FieldHandler/pose", auto_preview.getRobotPose());

        SmartDashboard.putData("Auto Preview", auto_preview);
        auto_preview.setRobotPose(auto_menus.get(0).getSelected().getActualPose());
    }

    public Command buildAuto() {
        // Initialize command group
        SequentialCommandGroup sequence = new SequentialCommandGroup();
        // Pre-allocate AutoPos's
        AutoPos start;
        AutoPos end = auto_menus.get(0).getSelected();
        // Reset the pose_estimator pose to the auto starting position
        SubsystemManager.instance.pose_estimator.resetPose(end.getActualPose());
        for (int i = 0; i + 1 < auto_menus.size(); i++) {
            // The end enum from the previous section becomes the start for the next
            start = end;
            end = auto_menus.get(i + 1).getSelected();

            if (end == AutoPos.kEnd) break;

            // Generate the path between two anchor points
            PathPlannerPath path = getPath(start, end);

            TargetState target = end.name().contains("Reef") ? TargetState.kL4 : TargetState.kSource;

            sequence.addCommands(
                // Run two command sequences in parallel
                new ParallelCommandGroup(
                    // Sequence 1: Following a PathPlanner curve into automatic alignment
                    new SequentialCommandGroup(
                        SubsystemManager.instance.commands.decoratePathplannerCmd(AutoBuilder.followPath(path)),
                        // Select alignment target based on elevator state
                        SubsystemManager.instance.commands.selectCommand(),
                        new WaitCommand(0.5)),
                    // Sequence 2: Wait for some time, then move the elevator to desired position
                    new SequentialCommandGroup(
                        // Wait longer if elevator is low, bring down quickly if elevator is high
                        new WaitCommand(target != TargetState.kL4 ? 0.5 : 
                            path.generateTrajectory(
                                new ChassisSpeeds(), 
                                start.rotation, 
                                DriveTrain.instance.config
                            ).getTotalTimeSeconds() - 1.5),
                        Elevator.instance.setStateCmd(target))),
                // Post-path command, intaking or shooting depending on elevator state
                target == TargetState.kSource ? Elevator.instance.intakeCoralCmd()
                     : RobotUtils.onOffCommand(Elevator.instance::runShooter, 0.3).withTimeout(1)
            );
        }
        sequence.addCommands(Elevator.instance.setStateCmd(TargetState.kSource));
        return sequence;
    }
}