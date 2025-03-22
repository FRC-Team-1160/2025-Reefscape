package frc.robot;

import java.io.IOException;
import java.time.Instant;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.HashMap;
import java.util.Set;
import java.util.function.Supplier;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.RobotConfig;
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
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.AutoConstants.Paths;
import frc.robot.Subsystems.Elevator.Elevator;
import frc.robot.Subsystems.Elevator.Elevator.TargetState;
import frc.robot.Constants.FieldConstants.AlgaeProcessor;
import frc.robot.Constants.FieldConstants.CoralStation;
import frc.robot.Constants.FieldConstants.Reef;
import frc.robot.Constants.FieldConstants.Barge;
import frc.robot.Constants.RobotConstants;

public class FieldHandler {

    public static FieldHandler instance = new FieldHandler();

    public SendableChooser<AutoPos>[] auto_menus;

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

        kReef2L("Reef 2 Left", () -> FieldPositions.reef[RobotUtils.isRedAlliance() ? 15 : 6], 
            Rotation2d.fromDegrees(RobotUtils.isRedAlliance() ? 120 : -60), Paths.REEF_CONTROL),
        kReef2R("Reef 2 Right", () -> FieldPositions.reef[RobotUtils.isRedAlliance() ? 17 : 8], 
            Rotation2d.fromDegrees(RobotUtils.isRedAlliance() ? 120 : -60), Paths.REEF_CONTROL),
        kReef4L("Reef 4 Left", () -> FieldPositions.reef[RobotUtils.isRedAlliance() ? 12 : 3], 
            Rotation2d.fromDegrees(RobotUtils.isRedAlliance() ? 60 : -120), Paths.REEF_CONTROL),
        kReef4R("Reef 4 Right", () -> FieldPositions.reef[RobotUtils.isRedAlliance() ? 14 : 5], 
            Rotation2d.fromDegrees(RobotUtils.isRedAlliance() ? 60 : -120), Paths.REEF_CONTROL),
        kReef6L("Reef 6 Left", () -> FieldPositions.reef[RobotUtils.isRedAlliance() ? 9 : 0], 
            Rotation2d.fromDegrees(RobotUtils.isRedAlliance() ? 0 : 180), Paths.REEF_CONTROL),
        kReef6R("Reef 6 Right", () -> FieldPositions.reef[RobotUtils.isRedAlliance() ? 11 : 2], 
            Rotation2d.fromDegrees(RobotUtils.isRedAlliance() ? 0 : 180), Paths.REEF_CONTROL),
        kReef8L("Reef 8 Left", () -> FieldPositions.reef[RobotUtils.isRedAlliance() ? 6 : 15], 
            Rotation2d.fromDegrees(RobotUtils.isRedAlliance() ? -60 : 120), Paths.REEF_CONTROL),
        kReef8R("Reef 8 Right", () -> FieldPositions.reef[RobotUtils.isRedAlliance() ? 8 : 17], 
            Rotation2d.fromDegrees(RobotUtils.isRedAlliance() ? -60 : 120), Paths.REEF_CONTROL),
        kReef10L("Reef 10 Left", () -> FieldPositions.reef[RobotUtils.isRedAlliance() ? 3 : 12], 
            Rotation2d.fromDegrees(RobotUtils.isRedAlliance() ? -120 : 60), Paths.REEF_CONTROL),
        kReef10R("Reef 10 Right", () -> FieldPositions.reef[RobotUtils.isRedAlliance() ? 5 : 14], 
            Rotation2d.fromDegrees(RobotUtils.isRedAlliance() ? -120 : 60), Paths.REEF_CONTROL),
        kReef12L("Reef 12 Left", () -> FieldPositions.reef[RobotUtils.isRedAlliance() ? 0 : 9], 
            Rotation2d.fromDegrees(RobotUtils.isRedAlliance() ? 180 : 0), Paths.REEF_CONTROL),
        kReef12R("Reef 12 Right", () -> FieldPositions.reef[RobotUtils.isRedAlliance() ? 2 : 11], 
            Rotation2d.fromDegrees(RobotUtils.isRedAlliance() ? 180 : 0), Paths.REEF_CONTROL),

        kSourceL("Source Left", () -> FieldPositions.source[1].transformBy(new Transform2d(0, 0, Rotation2d.kPi)),
            Rotation2d.fromRadians(FieldConstants.CoralStation.ANGLE_RADIANS).unaryMinus(), Paths.REEF_CONTROL),      
        kSourceR("Source Right", () -> FieldPositions.source[0].transformBy(new Transform2d(0, 0, Rotation2d.kPi)),
            Rotation2d.fromRadians(FieldConstants.CoralStation.ANGLE_RADIANS), Paths.REEF_CONTROL);

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
        auto_menus = new SendableChooser[4];
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
            FieldPositions.reef[3*i] = center.plus(new Transform2d(0, 0.15, Rotation2d.kZero));
            FieldPositions.reef[3*i + 2] = center.plus(new Transform2d(0, -0.19, Rotation2d.kZero));
            angle = angle.plus(Rotation2d.fromRotations(1.0 / Reef.NUM_SIDES));
        }

        FieldPositions.source = new Pose2d[] {
            // right source
            new Pose2d(
                CoralStation.CENTER_X,
                CoralStation.CENTER_Y,
                Rotation2d.fromRadians(CoralStation.ANGLE_RADIANS).plus(Rotation2d.kPi)
            ).plus(new Transform2d(-RobotConstants.BASE_WIDTH / 2, 0, Rotation2d.kZero)),
            // left source
            new Pose2d(
                CoralStation.CENTER_X,
                FieldConstants.WIDTH - CoralStation.CENTER_Y,
                Rotation2d.fromRadians(-CoralStation.ANGLE_RADIANS)
            ).plus(new Transform2d(RobotConstants.BASE_WIDTH / 2, 0, Rotation2d.kPi))
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
        // for (SendableChooser<AutoPos> menu : auto_menus) if (menu != null) menu.close();
        if (auto_menus[0] != null) auto_menus[0].close();
        auto_menus[0] = new SendableChooser<AutoPos>();
        for (AutoPos pos : auto_positions_start) auto_menus[0].addOption(pos.name, pos);
        auto_menus[0].setDefaultOption(AutoPos.kBargeMiddle.name, AutoPos.kBargeMiddle);
        auto_menus[0].onChange(pos -> autoMenuUpdateCallback(1));
        SmartDashboard.putData("Auto Start", auto_menus[0]);

    }

    public void updateGenericChooser(int index) {
        auto_menus[index] = new SendableChooser<AutoPos>();
        if (auto_menus[index - 1].getSelected() == null) return;
        AutoPos[] opts = auto_positions_map.get(auto_menus[index - 1].getSelected());
        auto_menus[index].setDefaultOption(AutoPos.kEnd.name, AutoPos.kEnd);
        if (opts.length > 0) for (AutoPos pos : opts) auto_menus[index].addOption(pos.name, pos);
        if (index + 1 < auto_menus.length) auto_menus[index].onChange(pos -> autoMenuUpdateCallback(index + 1));
        else auto_menus[index].onChange(pos -> updatePreview());
        String key = "Auto Menu " + String.valueOf(index);
        if (!SmartDashboard.containsKey(key)) SmartDashboard.putData(key, auto_menus[index]);
    }

    public PathPlannerPath getPath(AutoPos start, AutoPos end) {
        Waypoint start_point = start.getWaypoint(true);
        Waypoint end_point = end.getWaypoint(false);

        Translation2d reef_center = new Translation2d(FieldConstants.Reef.CENTER_X, FieldConstants.Reef.CENTER_Y);
        double end_angle = end_point.anchor().minus(reef_center).getAngle().getRotations();
        double start_angle = start_point.anchor().minus(reef_center).getAngle().getRotations();

        Waypoint mid_point;

        boolean add_midpoint = Math.abs(0.5 - Math.abs(end_angle - start_angle)) < 0.5 - 2.0 / Reef.NUM_SIDES;
        if (add_midpoint) {
            Pose2d mid_pose = new Pose2d(
                reef_center.plus(new Translation2d(
                        Math.sqrt(end_point.anchor().getDistance(reef_center) * start_point.anchor().getDistance(reef_center)), 
                        0
                    ).rotateBy(Rotation2d.fromRotations((start_angle + end_angle) / 2
                         + (Math.abs(end_angle - start_angle) > 0.5 ? 0.5 : 0)))), 
                end_point.anchor().plus(end_point.prevControl()).minus(
                    start_point.anchor().plus(start_point.nextControl())).getAngle()
            );

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
        } else {
            mid_point = null;
        }
        System.out.println(String.valueOf(end.rotation.getRotations()) + " " + String.valueOf(end.pose.getRotation().getRotations()));
        System.out.println((end.rotation == end.pose.getRotation() ? -1 : 1));
        return new PathPlannerPath(
            add_midpoint ? Arrays.asList(start_point, mid_point, end_point) : Arrays.asList(start_point, end_point),
            new PathConstraints(
                AutoConstants.MAX_SPEED, 
                AutoConstants.MAX_ACCEL, 
                AutoConstants.MAX_ANG_SPEED, 
                AutoConstants.MAX_ANG_ACCEL), 
            new IdealStartingState(0, start.rotation), 
            new GoalEndState(Math.abs(end.rotation.minus(end.pose.getRotation()).getRotations()) < 0.1 ? 0 : 2, end.rotation));
    }

    public void updatePreview() {
        RobotConfig config;
        try {
            config = RobotConfig.fromGUISettings();
        } catch (IOException | ParseException e) {
            e.printStackTrace();
            config = null;
        }
        for (SendableChooser<AutoPos> menu : auto_menus) if (menu == null || menu.getSelected() == null) return;

        List<Pose2d> preview_poses = new ArrayList<Pose2d>(); 
        List<PathPlannerTrajectoryState> pp_states = new ArrayList<PathPlannerTrajectoryState>();
        for (int i = 0; i + 1 < auto_menus.length; i++) {
            if (auto_menus[i + 1].getSelected() == AutoPos.kEnd) break;
            var temp = getPath(auto_menus[i].getSelected(), auto_menus[i + 1].getSelected())
                .generateTrajectory(new ChassisSpeeds(), auto_menus[0].getSelected().rotation, config)
                    .getStates();
            pp_states.addAll(temp);
            for (int d = 1; d <= AutoConstants.PREVIEW_DETAIL; d++) {
                preview_poses.add(temp.get(temp.size() * d / (AutoConstants.PREVIEW_DETAIL + 1)).pose);
            }
            preview_poses.add(auto_menus[i + 1].getSelected().getActualPose());
        }

        List<Trajectory.State> traj_states = new ArrayList<Trajectory.State>();

        for (PathPlannerTrajectoryState pp_state : pp_states) traj_states.add(new Trajectory.State(
                pp_state.timeSeconds, 
                pp_state.linearVelocity, 
                0, 
                pp_state.pose, 
                0));
        
        if (traj_states.size() > 0) auto_preview.getObject("auto_traj").setTrajectory(new Trajectory(traj_states));
        else auto_preview.getObject("auto_traj").setTrajectory(new Trajectory());

        auto_preview.setRobotPose(auto_menus[0].getSelected().getActualPose());

        for (int i = 0; i < preview_poses.size(); i += 10)
            auto_preview.getObject("poses_" + String.valueOf(i / 10))
                .setPoses(preview_poses.subList(i, Math.min(i + 10, preview_poses.size())));
    }

    public Command buildAuto() {
        RobotConfig config;
        try {
            config = RobotConfig.fromGUISettings();
        } catch (IOException | ParseException e) {
            e.printStackTrace();
            config = null;
        }
        SequentialCommandGroup sequence = new SequentialCommandGroup();
        AutoPos start;
        AutoPos end = auto_menus[0].getSelected();
        SubsystemManager.instance.pose_estimator.resetPose(end.getActualPose());
        for (int i = 0; i + 1 < auto_menus.length; i++) {
            start = end;
            end = auto_menus[i + 1].getSelected();
            if (end == AutoPos.kEnd) break;
            PathPlannerPath path = getPath(start, end);
            TargetState target = end.name.contains("Reef") ? TargetState.kL4 : TargetState.kSource;
            sequence.addCommands(
                new ParallelCommandGroup(
                    new SequentialCommandGroup(
                        SubsystemManager.instance.commands.decoratePathplannerCmd(AutoBuilder.followPath(path)),
                        SubsystemManager.instance.commands.selectCommand()),
                    new SequentialCommandGroup(
                        new WaitCommand(target != TargetState.kL4 ? 0.5 : 
                            path.generateTrajectory(
                                new ChassisSpeeds(), 
                                start.rotation, config
                            ).getTotalTimeSeconds() - 1.5),
                        Elevator.instance.setStateCmd(target))),
                target == TargetState.kSource ? Elevator.instance.intakeCoralCmd()
                     : RobotUtils.onOffCommand(Elevator.instance::runShooter, 0.3).withTimeout(0.5)
            );
        }

        return sequence;
    }
}