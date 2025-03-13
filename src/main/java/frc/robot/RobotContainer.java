package frc.robot;

import java.io.IOException;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;

import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.AutoLogOutputManager;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.IOConstants;
import frc.robot.SubsystemManager.PathplannerSpeeds;
import frc.robot.Subsystems.Climber.Climber;
import frc.robot.Subsystems.DriveTrain.DriveTrain;
import frc.robot.Subsystems.Elevator.Elevator;
import frc.robot.Subsystems.Elevator.ElevatorRealIO;
import frc.robot.Subsystems.Elevator.Elevator.TargetState;
import frc.robot.Subsystems.Funnel.Funnel;
import frc.robot.Subsystems.Funnel.Funnel.FunnelState;
import frc.robot.Subsystems.Vision.ObjectDetection;
import frc.robot.Subsystems.Vision.Vision;

/** 
 * RobotContainer handles joystick and controller inputs to the code as well as autonomous sequences.
 * Our use of singleton subsystem classes means that instances are no longer stored here.
 */
public class RobotContainer {

    public record JoystickInputs(double drive_x, double drive_y, double drive_a) {}

    private Joystick main_stick = new Joystick(IOConstants.MAIN_PORT);
    private Joystick second_stick = new Joystick(IOConstants.COPILOT_PORT);
    private Joystick driver_controller = new Joystick(2);
    private Joystick codriver_controller = new Joystick(3);
    private Joystick simp_stick = new Joystick(4);

    private final SendableChooser<Command> auto_chooser;

    public Command rumble() {
        return new StartEndCommand(
            () -> codriver_controller.setRumble(RumbleType.kLeftRumble, 1),
            () -> codriver_controller.setRumble(RumbleType.kBothRumble, 0))
            .withDeadline(Commands.waitSeconds(0.5));
    }

    public class DriveMode {
        /** An enum representing the different ways this robot can be driven. */
        public enum Drive {
            kXBox,
            kJoysticks,
            kSimple
        }
        /** An enum representing the different ways this robot can be controlled. */
        public enum Codrive {
            kXBox,
            kSimple
        }
        public final Drive driver = Drive.kXBox;
        public final Codrive codriver = Codrive.kXBox;
    }

    public DriveMode drive_mode = new DriveMode();

    /** Class constructor. */
    public RobotContainer() {
        // Map.of returns immutable map, so create a mutable hashmap
        HashMap<String, Command> commands_map = new HashMap<String, Command>(Map.of(
            "ElevatorL3", new InstantCommand(() -> Elevator.instance.setState(TargetState.kL3)),
            "ElevatorL4", new InstantCommand(() -> Elevator.instance.setState(TargetState.kL4)),
            "ShootCoral", RobotUtils.onOffCommand(Elevator.instance::runShooter, 0.4).withTimeout(0.5),
            "ReefNearest", SubsystemManager.instance.commands.alignReef(),
            "ReefNearestL4", SubsystemManager.instance.commands.alignReefClose(),
            "ReefSwitchLeft", new InstantCommand(() -> SwervePIDController.instance.align_right = false),
            "ReefSwitchRight", new InstantCommand(() -> SwervePIDController.instance.align_right = true)
        ));

        for (int i = 1; i <= 6; i++) {
            int face = 3 * ((i + 5) % 6);
            commands_map.put("Reef" + String.valueOf(i * 2) + "L2", 
                SubsystemManager.instance.commands.alignReef(face));
            commands_map.put("Reef" + String.valueOf(i * 2) + "R2",
                SubsystemManager.instance.commands.alignReef(face + 2));
            commands_map.put("Reef" + String.valueOf(i * 2) + "L4",
                SubsystemManager.instance.commands.alignReefClose(face));
            commands_map.put("Reef" + String.valueOf(i * 2) + "R4",
                SubsystemManager.instance.commands.alignReefClose(face + 2));
        }

        NamedCommands.registerCommands(commands_map);

        // Configure Autobuilder
        RobotConfig config;

        AutoLogOutputManager.addObject(SubsystemManager.instance);
        AutoLogOutputManager.addObject(SwervePIDController.instance);
        AutoLogOutputManager.addObject(DriveTrain.instance);
        AutoLogOutputManager.addObject(Elevator.instance);
        AutoLogOutputManager.addObject(Funnel.instance);
        AutoLogOutputManager.addObject(Climber.instance);
        AutoLogOutputManager.addObject(Vision.instance);
        AutoLogOutputManager.addObject(ObjectDetection.instance);
        
        try {
            config = RobotConfig.fromGUISettings();
        } catch (IOException | ParseException e) {
            e.printStackTrace();
            config = null;
        }

        AutoBuilder.configure(
            SubsystemManager.instance::getPoseEstimate,
            SubsystemManager.instance.pose_estimator::resetPose,
            DriveTrain.instance::getOdomSpeeds,
            (speeds, feedforwards) -> SubsystemManager.instance.m_pathplanner_speeds = new PathplannerSpeeds(
                    speeds, 
                    feedforwards.accelerationsMPSSq()),
            new PPHolonomicDriveController(
                new PIDConstants(
                    AutoConstants.translation_kP, 
                    AutoConstants.translation_kI,
                    AutoConstants.translation_kD),
                new PIDConstants(
                    AutoConstants.rotation_kP, 
                    AutoConstants.rotation_kI, 
                    AutoConstants.rotation_kD)),
            config,
            () -> RobotUtils.isRedAlliance(),
            DriveTrain.instance // Reference to drive subsystem to set requirements; unfortunately required to instantiate
        );

        auto_chooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", auto_chooser);
        configureBindings();
    }

    public void updateSubsystemManager() {
        switch (drive_mode.driver) {
            case kJoysticks:
                SubsystemManager.instance.update(new JoystickInputs(
                    main_stick.getRawAxis(1), 
                    main_stick.getRawAxis(0), 
                    second_stick.getRawAxis(0)));
                break;
            case kXBox:
                SubsystemManager.instance.update(new JoystickInputs(
                    driver_controller.getRawAxis(1) * (driver_controller.getRawButton(6) ? 0.5 : 1), 
                    driver_controller.getRawAxis(0) * (driver_controller.getRawButton(6) ? 0.5 : 1), 
                    driver_controller.getRawAxis(4) * (driver_controller.getRawButton(6) ? 0.7 : 1)));
                break;
            case kSimple:
                SubsystemManager.instance.update(new JoystickInputs(
                    simp_stick.getRawAxis(1), 
                    simp_stick.getRawAxis(0), 
                    simp_stick.getRawAxis(4)));
                break;
        }
    }

    private void configureBindings() {

        switch (drive_mode.driver) {
            case kJoysticks:
                new JoystickButton(main_stick, 8).onTrue(
                    new InstantCommand(DriveTrain.instance::resetGyroAngle));

                // new JoystickButton(main_stick, 1)
                //     .whileTrue(SubsystemManager.instance.commands.alignReef(false));
                new JoystickButton(main_stick, 1).whileTrue(
                    Commands.defer(SubsystemManager.instance.commands::selectCommand, null));
                    
                // new JoystickButton(second_stick, 1)
                //     .whileTrue(SubsystemManager.instance.commands.trackAlgae());

                break;
            
            case kXBox:
                new JoystickButton(driver_controller, 8).onTrue(
                    new InstantCommand(DriveTrain.instance::resetGyroAngle));

                new JoystickButton(driver_controller, 5)
                    .whileTrue(SubsystemManager.instance.commands.alignReef());

                new JoystickButton(driver_controller, 7).whileTrue(
                    Commands.defer(SubsystemManager.instance.commands::selectCommand, new HashSet<Subsystem>()));
                break;
            case kSimple:
                new JoystickButton(simp_stick, 8).onTrue(
                    new InstantCommand(DriveTrain.instance::resetGyroAngle));
                break;
        }

        switch (drive_mode.codriver) {
            case kXBox:
                // Reef setpoints
                new JoystickButton(codriver_controller, 3).onTrue(Commands.either(
                    Elevator.instance.setStateCmd(TargetState.kProcessor),
                    Elevator.instance.setStateCmd(TargetState.kL1),
                    () -> codriver_controller.getRawButton(6)));

                new JoystickButton(codriver_controller, 1).onTrue(Commands.either(
                    Elevator.instance.setStateCmd(TargetState.kL2Algae),
                    Elevator.instance.setStateCmd(TargetState.kL2),
                    () -> codriver_controller.getRawButton(6)));

                new JoystickButton(codriver_controller, 2).onTrue(Commands.either(
                    Elevator.instance.setStateCmd(TargetState.kL3Algae),
                    Elevator.instance.setStateCmd(TargetState.kL3),
                    () -> codriver_controller.getRawButton(6)));

                new JoystickButton(codriver_controller, 4).onTrue(Commands.either(
                    Elevator.instance.setStateCmd(TargetState.kBarge),
                    Elevator.instance.setStateCmd(TargetState.kL4),
                    () -> codriver_controller.getRawButton(6)));

                new JoystickButton(codriver_controller, 10)
                    .and(new JoystickButton(codriver_controller, 10)).onTrue(
                    new InstantCommand(() -> Elevator.instance.zeroWrist()));

                // Elevator manual
                new Trigger(() -> Math.abs(codriver_controller.getRawAxis(1)) > 0.25).whileTrue(
                    new RunCommand(() -> Elevator.instance.runElevator(0.35 - 1.5 * codriver_controller.getRawAxis(1)))
                        .finallyDo(Elevator.instance::stopElevator));
                
                // Wrist manual
                new Trigger(() -> Math.abs(codriver_controller.getRawAxis(5)) > 0.2).whileTrue(
                    new RunCommand(() -> Elevator.instance.runWrist(-codriver_controller.getRawAxis(5)))
                        .finallyDo(Elevator.instance::stopWrist));
                
                // Coral reef left/right
                new Trigger(() -> codriver_controller.getPOV() == 90).onTrue(
                    new InstantCommand(() -> SwervePIDController.instance.align_right = true));
        
                new Trigger(() -> codriver_controller.getPOV() == 270).onTrue(
                    new InstantCommand(() -> SwervePIDController.instance.align_right = false));

                // Coral intake / shoot
                new Trigger(() -> codriver_controller.getRawAxis(3) > 0.8).whileTrue(Commands.either(
                    RobotUtils.onOffCommand(Elevator.instance::runShooter, 
                        Elevator.instance.m_current_state == TargetState.kL4? 0.3 : 0.4), 
                    Commands.either(
                        RobotUtils.onOffCommand(Elevator.instance::runShooter, 0.2),
                        RobotUtils.decorateCommandFeedback(
                            Elevator.instance.intakeCoralSequence(),
                            this::rumble),
                        Elevator.instance::getCoralStored),
                    () -> codriver_controller.getRawButton(8)));

                // Algae ground intake
                new JoystickButton(codriver_controller, 5).whileTrue(Commands.either(
                    RobotUtils.onOffCommand(Elevator.instance::runIntake, -0.5), 
                    Commands.either(
                        Elevator.instance.intakeAlgaeCmd(),
                        Commands.either(    
                            Elevator.instance.setStateCmd(TargetState.kIntake)
                                .andThen(Elevator.instance.intakeAlgaeCmd())
                                .finallyDo(() -> Elevator.instance.setState(TargetState.kIntakePrepare)),
                            Elevator.instance.setStateCmd(TargetState.kIntakePrepare),
                                () -> Elevator.instance.m_current_state == TargetState.kIntakePrepare),
                        () -> Elevator.instance.m_current_state == TargetState.kL2Algae
                            || Elevator.instance.m_current_state == TargetState.kL3Algae 
                    ),
                    () -> codriver_controller.getRawButton(8)));

                // Algae outtake
                new Trigger(() -> codriver_controller.getRawAxis(2) > 0.8).whileTrue(new StartEndCommand(
                    () -> Elevator.instance.runIntake(0.7), 
                    () -> Elevator.instance.runIntake(0)));

                new Trigger(() -> codriver_controller.getPOV() == 0).whileTrue(new StartEndCommand(
                    () -> Climber.instance.runClimber(6), 
                    () -> Climber.instance.runClimber(0)));

                new Trigger(() -> codriver_controller.getPOV() == 180).whileTrue(new StartEndCommand(
                    () -> Climber.instance.runClimber(-4), 
                    () -> Climber.instance.runClimber(0)));

                new JoystickButton(codriver_controller, 7).whileTrue(Commands.either(
                    new StartEndCommand(
                        () -> Funnel.instance.setState(FunnelState.kOn), 
                        () -> Funnel.instance.setState(FunnelState.kOff)),
                    new StartEndCommand(
                        () -> Funnel.instance.setState(FunnelState.kReverse), 
                        () -> Funnel.instance.setState(FunnelState.kOff)),
                    () -> codriver_controller.getRawButton(8)));

                break;
        
            case kSimple:
                new JoystickButton(simp_stick, 4).whileTrue(new StartEndCommand(
                    () -> Climber.instance.runClimber(6), 
                    () -> Climber.instance.runClimber(0)));
                new JoystickButton(simp_stick, 1).whileTrue(new StartEndCommand(
                    () -> Climber.instance.runClimber(-3), 
                    () -> Climber.instance.runClimber(0)));
                break;
        }

    }

    /**
     * Decorates a PathPlanner sequence with setter functions to integrate into our modified framework.
     * @return The modified pathplanner sequence command selected on the dashboard.
     */
    public Command getAutonomousCommand() {
        return SubsystemManager.instance.commands.decoratePathplannerCmd(auto_chooser.getSelected());
    }
}
  