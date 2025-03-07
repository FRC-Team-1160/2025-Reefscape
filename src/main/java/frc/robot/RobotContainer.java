package frc.robot;

import java.io.IOException;

import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.AutoLogOutputManager;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.IOConstants;
import frc.robot.SubsystemManager.RobotState.ElevatorStates;
import frc.robot.Subsystems.Climber.Climber;
import frc.robot.Subsystems.DriveTrain.DriveTrain;
import frc.robot.Subsystems.Elevator.Elevator;
import frc.robot.Subsystems.Elevator.ElevatorRealIO;
import frc.robot.Subsystems.Elevator.Elevator.TargetState;
import frc.robot.Subsystems.Funnel.Funnel;
import frc.robot.Subsystems.Funnel.Funnel.FunnelState;
import frc.robot.Subsystems.Vision.ObjectDetection;
import frc.robot.Subsystems.Vision.Vision;

public 
class RobotContainer {

    public record JoystickInputs(double drive_x, double drive_y, double drive_a, double elevator) {}

    private Joystick main_stick = new Joystick(IOConstants.MAIN_PORT);
    private Joystick second_stick = new Joystick(IOConstants.COPILOT_PORT);
    private Joystick left_board = new Joystick(IOConstants.LEFT_BOARD_PORT);
    private Joystick right_board = new Joystick(IOConstants.RIGHT_BOARD_PORT);
    private Joystick simp_stick = new Joystick(3);
    private Joystick codriver_controller = new Joystick(5);

    private final SendableChooser<Command> auto_chooser;

    public RobotContainer() {

        // Configure Autobuilder
        RobotConfig config;
        
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
            // Pathplanner commands are redirected to the PathplannerController instance
            (speeds, feedforwards) -> PathplannerController.instance.acceptGeneratedSpeeds(speeds),
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

        AutoLogOutputManager.addObject(SubsystemManager.instance);
        AutoLogOutputManager.addObject(SwervePIDController.instance);
        AutoLogOutputManager.addObject(PathplannerController.instance);
        AutoLogOutputManager.addObject(DriveTrain.instance);
        AutoLogOutputManager.addObject(Elevator.instance);
        AutoLogOutputManager.addObject(Funnel.instance);
        AutoLogOutputManager.addObject(Climber.instance);
        AutoLogOutputManager.addObject(Vision.instance);
        AutoLogOutputManager.addObject(ObjectDetection.instance);
    }

    public void updateSubsystemManager() {
        SubsystemManager.instance.update(new JoystickInputs(
            main_stick.getRawAxis(1), 
            main_stick.getRawAxis(0), 
            second_stick.getRawAxis(0), 
            right_board.getRawAxis(0)));
        // SubsystemManager.instance.update(new JoystickInputs(
        //     simp_stick.getRawAxis(5), 
        //     simp_stick.getRawAxis(4), 
        //     simp_stick.getRawAxis(0), 
        //     right_board.getRawAxis(0)));


    }

    private void configureBindings() {
        new JoystickButton(main_stick, 8).onTrue(
            new InstantCommand(DriveTrain.instance::resetGyroAngle));

        new JoystickButton(right_board, 9).onTrue(
            new StartEndCommand(
                () -> Elevator.instance.runShooter(0.5),
                () -> Elevator.instance.runShooter(0)));

        new JoystickButton(simp_stick, 7).onTrue(
            new InstantCommand(Elevator.instance::zeroWrist));

        new JoystickButton(simp_stick, 8).onTrue(
            new InstantCommand(DriveTrain.instance::resetGyroAngle));

        // new JoystickButton(simp_stick, 4).whileTrue(
        //     new StartEndCommand(
        //         () -> Climber.instance.runClimber(3), 
        //         () -> Climber.instance.runClimber(0)
        //     )
        // );

        new Trigger(() -> simp_stick.getRawAxis(3) > 0.8).whileTrue(
            Elevator.instance.intakeAlgaeCmd()
        );

        new Trigger(() -> simp_stick.getRawAxis(2) > 0.8).whileTrue(
            new StartEndCommand(() -> Elevator.instance.runIntake(0.5),
            () -> Elevator.instance.runIntake(0))
        );

        new Trigger(() -> simp_stick.getPOV() == 0).whileTrue(
            new StartEndCommand(() -> Elevator.instance.runWrist(1),
            () -> ((ElevatorRealIO) Elevator.instance).changeWristSetpoint(0)));

        new Trigger(() -> simp_stick.getPOV() == 180).whileTrue(
            new StartEndCommand(() -> Elevator.instance.runWrist(-1),
            () -> ((ElevatorRealIO) Elevator.instance).changeWristSetpoint(0)));

        new Trigger(() -> simp_stick.getPOV() == 90).whileTrue(
            new StartEndCommand(() -> Elevator.instance.runElevator(2),
            () -> Elevator.instance.runElevator(0.4)));
    
        new Trigger(() -> simp_stick.getPOV() == 270).whileTrue(
            new StartEndCommand(() -> Elevator.instance.runElevator(-1),
            () -> Elevator.instance.runElevator(0.4)));
    
        // new JoystickButton(simp_stick, 1).whileTrue(
        //     new StartEndCommand(
        //         () -> Climber.instance.runClimber(-3), 
        //         () -> Climber.instance.runClimber(0)
        //     )
        // );

        new JoystickButton(simp_stick, 6).onTrue(
            new InstantCommand(() -> Elevator.instance.setState(TargetState.kBarge))
        );

        
        // new JoystickButton(second_stick, 2)
        //     .whileTrue(m_subsystem_manager.commands.alignProcessor());
        if (Robot.isSimulation()) {
            new JoystickButton(second_stick, 3)
                .whileTrue(SubsystemManager.instance.commands.alignReef(true));

            new JoystickButton(second_stick,2)
                .whileTrue(SubsystemManager.instance.commands.alignSource(true));

            new JoystickButton(second_stick, 4)
                .whileTrue(SubsystemManager.instance.commands.alignProcessor(true));

            new JoystickButton(main_stick, 3)
                .whileTrue(SubsystemManager.instance.commands.trackAlgae());
        }
        
        // new JoystickButton(simp_stick, 1)
        //     .onTrue(Elevator.instance.intakeAlgaeCmd());

        // new JoystickButton(simp_stick, 5)
        //     .whileTrue(new StartEndCommand(
        //         () -> Elevator.instance.runIntake(-0.5), 
        //         () -> Elevator.instance.runIntake(0)));

        // new JoystickButton(simp_stick, 6)
        // .whileTrue(new StartEndCommand(
        //     () -> Elevator.instance.runIntake(0.7), 
        //     () -> Elevator.instance.runIntake(0)));
        
        // new JoystickButton(second_stick, 4)
        //     .whileTrue(m_subsystem_manager.commands.alignSource());
        
        // new JoystickButton(main_stick, 6).whileTrue(
        //     new StartEndCommand(
        //         () -> Elevator.instance.runShooter(-0.25), 
        //         () -> Elevator.instance.runShooter(0))
        // );

        // new JoystickButton(main_stick, 7).whileTrue(
        //     new StartEndCommand(
        //         () -> Elevator.instance.runShooter(0.25), 
        //         () -> Elevator.instance.runShooter(0))
        // );

        // new JoystickButton(main_stick, 10).whileTrue(
        //     new StartEndCommand(
        //         () -> Elevator.instance.runIntake(0.5), 
        //         () -> Elevator.instance.runIntake(0))
        // );

        // new JoystickButton(main_stick, 11).whileTrue(
        //     new StartEndCommand(
        //         () -> Elevator.instance.runIntake(-0.5), 
        //         () -> Elevator.instance.runIntake(0))
        // );

        // new JoystickButton(second_stick, 6).whileTrue(
        //     new StartEndCommand(
        //         () -> Elevator.instance.runElevator(2), 
        //         () -> Elevator.instance.runElevator(0))
        // );

        // new JoystickButton(second_stick, 7).whileTrue(
        //     new StartEndCommand(
        //         () -> Elevator.instance.runElevator(-1), 
        //         () -> Elevator.instance.runElevator(0))
        // );

        // new JoystickButton(second_stick, 11).whileTrue(
        //     new StartEndCommand(
        //         () -> Elevator.instance.runWrist(1.5), 
        //         () -> Elevator.instance.runWrist(0))
        // );

        // new JoystickButton(second_stick, 10).whileTrue(
        //     new StartEndCommand(
        //         () -> Elevator.instance.runWrist(-1), 
        //         () -> Elevator.instance.runWrist(0))
        // );

        new JoystickButton(main_stick, 1)
            .whileTrue(SubsystemManager.instance.commands.alignReef(false));


        // new JoystickButton(second_stick, 1)
        //     .whileTrue(SubsystemManager.instance.commands.trackAlgae());


        new JoystickButton(simp_stick, 3).whileTrue(new StartEndCommand(
            () -> Funnel.instance.setState(FunnelState.kDown),
            () -> Funnel.instance.setState(FunnelState.kUp))
        );

        // new JoystickButton(main_stick, 9).onTrue(
        //     new InstantCommand(() -> Elevator.instance.zeroWrist()));

        // new JoystickButton(codriver_controller, 5).onTrue(
        //     new InstantCommand(() -> Elevator.instance.setState(TargetState.kIntakePrepare)));

        new JoystickButton(codriver_controller, 3).onTrue(
            new InstantCommand(() -> Elevator.instance.setState(TargetState.kL1)));
        new JoystickButton(codriver_controller, 1).onTrue(
            new InstantCommand(() -> Elevator.instance.setState(TargetState.kL2)));
        new JoystickButton(codriver_controller, 2).onTrue(
            new InstantCommand(() -> Elevator.instance.setState(TargetState.kL3)));
        new JoystickButton(codriver_controller, 4).onTrue(
            new InstantCommand(() -> Elevator.instance.setState(TargetState.kL4)));

        // new JoystickButton(codriver_controller, 8).whileTrue(
        //     new StartEndCommand(() -> Elevator.instance.runShooter(0.4),
        //     () -> Elevator.instance.runShooter(0)));

        // new JoystickButton(codriver_controller, 7).whileTrue(
        //     new StartEndCommand(
        //         () -> Climber.instance.runClimber(3),
        //         () -> Climber.instance.runClimber(0)
        //     )
        // );

        // new JoystickButton(codriver_controller, 8).whileTrue(
        //     new StartEndCommand(
        //         () -> Climber.instance.runClimber(-3),
        //         () -> Climber.instance.runClimber(0)
        //     )
        // );

        // new JoystickButton(codriver_controller, 4).whileTrue(new StartEndCommand(
        //     () -> Funnel.instance.setState(FunnelState.kDown),
        //     () -> Funnel.instance.setState(FunnelState.kUp))
        // );

        // new JoystickButton(codriver_controller, 1).onTrue(
        //     new InstantCommand(DriveTrain.instance::resetGyroAngle));

        new JoystickButton(codriver_controller, 8).onTrue(
            new InstantCommand(() -> Elevator.instance.zeroWrist()));
        
        new Trigger(() -> codriver_controller.getRawAxis(3) > 0.8).whileTrue(Commands.either(
            new StartEndCommand(
                () -> Elevator.instance.runShooter(0.3),
                () -> Elevator.instance.runShooter(0)
            ),
            new InstantCommand(() -> Elevator.instance.runElevator(-0.1))
                .until(Elevator.instance::getElevatorZeroed)
                .andThen(new InstantCommand(() -> Elevator.instance.setState(TargetState.kSource)))
                .andThen(Elevator.instance.intakeCoralCmd()),
            Elevator.instance::getCoralStored));
        
        new Trigger(() -> codriver_controller.getPOV() == 90).onTrue(
            new InstantCommand(() -> SwervePIDController.instance.align_right = true));

        new Trigger(() -> codriver_controller.getPOV() == 270).onTrue(
            new InstantCommand(() -> SwervePIDController.instance.align_right = false));

        new Trigger(() -> Math.abs(codriver_controller.getRawAxis(1)) > 0.25).whileTrue(
            new RunCommand(() -> Elevator.instance.runElevator(0.35 - 1.5 * codriver_controller.getRawAxis(1)))
                .finallyDo(() -> Elevator.instance.runElevator(0.35)));
        
        new Trigger(() -> Math.abs(codriver_controller.getRawAxis(5)) > 0.2).whileTrue(
            new InstantCommand(() -> Elevator.instance.runWrist(-codriver_controller.getRawAxis(5)))
        ).onFalse(
            new InstantCommand(() -> Elevator.instance.runWrist(0))
        );
    }


    public Command getAutonomousCommand() {
        return SubsystemManager.instance.commands.pathCmdWrapper(auto_chooser.getSelected());
    }
}
  