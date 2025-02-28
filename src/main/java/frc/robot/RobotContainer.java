package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.IOConstants;

public class RobotContainer {
    private Joystick main_stick = new Joystick(IOConstants.MAIN_PORT);
    private Joystick second_stick = new Joystick(IOConstants.COPILOT_PORT);
    private Joystick left_board = new Joystick(IOConstants.LEFT_BOARD_PORT);
    private Joystick right_board = new Joystick(IOConstants.RIGHT_BOARD_PORT);
    private Joystick simp_stick = new Joystick(4);

    public final SubsystemManager m_subsystem_manager = new SubsystemManager(
        () -> main_stick.getRawAxis(1), // i think these should be swapped
        () -> main_stick.getRawAxis(0),
        () -> second_stick.getRawAxis(0),
        () -> right_board.getRawAxis(1)
    );

    private final SendableChooser<Command> auto_chooser;

    public RobotContainer() {
        auto_chooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", auto_chooser);   
    configureBindings();
  }

    private void configureBindings() {
        new JoystickButton(main_stick, 8).onTrue(
            new InstantCommand(m_subsystem_manager.m_drive::resetGyroAngle));

        new JoystickButton(right_board, 9).onTrue(
            new StartEndCommand(
                () -> m_subsystem_manager.m_elevator.runShooter(0.5),
                () -> m_subsystem_manager.m_elevator.runShooter(0)));

        new JoystickButton(simp_stick, 4)
            .whileTrue(m_subsystem_manager.commands.trackAlgae());
        
        // new JoystickButton(second_stick, 2)
        //     .whileTrue(m_subsystem_manager.commands.alignProcessor());
        if (Robot.isSimulation()) {
            new JoystickButton(second_stick, 3)
                .whileTrue(m_subsystem_manager.commands.alignReef());
        }
        
        new JoystickButton(simp_stick, 1)
            .onTrue(m_subsystem_manager.m_elevator.intakeAlgaeCmd());

        new JoystickButton(simp_stick, 5)
            .whileTrue(new StartEndCommand(
                () -> m_subsystem_manager.m_elevator.runIntake(-0.5), 
                () -> m_subsystem_manager.m_elevator.runIntake(0)));

        new JoystickButton(simp_stick, 6)
        .whileTrue(new StartEndCommand(
            () -> m_subsystem_manager.m_elevator.runIntake(0.7), 
            () -> m_subsystem_manager.m_elevator.runIntake(0)));
        
        // new JoystickButton(second_stick, 4)
        //     .whileTrue(m_subsystem_manager.commands.alignSource());
        
        new JoystickButton(main_stick, 6).whileTrue(
            new StartEndCommand(
                () -> m_subsystem_manager.m_elevator.runShooter(-0.25), 
                () -> m_subsystem_manager.m_elevator.runShooter(0))
        );

        new JoystickButton(main_stick, 7).whileTrue(
            new StartEndCommand(
                () -> m_subsystem_manager.m_elevator.runShooter(0.25), 
                () -> m_subsystem_manager.m_elevator.runShooter(0))
        );

        new JoystickButton(main_stick, 10).whileTrue(
            new StartEndCommand(
                () -> m_subsystem_manager.m_elevator.runIntake(0.5), 
                () -> m_subsystem_manager.m_elevator.runIntake(0))
        );

        new JoystickButton(main_stick, 11).whileTrue(
            new StartEndCommand(
                () -> m_subsystem_manager.m_elevator.runIntake(-0.5), 
                () -> m_subsystem_manager.m_elevator.runIntake(0))
        );

        new JoystickButton(second_stick, 6).whileTrue(
            new StartEndCommand(
                () -> m_subsystem_manager.m_elevator.runElevator(6), 
                () -> m_subsystem_manager.m_elevator.runElevator(0))
        );

        new JoystickButton(second_stick, 7).whileTrue(
            new StartEndCommand(
                () -> m_subsystem_manager.m_elevator.runElevator(-6), 
                () -> m_subsystem_manager.m_elevator.runElevator(0))
        );

        new JoystickButton(second_stick, 11).whileTrue(
            new StartEndCommand(
                () -> m_subsystem_manager.m_elevator.runWrist(1.5), 
                () -> m_subsystem_manager.m_elevator.runWrist(0))
        );

        new JoystickButton(second_stick, 10).whileTrue(
            new StartEndCommand(
                () -> m_subsystem_manager.m_elevator.runWrist(-1.5), 
                () -> m_subsystem_manager.m_elevator.runWrist(0))
        );
    }


    public Command getAutonomousCommand() {
        return m_subsystem_manager.commands.pathCmdWrapper(auto_chooser.getSelected());
    }
}
  