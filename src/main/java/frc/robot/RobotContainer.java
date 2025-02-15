// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.Joystick;
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
      new InstantCommand(m_subsystem_manager.m_drive::resetGyroAngle)
    );

    new JoystickButton(right_board, 9).onTrue(
      new StartEndCommand(
        () -> m_subsystem_manager.m_elevator.runShooter(0.5),
        () -> m_subsystem_manager.m_elevator.runShooter(0)
      )
    );


    new JoystickButton(main_stick, 3)
      .whileTrue(m_subsystem_manager.commands.trackAlgae());

    new JoystickButton(second_stick, 3)
      .whileTrue(m_subsystem_manager.commands.pathCmdWrapper(
        m_subsystem_manager.m_pathplanner_controller::getNearestReefCmd
      ));

    new JoystickButton(main_stick, 7)
      .toggleOnFalse(m_subsystem_manager.commands.playMusic("chords"));

    new JoystickButton(main_stick, 6)
      .toggleOnFalse(m_subsystem_manager.commands.playMusic("megalo"));

  }


  public Command getAutonomousCommand() {
    return m_subsystem_manager.commands.pathCmdWrapper(auto_chooser.getSelected());
  }
}
