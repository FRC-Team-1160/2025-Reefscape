// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Commands.SpeedController;
import frc.robot.Commands.AlgaeAlignmentPID;
import frc.robot.Subsystems.Vision.ObjectDetection;

public class RobotContainer {
  private Joystick main_stick = new Joystick(Constants.IO.MAIN_PORT);
  private Joystick second_stick = new Joystick(Constants.IO.COPILOT_PORT);
  // private Joystick left_board = new Joystick(Constants.IO.LEFT_BOARD_PORT);
  private Joystick right_board = new Joystick(Constants.IO.RIGHT_BOARD_PORT);

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

  public void updateSubsystemManager() {
    if (RobotState.isEnabled()) {
      m_subsystem_manager.periodic();
    } else if (RobotState.isAutonomous()) {
      m_subsystem_manager.periodic();
    }
  }

  private void configureBindings() {
    new JoystickButton(main_stick, 8).onTrue(
      new InstantCommand(m_subsystem_manager.m_drive::resetGyroAngle)
    );

    new JoystickButton(main_stick, 9).onTrue(
      new InstantCommand(m_subsystem_manager.m_drive::resetGyroAngle)
    );


    // new JoystickButton(main_stick, 3)
    //   .whileTrue(new StartEndCommand(
    //     () -> {m_subsystem_manager.robot_state.drive_state = SubsystemManager.RobotState.DriveStates.PID_CONTROL;},
    //     () -> {m_subsystem_manager.robot_state.drive_state = SubsystemManager.RobotState.DriveStates.FULL_CONTROL;}
    //   ));
    // }

    new JoystickButton(main_stick, 3)
      .whileTrue(m_subsystem_manager.algae_alignment_PID);
  }

  public Command getAutonomousCommand() {
    return auto_chooser.getSelected();
  }
}
