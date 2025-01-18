// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class RobotContainer {

  public final SubsystemManager m_subsystem_manager = new SubsystemManager();

  private Joystick m_mainStick = new Joystick(Constants.IO.MAIN_PORT);
  private Joystick m_codriverStick = new Joystick(Constants.IO.COPILOT_PORT);
  private Joystick m_leftBoard = new Joystick(Constants.IO.LEFT_BOARD_PORT);
  private Joystick m_rightBoard = new Joystick(Constants.IO.RIGHT_BOARD_PORT);

  public RobotContainer() {
    configureBindings();
  }

  public void updateSubsystemManager(){
    m_subsystem_manager.periodic(
      m_mainStick.getRawAxis(1),
      m_mainStick.getRawAxis(0),
      m_codriverStick.getRawAxis(0));
  }

  private void configureBindings() {}

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
