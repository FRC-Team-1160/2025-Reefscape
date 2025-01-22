// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Commands.AlgaeAlignment;
import frc.robot.Commands.AlgaeAlignmentPID;
import frc.robot.Subsystems.DriveTrain.DriveTrain;
import frc.robot.Subsystems.Vision.ObjectDetection;

public class RobotContainer {

  public final SubsystemManager m_subsystem_manager = new SubsystemManager();

  private Joystick m_mainStick = new Joystick(Constants.IO.MAIN_PORT);
  private Joystick m_codriverStick = new Joystick(Constants.IO.COPILOT_PORT);
  private Joystick m_leftBoard = new Joystick(Constants.IO.LEFT_BOARD_PORT);
  private Joystick m_rightBoard = new Joystick(Constants.IO.RIGHT_BOARD_PORT);

  ObjectDetection m_ObjectDetection;
  DriveTrain m_DriveTrain;
  
  public RobotContainer() {
    configureBindings();
    m_ObjectDetection = new ObjectDetection();
    m_DriveTrain = m_subsystem_manager.m_drive;
  }

  public void updateSubsystemManager(){
    m_subsystem_manager.periodic(
      m_mainStick.getRawAxis(1),
      m_mainStick.getRawAxis(0),
      m_codriverStick.getRawAxis(0));
  }

  private void configureBindings() {
    new JoystickButton(m_mainStick, 3)
      .whileTrue(new AlgaeAlignmentPID(m_ObjectDetection, m_DriveTrain));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
