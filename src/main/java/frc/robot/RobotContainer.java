// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Subsystems.DriveTrain.DriveTrain;
import frc.robot.Subsystems.DriveTrain.DriveTrainRealIO;
import frc.robot.Subsystems.DriveTrain.DriveTrainSimIO;

public class RobotContainer {
  private Joystick main_stick = new Joystick(Constants.IO.MAIN_PORT);
  private Joystick second_stick = new Joystick(Constants.IO.COPILOT_PORT);
  // private Joystick left_board = new Joystick(Constants.IO.LEFT_BOARD_PORT);
  // private Joystick right_board = new Joystick(Constants.IO.RIGHT_BOARD_PORT);


  public DriveTrain m_drivetrain;

  public RobotContainer() {
    configureBindings();

    if (RobotBase.isReal()) {
      m_drivetrain = new DriveTrainRealIO();
    } else {
      m_drivetrain = new DriveTrainSimIO();
    }
  }

  private void configureBindings() {

  }

  public void runSwerve() {
    m_drivetrain.setInputs(
      main_stick.getRawAxis(0),
      main_stick.getRawAxis(1),
      second_stick.getRawAxis(0)
    );
  }

  public Command getAutonomousCommand() {
    return new InstantCommand();
  }
}
