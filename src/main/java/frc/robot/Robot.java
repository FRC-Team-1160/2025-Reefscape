// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
    private Command autonomous_command;

    private final RobotContainer m_robot_container;

    public Robot() {
        m_robot_container = new RobotContainer();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        autonomous_command = m_robot_container.getAutonomousCommand();
        if (autonomous_command != null) {
            autonomous_command.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {
        m_robot_container.updateSubsystemManager();
    }

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        if (autonomous_command != null) {
            autonomous_command.cancel();
        }
    }

    @Override
    public void teleopPeriodic() {
        m_robot_container.updateSubsystemManager();
    }

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}
}
