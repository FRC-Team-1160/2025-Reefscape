package frc.robot;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.SubsystemManager.RobotState.DriveStates;
import frc.robot.Subsystems.Vision.Vision;
import frc.robot.Subsystems.Vision.Vision.CameraMode;

/** 
 * The main Robot class. 
 * Our class is slightly modified to implement AdvantageKit and add the subsystem manager to the command loop.
*/
public class Robot extends LoggedRobot {
    private Command autonomous_command;

    private final RobotContainer m_robot_container;

    public Robot() {
        Logger.recordMetadata("Titanium1160", "S[Ti]ngray"); // Set a metadata value

        if (isReal()) {
            Logger.addDataReceiver(new WPILOGWriter("/home/lvuser/logs")); // Log to a USB stick ("/U/logs")
            Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
            // new PowerDistribution(1, ModuleType.kRev); // Enables power distribution logging
        } else {
            Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
            // setUseTiming(false); // Run as fast as possible
            // String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
            // Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
            // Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
        }

        Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may be added.

        m_robot_container = new RobotContainer();

        SmartDashboard.putData(CommandScheduler.getInstance());
    }

    @Override
    public void robotPeriodic() {
        m_robot_container.updateSubsystemManager();
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
        Vision.instance.setCameraPipelines(CameraMode.kStereoAprilTag);
        autonomous_command = m_robot_container.getAutonomousCommand();
        if (autonomous_command != null) {
            autonomous_command.schedule();
        }
    }

    @Override
    public void autonomousPeriodic() {

    }

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        Vision.instance.setCameraPipelines(CameraMode.kStereoAprilTag);
        SubsystemManager.instance.m_robot_state.drive_state = DriveStates.DRIVER_CONTROL;
        if (autonomous_command != null) {
            autonomous_command.cancel();
        }
    }

    @Override
    public void teleopPeriodic() {

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
