package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;

public abstract class Controller {
    public abstract void init();
    public abstract void exec();
    public abstract boolean is_fin();
    public abstract void exit(boolean interrupted);

    public Command getCommand() {
        return new FunctionalCommand(this::init, null, this::exit, this::is_fin);
    }
}