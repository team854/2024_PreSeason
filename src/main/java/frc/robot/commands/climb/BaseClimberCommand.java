package frc.robot.commands.climb;

import frc.robot.commands.LoggingCommand;
import frc.robot.subsystems.ClimbSubsystem;

public abstract class BaseClimberCommand extends LoggingCommand {

    final ClimbSubsystem climbSubsystem;

    public BaseClimberCommand(ClimbSubsystem climbSubsystem) {
        this.climbSubsystem = climbSubsystem;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(climbSubsystem);
    }


}
