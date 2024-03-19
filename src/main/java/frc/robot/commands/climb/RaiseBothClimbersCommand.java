package frc.robot.commands.climb;

import frc.robot.Constants.ClimbConstants;
import frc.robot.operator.OperatorInput;
import frc.robot.subsystems.ClimbSubsystem;

public class RaiseBothClimbersCommand extends BaseClimberCommand {

    OperatorInput operatorInput;

    public RaiseBothClimbersCommand(ClimbSubsystem climbSubsystem, OperatorInput operatorInput) {
        super(climbSubsystem);
        this.operatorInput = operatorInput;
    }

    @Override
    public void initialize() {
        logCommandStart();
        super.initialize();
    }

    // Check here
    @Override
    public void execute() {

        if (climbSubsystem.isLeftAtLimit()) {
            climbSubsystem.setLeftSpeed(0);
        }
        else {
            climbSubsystem.setLeftSpeed(-ClimbConstants.CLIMBER_LOWER_SPEED);
        }

        if (climbSubsystem.isRightAtLimit()) {
            climbSubsystem.setRightSpeed(0);
        }
        else {
            climbSubsystem.setRightSpeed(-ClimbConstants.CLIMBER_LOWER_SPEED);
        }

    }

    @Override
    public boolean isFinished() {

        if (!operatorInput.isRaiseClimbers()) {
            setFinishReason("let go of intake button");
            return true;
        }

        return false;
    }

    @Override
    public void end(boolean interrupted) {
        climbSubsystem.setLeftSpeed(0);
        climbSubsystem.setRightSpeed(0);
        logCommandEnd(interrupted);
    }

}
