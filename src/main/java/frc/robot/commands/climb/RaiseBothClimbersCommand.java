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
        climbSubsystem.setLeftSpeed(-ClimbConstants.CLIMBER_RAISE_SPEED);
        climbSubsystem.setRightSpeed(-ClimbConstants.CLIMBER_RAISE_SPEED);
    }

    @Override
    public boolean isFinished() {

        if (!operatorInput.isRaiseClimbers() && isTimeoutExceeded(0.25)) {
            setFinishReason("let go of intake button");
            return true;
        }

        return false;
    }

    @Override
    public void end(boolean interrupted) {
        logCommandEnd(interrupted);
    }

}
