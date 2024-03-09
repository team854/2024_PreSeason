package frc.robot.commands.arm;

import frc.robot.Constants.ArmConstants;
import frc.robot.operator.OperatorInput;
import frc.robot.subsystems.ArmSubsystem;

public class DefaultArmCommand extends BaseArmCommand {


    private final OperatorInput operatorInput;



    /**
     * Creates a new ExampleCommand.
     *
     * @param driveSubsystem The subsystem used by this command.
     */
    public DefaultArmCommand(OperatorInput operatorInput, ArmSubsystem armSubsystem) {

        super(armSubsystem);

        this.operatorInput = operatorInput;

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        logCommandStart();
        super.initialize();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        if (operatorInput.isPivotUp()) {
            armSubsystem.pivotRotSetSpeed(.1);
        }
        else if (operatorInput.isPivotDown()) {
            armSubsystem.pivotRotSetSpeed(-.1);
        }
        else {
            moveToTargetAngle(ArmConstants.EQUILIBRIUM_ARM_ANGLE);
        }


    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // The default drive command never ends, but can be interrupted by other commands.
        return false;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        logCommandEnd(interrupted);
    }

}