package frc.robot.commands.arm;

import frc.robot.Constants.ArmConstants;
import frc.robot.operator.OperatorInput;
import frc.robot.subsystems.ArmSubsystem;

public class IntakeCommand extends BaseArmCommand {

    final OperatorInput operatorInput;

    // Speeds

    public IntakeCommand(OperatorInput operatorInput, ArmSubsystem armSubsystem) {

        super(armSubsystem);

        this.operatorInput = operatorInput;

    }

    @Override
    public void initialize() {
        logCommandStart();

        super.initialize();

    }

    @Override
    public void execute() {

        boolean atTarget = moveToTargetAngle(ArmConstants.INTAKE_ANGLE);

        if (atTarget) {
            armSubsystem.intakeSetSpeed(ArmConstants.INTAKE_SPEED);
        }

    }

    @Override
    public boolean isFinished() {

        if (!operatorInput.isIntake() && isTimeoutExceeded(0.25)) {
            setFinishReason("let go of intake button");
            return true;
        }

        return false;



    }

    @Override
    public void end(boolean interrupted) {

        armSubsystem.intakeSetSpeed(0);


        logCommandEnd(interrupted);

    }



}
