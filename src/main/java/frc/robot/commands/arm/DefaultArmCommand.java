package frc.robot.commands.arm;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmConstants.AngleStates;
import frc.robot.commands.LoggingCommand;
import frc.robot.operator.OperatorInput;
import frc.robot.subsystems.ArmSubsystem;

public class DefaultArmCommand extends LoggingCommand {

    private final ArmSubsystem  armSubsystem;
    private final OperatorInput operatorInput;

    AngleStates                 angleState;

    double                      previousError;
    double                      currentError;
    double                      diffError;

    double                      sgnError;
    double                      sgnErrorSignal;

    double                      pTerm;
    double                      iTerm = 0;
    double                      dTerm;
    double                      errorSignal;



    /**
     * Creates a new ExampleCommand.
     *
     * @param driveSubsystem The subsystem used by this command.
     */
    public DefaultArmCommand(OperatorInput operatorInput, ArmSubsystem armSubsystem) {

        this.operatorInput = operatorInput;
        this.armSubsystem  = armSubsystem;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(armSubsystem);

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        logCommandStart();

        previousError = armSubsystem.getAngleErrorPivot(ArmConstants.EQUILIBRIUM_ARM_ANGLE);

        angleState    = ArmConstants.AngleStates.FAR;
        /*
         * if (Math.abs(previousError) > ArmConstants.EQUILIBRIUM_ARM_ANGLE_BUFFER) {
         * angleState = ArmConstants.AngleStates.FAR;
         * }
         * else {
         * angleState = ArmConstants.AngleStates.CLOSE;
         * }
         */
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

            currentError = armSubsystem.getAngleErrorPivot(ArmConstants.EQUILIBRIUM_ARM_ANGLE);
            sgnError     = currentError / Math.abs(currentError);
            diffError    = currentError - previousError;

            switch (angleState) {

            case CLOSE:

                pTerm = ArmConstants.PIVOT_TO_ANGLE_PID_KP * currentError;
                iTerm += ArmConstants.PIVOT_TO_ANGLE_PID_KI * currentError;
                dTerm += ArmConstants.PIVOT_TO_ANGLE_PID_KD * diffError;

                errorSignal = pTerm + iTerm + dTerm;
                try {
                    sgnErrorSignal = errorSignal / Math.abs(errorSignal);
                }
                catch (Exception e) {
                    sgnErrorSignal = 0;
                }


                errorSignal = Math.max(Math.min(errorSignal + sgnErrorSignal * ArmConstants.PIVOT_DEFAULT_SPEED, 1.0), -1.0);

                armSubsystem.pivotRotSetSpeed(errorSignal * sgnError);

                break;

            case FAR:

                armSubsystem.pivotRotSetSpeed(ArmConstants.PIVOT_DEFAULT_SPEED * sgnError);

                break;

            }



        }


    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // The default drive command never ends, but can be interrupted by other commands.

        if (Math.abs(previousError) > ArmConstants.EQUILIBRIUM_ARM_ANGLE_BUFFER) {
            angleState = ArmConstants.AngleStates.FAR;
        }
        else {
            angleState = ArmConstants.AngleStates.CLOSE;
        }

        return false;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        logCommandEnd(interrupted);
    }

}