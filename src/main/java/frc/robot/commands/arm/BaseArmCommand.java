package frc.robot.commands.arm;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmConstants.AngleStates;
import frc.robot.commands.LoggingCommand;
import frc.robot.subsystems.ArmSubsystem;

public abstract class BaseArmCommand extends LoggingCommand {

    final ArmSubsystem armSubsystem;

    double             previousError;

    double             pTerm;
    double             iTerm = 0;
    double             dTerm;
    double             errorSignal;


    public BaseArmCommand(ArmSubsystem armSubsystem) {
        this.armSubsystem = armSubsystem;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {
        previousError = 0;
        iTerm         = 0;
    }

    public boolean moveToTargetAngle(double targetAngle) {

        // Go to the equilibrium state
        double      error = armSubsystem.getAngleErrorPivot(targetAngle);

        AngleStates angleState;

        if (Math.abs(error) > ArmConstants.EQUILIBRIUM_ARM_ANGLE_CLOSE) {
            angleState = ArmConstants.AngleStates.FAR;
        }
        else {
            angleState = ArmConstants.AngleStates.CLOSE;
            // If we are at the target, then stop

        }



        switch (angleState) {

        case CLOSE:

            // When close, use PID control

            double diffError = error - previousError;
            previousError = error;

            pTerm = ArmConstants.PIVOT_TO_ANGLE_PID_KP * error;
            iTerm += ArmConstants.PIVOT_TO_ANGLE_PID_KI * error;
            dTerm = ArmConstants.PIVOT_TO_ANGLE_PID_KD * diffError;

            errorSignal = pTerm + iTerm + dTerm;

            errorSignal = Math.max(Math.min(errorSignal, 0.1), -0.1);

            armSubsystem.pivotRotSetSpeed(errorSignal);

            break;

        case FAR:

            // PID is off, reset the iTerm;
            iTerm = 0;
            previousError = error;

            armSubsystem.pivotRotSetSpeed(ArmConstants.PIVOT_DEFAULT_SPEED * Math.signum(error));

            break;

        }

        if (Math.abs(error) <= ArmConstants.EQUILIBRIUM_ARM_ANGLE_TOLERANCE) {
            return true;
        }
        return false;
    }

    public double getAngleError(double targetAngle) {
        return armSubsystem.getAngleErrorPivot(targetAngle);
    }

}
