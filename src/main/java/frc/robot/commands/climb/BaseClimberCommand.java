package frc.robot.commands.climb;

import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.ClimbConstants.ClimbStates;
import frc.robot.commands.LoggingCommand;
import frc.robot.subsystems.ClimbSubsystem;

public abstract class BaseClimberCommand extends LoggingCommand {

    final ClimbSubsystem climbSubsystem;

    double               previousError;

    double               pTerm;
    double               iTerm = 0;
    double               dTerm;
    double               errorSignal;

    public BaseClimberCommand(ClimbSubsystem climbSubsystem) {
        this.climbSubsystem = climbSubsystem;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(climbSubsystem);
    }

    @Override
    public void initialize() {
        previousError = 0;
        iTerm         = 0;
    }

    public boolean keepLeftAtDisp(double targetDisp) {
        // Go to the equilibrium state
        double      error = climbSubsystem.getDispErrorLeft(targetDisp);

        ClimbStates climbState;

        if (Math.abs(error) > ClimbConstants.CLIMB_DISP_CLOSE) {
            climbState = ClimbConstants.ClimbStates.FAR;
        }
        else {
            climbState = ClimbConstants.ClimbStates.CLOSE;
        }

        switch (climbState) {

        case CLOSE:

            // When close, use PID control

            double diffError = error - previousError;
            previousError = error;

            pTerm = ClimbConstants.CLIMB_PID_KP * error;
            iTerm += ClimbConstants.CLIMB_PID_KI * error;
            dTerm = ClimbConstants.CLIMB_PID_KD * diffError;

            errorSignal = pTerm + iTerm + dTerm;

            errorSignal = Math.max(Math.min(errorSignal, 0.1), -0.1);

            climbSubsystem.setLeftSpeed(errorSignal);

            break;

        case FAR:

            // PID is off, reset the iTerm;
            iTerm = 0;
            previousError = error;

            climbSubsystem.setLeftSpeed(ClimbConstants.CLIMBER_DEFAULT_SPEED * Math.signum(error));

            break;

        }

        if (Math.abs(error) <= ClimbConstants.DISP_TOLERANCE) {
            return true;
        }
        return false;
    }

    public boolean keepRightAtDisp(double targetDisp) {
        // Go to the equilibrium state
        double      error = climbSubsystem.getDispErrorRight(targetDisp);

        ClimbStates climbState;

        if (Math.abs(error) > ClimbConstants.CLIMB_DISP_CLOSE) {
            climbState = ClimbConstants.ClimbStates.FAR;
        }
        else {
            climbState = ClimbConstants.ClimbStates.CLOSE;
        }

        switch (climbState) {

        case CLOSE:

            // When close, use PID control

            double diffError = error - previousError;
            previousError = error;

            pTerm = ClimbConstants.CLIMB_PID_KP * error;
            iTerm += ClimbConstants.CLIMB_PID_KI * error;
            dTerm = ClimbConstants.CLIMB_PID_KD * diffError;

            errorSignal = pTerm + iTerm + dTerm;

            errorSignal = Math.max(Math.min(errorSignal, 0.1), -0.1);

            climbSubsystem.setRightSpeed(errorSignal);

            break;

        case FAR:

            // PID is off, reset the iTerm;
            iTerm = 0;
            previousError = error;

            climbSubsystem.setRightSpeed(ClimbConstants.CLIMBER_DEFAULT_SPEED * Math.signum(error));

            break;

        }

        if (Math.abs(error) <= ClimbConstants.DISP_TOLERANCE) {
            return true;
        }
        return false;
    }


}
