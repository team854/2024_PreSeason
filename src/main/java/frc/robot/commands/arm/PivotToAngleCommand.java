package frc.robot.commands.arm;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmConstants.HeadingStates;
import frc.robot.commands.LoggingCommand;
import frc.robot.subsystems.ArmSubsystem;

public class PivotToAngleCommand extends LoggingCommand {

    private ArmSubsystem armSubsystem;



    // PID
    private double        currentError;
    private double        previousError;
    private double        diffError;
    private double        errorSignal;
    private double        pTerm;
    private double        iTerm = 0;
    private double        dTerm;

    // Time measure
    private double        initTime;
    private double        passedTime;
    private double        timeoutTimeMS;

    private HeadingStates headingState;

    // Logging skibidi
    private double        speed;
    private boolean       brakeAtEnd;
    private String        reason;
    private double        targetAngle;



    public PivotToAngleCommand(double speed, double targetAngle, boolean brakeAtEnd, double timeoutTimeMS,
        ArmSubsystem armSubsystem) {
        this.speed         = speed;
        this.brakeAtEnd    = brakeAtEnd;
        this.targetAngle   = targetAngle;
        this.timeoutTimeMS = timeoutTimeMS;
        this.armSubsystem  = armSubsystem;

        addRequirements(armSubsystem);
    }

    @Override
    public void initialize() {

        String commandParms = "speed: " + speed + ", brake: " + brakeAtEnd + ", target heading: " + targetAngle
            + ", timeout time (ms): " + timeoutTimeMS;
        logCommandStart(commandParms);

        previousError = armSubsystem.getAngleErrorPivot(targetAngle);


        initTime      = System.currentTimeMillis();

        if (Math.abs(previousError) > 10) {
            headingState = HeadingStates.FAR;
        }
        else {
            headingState = HeadingStates.CLOSE;
        }


    }

    @Override
    public void execute() {

        // executes every 20ms

        currentError  = armSubsystem.getAngleErrorPivot(targetAngle);
        diffError     = currentError - previousError;
        previousError = currentError;

        double sgnError = Math.abs(currentError) / currentError;

        switch (headingState) {

        case FAR:
        default:

            armSubsystem.pivotRotSetSpeed(speed * sgnError);
            break;

        case CLOSE:

            pTerm = ArmConstants.PIVOT_TO_ANGLE_PID_KP * currentError;
            iTerm += ArmConstants.PIVOT_TO_ANGLE_PID_KI * currentError;
            dTerm += ArmConstants.PIVOT_TO_ANGLE_PID_KD * diffError;

            errorSignal = pTerm + iTerm + dTerm;

            Math.max(Math.min(errorSignal + Math.abs(errorSignal) / errorSignal * 0.2, 1), -1);

            armSubsystem.pivotRotSetSpeed(speed * sgnError);

            break;
        }

        if (Math.abs(previousError) > ArmConstants.PIVOT_FAR_TO_CLOSE) {
            headingState = HeadingStates.FAR;
        }
        else {
            headingState = HeadingStates.CLOSE;
        }


    }

    @Override
    public boolean isFinished() {

        // executes every 20ms

        currentError = armSubsystem.getAngleErrorPivot(targetAngle);


        if (Math.abs(currentError) <= ArmConstants.PIVOT_ROT_BUFFER) {
            reason = "Within buffer accuracy";
            return true;
        }


        passedTime = System.currentTimeMillis();
        if (passedTime - initTime > timeoutTimeMS) {
            reason = "timeout";
            return true;
        }

        return false;

    }

    @Override
    public void end(boolean interrupted) {

        if (brakeAtEnd) {
            armSubsystem.pivotRotSetSpeed(0);
        }


        setFinishReason(reason);
        logCommandEnd(interrupted);
    }

}
