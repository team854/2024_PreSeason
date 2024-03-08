package frc.robot.commands.arm;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmConstants.AngleStates;
import frc.robot.Constants.ArmConstants.PivotShootStates;
import frc.robot.commands.LoggingCommand;
import frc.robot.subsystems.ArmSubsystem;

public class PivotShootCommand extends LoggingCommand {

    ArmSubsystem             armSubsystem;

    // PivotPID
    private double           currentError;
    private double           previousError;
    private double           diffError;
    private double           errorSignal;
    private double           pTerm;
    private double           iTerm = 0;
    private double           dTerm;

    // Speeds
    private double           shootSpeed;
    private double           pivotSpeed;

    // Time Measure
    private double           initTime;
    private double           currTime;
    private double           timeoutMS;

    // Logging skibidi
    private double           speed;
    private boolean          brakeAtEnd;
    private double           targetAngle;
    private String           reason;

    // States
    private PivotShootStates state;
    private AngleStates      angleState;



    public PivotShootCommand(double shootSpeed, double pivotSpeed, double targetAngle, double timeoutMS,
        ArmSubsystem armSubsystem) {

        this.shootSpeed   = shootSpeed;
        this.pivotSpeed   = pivotSpeed;
        this.timeoutMS    = timeoutMS;
        this.armSubsystem = armSubsystem;
        this.targetAngle  = targetAngle;

        addRequirements(armSubsystem);

    }

    @Override
    public void initialize() {

        String commandParms = "shoot speed: " + shootSpeed + "pivot speed: " + pivotSpeed + "target angle: " + targetAngle
            + ", timeout time (ms): " + timeoutMS;
        logCommandStart(commandParms);

        state         = PivotShootStates.PIVOTING;

        previousError = armSubsystem.getAngleErrorPivot(targetAngle);
        initTime      = System.currentTimeMillis();

        if (Math.abs(previousError) > 10) {
            angleState = AngleStates.FAR;
        }
        else {
            angleState = AngleStates.CLOSE;
        }

    }

    @Override
    public void execute() {

        switch (state) {

        case PIVOTING:

            currentError = armSubsystem.getAngleErrorPivot(targetAngle);
            diffError = currentError - previousError;
            previousError = currentError;

            double sgnError = Math.abs(currentError) / currentError;

            switch (angleState) {

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

            if (Math.abs(previousError) > ArmConstants.EQUILIBRIUM_ARM_ANGLE_BUFFER) {
                angleState = AngleStates.FAR;
            }
            else {
                angleState = AngleStates.CLOSE;
            }

            break;

        case SHOOTING:

            armSubsystem.intakeSetSpeed(-speed);

            break;


        }

    }

    @Override
    public boolean isFinished() {

        currTime = System.currentTimeMillis();

        if (currTime - initTime >= timeoutMS) {
            reason = "timeout of " + timeoutMS + " ms extends";
            return true;
        }

        if (state == PivotShootStates.PIVOTING) {
            if (Math.abs(currentError) <= ArmConstants.EQUILIBRIUM_ARM_ANGLE_BUFFER) {
                state = PivotShootStates.SHOOTING;
                armSubsystem.pivotRotSetSpeed(0);
            }
        }

        return false;

    }

    @Override
    public void end(boolean interrupted) {

        armSubsystem.intakeSetSpeed(0);
        armSubsystem.pivotRotSetSpeed(0);


        setFinishReason(reason);
        logCommandEnd(interrupted);

    }



}
