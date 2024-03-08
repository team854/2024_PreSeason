package frc.robot.commands.arm;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmConstants.AngleStates;
import frc.robot.Constants.ArmConstants.IntakeStates;
import frc.robot.commands.LoggingCommand;
import frc.robot.operator.OperatorInput;
import frc.robot.subsystems.ArmSubsystem;

public class IntakeCommand extends LoggingCommand {

    ArmSubsystem         armSubsystem;
    OperatorInput        operatorInput;

    // PivotPID
    private double       currentError;
    private double       sgnError;
    private double       previousError;
    private double       diffError;
    private double       errorSignal;
    private double       sgnErrorSignal;
    private double       pTerm;
    private double       iTerm       = 0;
    private double       dTerm;

    // Speeds
    private double       intakeSpeed;
    private double       pivotSpeed;

    // Time Measure
    private double       initTime;
    private double       currTime;
    private double       timeoutMS;

    // Logging skibidi
    private double       speed;
    private double       targetAngle = 0;
    private String       reason;

    // States
    private IntakeStates state;
    private AngleStates  angleState;



    public IntakeCommand(double intakeSpeed, double pivotSpeed, double timeoutMS, ArmSubsystem armSubsystem) {

        this.intakeSpeed  = intakeSpeed;
        this.pivotSpeed   = pivotSpeed;
        this.timeoutMS    = timeoutMS;
        this.armSubsystem = armSubsystem;

        addRequirements(armSubsystem);

    }

    @Override
    public void initialize() {

        String commandParms = "target angle: " + targetAngle + "intake speed: " + intakeSpeed + "pivot speed: " + pivotSpeed
            + "pivot speed: " + pivotSpeed
            + ", timeout time (ms): " + timeoutMS;
        logCommandStart(commandParms);

        state         = IntakeStates.PIVOTING;

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
            sgnError = currentError / Math.abs(currentError);
            diffError = currentError - previousError;

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

            if (Math.abs(previousError) > ArmConstants.EQUILIBRIUM_ARM_ANGLE_BUFFER) {
                angleState = AngleStates.FAR;
            }
            else {
                angleState = AngleStates.CLOSE;
            }

            break;

        case INTAKING:

            armSubsystem.intakeSetSpeed(speed);

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

        if (state == IntakeStates.PIVOTING) {
            if (Math.abs(currentError) <= ArmConstants.EQUILIBRIUM_ARM_ANGLE_BUFFER) {
                state = IntakeStates.INTAKING;
                armSubsystem.pivotRotSetSpeed(0);
            }
        }
        /*
         * if (!operatorInput.isIntake()) {
         * reason = "let go of intake button";
         * return true;
         * }
         */

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
