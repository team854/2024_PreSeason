package frc.robot.commands.arm;

import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;

public class PivotToAngleCommand extends BaseArmCommand {

    // Time measure
    private double  initTime;
    private double  passedTime;
    private double  timeoutTimeMS;

    // Logging skibidi
    private double  speed;
    private boolean brakeAtEnd;
    private String  reason;
    private double  targetAngle;



    public PivotToAngleCommand(double speed, double targetAngle, double timeoutTimeMS,
        ArmSubsystem armSubsystem) {

        super(armSubsystem);

        this.speed         = speed;
        this.targetAngle   = targetAngle;
        this.timeoutTimeMS = timeoutTimeMS;
    }

    @Override
    public void initialize() {

        String commandParms = "speed: " + speed + ", brake: " + brakeAtEnd + ", target heading: " + targetAngle
            + ", timeout time (ms): " + timeoutTimeMS;
        logCommandStart(commandParms);

        super.initialize();

        initTime = System.currentTimeMillis();


    }

    @Override
    public void execute() {

        moveToTargetAngle(targetAngle);

    }

    @Override
    public boolean isFinished() {

        if (Math.abs(getAngleError(targetAngle)) <= ArmConstants.EQUILIBRIUM_ARM_ANGLE_TOLERANCE) {
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
        setFinishReason(reason);
        logCommandEnd(interrupted);
    }

}
