package frc.robot.commands.arm;

import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSubsystem;

public class PivotShootCommand extends BaseArmCommand {

    // Speeds
    private double  shootSpeed;
    private double  pivotSpeed;

    // Time Measure
    private double  initTime;
    private double  currTime;
    private double  timeoutMS;

    // Logging skibidi
    private double  speed;
    private double  targetAngle;
    private String  reason;

    // States
    private boolean shooting;



    public PivotShootCommand(double shootSpeed, double pivotSpeed, double targetAngle, double timeoutMS,
        ArmSubsystem armSubsystem) {

        super(armSubsystem);

        this.shootSpeed  = shootSpeed;
        this.pivotSpeed  = pivotSpeed;
        this.targetAngle = targetAngle;
        this.timeoutMS   = timeoutMS;

    }

    @Override
    public void initialize() {

        String commandParms = "shoot speed: " + shootSpeed + "pivot speed: " + pivotSpeed + "target angle: " + targetAngle
            + ", timeout time (ms): " + timeoutMS;
        logCommandStart(commandParms);

        super.initialize();

        initTime = System.currentTimeMillis();

    }

    @Override
    public void execute() {

        moveToTargetAngle(targetAngle);

        if (Math.abs(getAngleError(targetAngle)) <= ArmConstants.EQUILIBRIUM_ARM_ANGLE_TOLERANCE) {
            shooting = true;
        }

        if (shooting) {
            armSubsystem.intakeSetSpeed(-speed);
        }

    }

    @Override
    public boolean isFinished() {

        currTime = System.currentTimeMillis();

        if (currTime - initTime >= timeoutMS) {
            reason = "timeout of " + timeoutMS + " ms extends";
            return true;
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
