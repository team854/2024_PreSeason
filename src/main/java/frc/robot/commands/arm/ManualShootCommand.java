package frc.robot.commands.arm;

import frc.robot.operator.OperatorInput;
import frc.robot.subsystems.ArmSubsystem;

public class ManualShootCommand extends BaseArmCommand {

    // Speeds
    private double        shootSpeed;
    private double        timeoutMS;

    private OperatorInput operatorInput;

    public ManualShootCommand(double shootSpeed, double timeoutMS,
        ArmSubsystem armSubsystem, OperatorInput operatorInput) {

        super(armSubsystem);

        this.shootSpeed    = shootSpeed;
        this.timeoutMS     = timeoutMS;
        this.operatorInput = operatorInput;
    }

    @Override
    public void initialize() {
        super.initialize();
        String commandParms = "shoot speed: " + shootSpeed;
        logCommandStart(commandParms);
    }

    @Override
    public void execute() {
        // boolean atAngle = moveToTargetAngle(targetAngle);
        // armSubsystem.setShooterSpeed(shootSpeed);

        // Perform additional action if at the target angle
        // if (atAngle && isTimeoutExceeded(3.5)) {
        // armSubsystem.setKeeperSpeed(shootSpeed);
        // }


        // This does not work at this very moment
        // Not so skibidi if you ask me
        armSubsystem.setShooterSpeed(shootSpeed); // armSubsystem.setShooterSpeed(shootSpeed);

        if (operatorInput.isPivotUp()) {
            armSubsystem.pivotRotSetSpeed(0.1);
        }
        else if (operatorInput.isPivotDown()) {
            armSubsystem.pivotRotSetSpeed(-0.1);
        }
        else if (operatorInput.isShoot()) {
            if (isTimeoutExceeded(1.0)) {
                armSubsystem.setKeeperSpeed(shootSpeed);
            }
        }
        else {
            double angle = armSubsystem.getAnglePivot();
            moveToTargetAngle(angle);
        }
    }

    @Override
    public boolean isFinished() {
        // Stop after 3.5 seconds.
        return isTimeoutExceeded(timeoutMS / 1000d);
    }

    @Override
    public void end(boolean interrupted) {
        armSubsystem.intakeSetSpeed(0);
        // armSubsystem.pivotRotSetSpeed(0);
        logCommandEnd(interrupted);
    }
}
