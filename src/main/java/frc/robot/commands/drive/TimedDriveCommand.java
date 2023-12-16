package frc.robot.commands.drive;

import frc.robot.commands.LoggingCommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class TimedDriveCommand extends LoggingCommandBase {

    private DriveSubsystem driveSubsystem;
    private double         time, leftSpeed, rightSpeed;
    private boolean        brakeAtEnd;

    private long           initialTime, currentTime;
    private long           runTime = 0;


    public TimedDriveCommand(double time, double leftSpeed, double rightSpeed, DriveSubsystem driveSubsystem) {

        /*
         * RM: I like the idea of the brake at end.
         * FIXME breakAtEnd should be a parameter
         * passed into the command on the constructor.
         *
         * If this command returns control to the DefaultDriveCommand (ie. if there is no next
         * command), then the robot will brake at that point unless the driver is manipulating the
         * joysticks.
         *
         */
        this.time           = time;
        this.leftSpeed      = leftSpeed;
        this.rightSpeed     = rightSpeed;
        this.driveSubsystem = driveSubsystem;

        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {

        /*
         * RM: This is not really required if you use the corresponding feature in the
         * super().
         *
         * FIXME Log command start
         * You should always log the command start so that it appears in the logs, ideally with
         * all of the parameters that were passed in.
         *
         * logCommandStart(commandParms) where commandParms is a string that lists the parms
         * passed into this command, maybe something like
         * String commandParms = "x ms, [left, right], brake" <- but with the values filled in
         */
        initialTime = System.currentTimeMillis();

    }

    @Override
    public void execute() {

        /*
         * RM: This is correct
         */
        // executes every 20ms
        driveSubsystem.setMotorSpeeds(leftSpeed, rightSpeed);

    }

    @Override
    public boolean isFinished() {

        /*
         * RM: The concept of the isFinished() routine is to determine whether or not one
         * of the stopping conditions of the command has been met. In this case, there is
         * only one: time
         *
         * The super has a convenience method to help with timing. When you log the command start,
         * the super saves the start time, and you can use the following to determine if the
         * command has expired: isTimeoutExceeded(timeInSeconds)
         *
         * if (isTimeoutExceeded(timeInSeconds)) {
         * ...setFinishReason("timeInSeconds exceeded");
         * ...return true;
         * }
         */

        currentTime = System.currentTimeMillis();
        runTime     = currentTime - initialTime;

        /*
         * RM:
         * FIXME brakeAtEnd should be a command parm
         */
        brakeAtEnd  = false;

        /*
         * RM: This is a bit confusing. It seems like the ideas of brake and isFinished are
         * conflated here.
         *
         * Using the calculation of the runTime (in ms) above (which is correct), the code should
         * return true if the time has been exceeded or false otherwise.
         *
         * FIXME conflated brakeAtEnd and isFinished()
         *
         * if (runTime > time) {
         * ...return true;
         * }
         *
         * return false;
         *
         */
        if (runTime <= time) {
            brakeAtEnd = false;
        }
        else {
            brakeAtEnd = true;
        }

        return brakeAtEnd;

    }

    @Override
    /*
     * RM: This code is incorrect.
     *
     * FIXME Cannot override/change parm name
     * The value passed into the end method is always whether or
     * not the command was interrupted. The value is set by the wpilib framework, and cannot
     * be overridden (to indicated brakeAtEnd). It has nothing to do with the brakeAtEnd, and
     * indicates whether another command took over the subsystem before the isFinished returned
     * true.
     *
     * The method signature should always be:
     * public void end(boolean interrupted)
     */
    public void end(boolean brakeAtEnd) {

        /*
         * RM: Always log the command end
         * NOTE: logging of the command start is required in order for the command end to
         * function correctly
         *
         * FIXME log the command end
         * logCommandEnd(interrupted)
         */
        /*
         * RM: If brakeAtEnd is a separate passed in concept, then this code is correct
         */
        if (brakeAtEnd) {
            driveSubsystem.setMotorSpeeds(0, 0);
        }
    }

}
