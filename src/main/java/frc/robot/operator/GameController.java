package frc.robot.operator;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.XboxController;

/**
 * The Game Controller extends {@link XboxController}
 * <p>
 * This class adds deadbanding to the axes values (X,Y) of the
 * left and right joysticks on the XBox controller, as well as the Triggers.
 */
public class GameController extends XboxController {

    public static final double DEFAULT_AXIS_DEADBAND = .2;
    private double             axisDeadband          = DEFAULT_AXIS_DEADBAND;

    /**
     * Construct a GameController on the specified port
     * <p>
     * Uses the {{@link #DEFAULT_AXIS_DEADBAND} as the joystick deadband
     *
     * @param port on the driver station which the joystick is plugged into
     */
    public GameController(int port) {
        this(port, DEFAULT_AXIS_DEADBAND);
    }

    /**
     * Construct a GameController on the specified port with the specified deadband
     *
     * @param port on the driver station which the joystick is plugged into
     * @param axisDeadband (0 - 0.4) to use for all axis values on this controller.
     */
    public GameController(int port, final double axisDeadband) {
        super(port);
        if (axisDeadband < 0 || axisDeadband > 0.4) {
            System.out.println("Invalid axis deadband(" + axisDeadband + ") must be between 0 - 0.4. Overriding value to "
                + DEFAULT_AXIS_DEADBAND);
            setAxisDeadband(DEFAULT_AXIS_DEADBAND);
        }
        else {
            setAxisDeadband(axisDeadband);
        }
    }

    @Override
    public double getRawAxis(int axis) {
        double axisValue = super.getRawAxis(axis);
        if (Math.abs(axisValue) < axisDeadband) {
            axisValue = 0;
        }
        else {
            double value = Math.abs(axisValue) - axisDeadband;
            value      = value / (1.0 - axisDeadband);
            value     *= Math.signum(axisValue);
            axisValue  = value;
        }
        if (axis == XboxController.Axis.kLeftY.value || axis == XboxController.Axis.kRightY.value) {
            axisValue *= -1.0;
        }
        return Math.round(axisValue * 100) / 100.0d;
    }

    /**
     * Set the axis deadband on the stick and trigger axes of this gameController
     *
     * @param axisDeadband (0 - 0.4) to use for all axis values on this controller.
     */
    public void setAxisDeadband(double axisDeadband) {
        if (axisDeadband < 0 || axisDeadband > 0.4) {
            System.out.println("Invalid axis deadband(" + axisDeadband
                + ") must be between 0 - 0.4. Axis deadband value not changed. Currently " + this.axisDeadband);
            return;
        }
        this.axisDeadband = axisDeadband;
    }

    /**
     * Pulse the rumble on the controller without blocking the robot's main thread.
     *
     * @param intensity the strength of the rumble (0.0 - 1.0)
     * @param duration the duration of each rumble in seconds
     */
    public void pulseRumble(double intensity, double duration) {
        // Start the rumble
        setRumble(RumbleType.kLeftRumble, intensity);
        setRumble(RumbleType.kRightRumble, intensity);

        // First rumble pulse and stop using Notifier
        try (Notifier stopRumble = new Notifier(() -> {
            setRumble(RumbleType.kLeftRumble, 0);
            setRumble(RumbleType.kRightRumble, 0);
        })) {
            // Schedule stopping the rumble after the duration
            stopRumble.startSingle(duration);
        }

        // Second rumble pulse and stop using Notifier
        try (Notifier secondPulse = new Notifier(() -> {
            setRumble(RumbleType.kLeftRumble, intensity);
            setRumble(RumbleType.kRightRumble, intensity);

            // Stop the second rumble after duration
            try (Notifier stopSecondPulse = new Notifier(() -> {
                setRumble(RumbleType.kLeftRumble, 0);
                setRumble(RumbleType.kRightRumble, 0);
            })) {
                stopSecondPulse.startSingle(duration);
            }
        })) {
            // Schedule the second pulse after the pause (duration + 0.1 seconds)
            secondPulse.startSingle(duration + 0.1);
        }
    }

    @Override
    public String toString() {
        StringBuilder sb = new StringBuilder();
        sb.append('(').append(getLeftX()).append(',').append(getLeftY()).append(") ")
            .append('(').append(getRightX()).append(',').append(getRightY()).append(") ")
            .append("T(").append(getRightTriggerAxis()).append(',').append(getLeftTriggerAxis()).append(")");
        sb.append(getAButton() ? " A" : "");
        sb.append(getBButton() ? " B" : "");
        sb.append(getXButton() ? " X" : "");
        sb.append(getYButton() ? " Y" : "");
        sb.append(getLeftBumper() ? " Lb" : "");
        sb.append(getRightBumper() ? " Rb" : "");
        sb.append(getStartButton() ? " Start" : "");
        sb.append(getBackButton() ? " Back" : "");
        if (getPOV() >= 0) {
            sb.append(" POV(").append(getPOV()).append(')');
        }
        return sb.toString();
    }
}
