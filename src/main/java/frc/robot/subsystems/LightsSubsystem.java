package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LightConstants;


public class LightsSubsystem extends SubsystemBase {

    // define lights
    private final AddressableLED              ledStrip      = new AddressableLED(LightConstants.LED_STRIP_PWM_PORT);
    private final AddressableLEDBuffer        ledBuffer     = new AddressableLEDBuffer(LightConstants.LED_STRIP_LENGTH);

    private static final AddressableLEDBuffer RSL_OFF       = new AddressableLEDBuffer(LightConstants.LED_STRIP_LENGTH);
    private static final AddressableLEDBuffer RSL_ON        = new AddressableLEDBuffer(LightConstants.LED_STRIP_LENGTH);

    private static final Color                RSL_COLOR     = Color.kOrange;

    private int                               rslFlashCount = -1;
    private boolean                           prevRSLOn     = false;

    static {
        // Initialize the RSL buffers
        for (int i = 0; i < LightConstants.LED_STRIP_LENGTH; i++) {
            RSL_ON.setLED(i, RSL_COLOR);
            RSL_OFF.setLED(i, Color.kBlack);
        }
    }

    public LightsSubsystem() {

        // Start with all LEDs at a medium grey (not too blinding)
        setAllLEDs(new Color(60, 60, 60));
        ledStrip.start();
    }

    public void setEnabled() {

        // When the robot is first enabled flash the LEDs with the robot RSL for 5 flashes
        rslFlashCount = 5;
    }

    @Override
    public void periodic() {

        // Flash all lights in time with the RLS when the robot is first enabled
        if (rslFlashCount >= 0) {
            flashRSL();
        }
        else {
            ledStrip.setData(ledBuffer);
        }
    }

    private void clear() {
        for (int i = 0; i < LightConstants.LED_STRIP_LENGTH; i++) {
            ledBuffer.setLED(i, RSL_COLOR);
        }
    }

    private void setAllLEDs(Color color) {

        for (int i = 0; i < LightConstants.LED_STRIP_LENGTH; i++) {
            ledBuffer.setLED(i, color);
        }
    }

    private void setLED(int index, Color color) {
        ledBuffer.setLED(index, color);
    }

    private void flashRSL() {

        // Flash in time with the RSL light
        boolean rslOn = RobotController.getRSLState();

        // when the RSL changes from on to off, then
        // update the RSL count.
        if (prevRSLOn && !rslOn) {
            rslFlashCount--;
        }
        prevRSLOn = rslOn;

        // Set the LEDs based on the RSL state
        if (rslOn) {
            ledStrip.setData(RSL_ON);
        }
        else {
            ledStrip.setData(RSL_OFF);
        }
    }


}
