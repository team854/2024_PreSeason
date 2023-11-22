package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.lightConstants;
import frc.robot.operator.OperatorInput;
import frc.robot.operator.OperatorInput.Axis;
import frc.robot.operator.OperatorInput.Stick;


public class LightsSubsystem extends SubsystemBase {

    // define lights
    private final AddressableLED       firstLEDStrip       = new AddressableLED(lightConstants.LED_STRIP_ONE_PORT);
    private final AddressableLEDBuffer firstLEDStripBuffer = new AddressableLEDBuffer(lightConstants.LED_STRIP_ONE_LENGTH);



    public LightsSubsystem() {

    }

    private void redLightIndividual(int index, AddressableLED LEDStrip, AddressableLEDBuffer LEDStripBuffer) {
        LEDStripBuffer.setRGB(index, 255, 0, 0);
        LEDStrip.setData(LEDStripBuffer);
    }

    private void greenLightIndividual(int index, AddressableLED LEDStrip, AddressableLEDBuffer LEDStripBuffer) {
        LEDStripBuffer.setRGB(index, 0, 255, 0);
        LEDStrip.setData(LEDStripBuffer);
    }

    private void blueLightIndividual(int index, AddressableLED LEDStrip, AddressableLEDBuffer LEDStripBuffer) {
        LEDStripBuffer.setRGB(index, 0, 0, 255);
        LEDStrip.setData(LEDStripBuffer);
    }

    public void testThrottle(OperatorInput OperatorInput, AddressableLED LEDStrip, AddressableLEDBuffer LEDStripBuffer) {

        double throttle = OperatorInput.getDriverControllerAxis(Stick.LEFT, Axis.Y);
        int    centre   = (int) Math.round(lightConstants.LED_STRIP_ONE_LENGTH / 2 * (throttle + 1));
        centre = Math.max(1, centre);
        centre = Math.min(lightConstants.LED_STRIP_ONE_LENGTH - 2, centre);

        if (throttle == 0) {
            LEDStripBuffer.setRGB(centre - 1, 255, 255, 255);
            LEDStripBuffer.setRGB(centre, 255, 255, 255);
            LEDStripBuffer.setRGB(centre + 1, 255, 255, 255);
        }
        if (throttle > 0) {
            LEDStripBuffer.setRGB(centre - 1, 0, 255, 0);
            LEDStripBuffer.setRGB(centre, 0, 255, 0);
            LEDStripBuffer.setRGB(centre + 1, 0, 255, 0);
        }
        if (throttle < 0) {
            LEDStripBuffer.setRGB(centre - 1, 255, 0, 0);
            LEDStripBuffer.setRGB(centre, 255, 0, 0);
            LEDStripBuffer.setRGB(centre + 1, 255, 0, 0);
        }

        LEDStrip.setData(LEDStripBuffer);

    }


}
