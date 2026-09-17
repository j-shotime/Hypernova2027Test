package first.robot.custom;

import org.wpilib.driverstation.GenericHID;
import org.wpilib.driverstation.GenericHID.RumbleType;

public class VexController {
    private final GenericHID hid;
    private static final double AXIS_DEADBAND = 0.05;

    // Constructor
    public VexController(int port) {
        this.hid = new GenericHID(port);
    }

    public VexController(GenericHID hid) {
        this.hid = hid;
    }

    // Getters for axes (left stick, right stick, triggers, etc.)
    public double getLeftX() {
        return driveCurve(hid.getRawAxis(0));
    }

    public double getLeftY() {
        return -driveCurve(hid.getRawAxis(1));
    }

    public double getRightX() {
        return driveCurve(hid.getRawAxis(2));
    }

    public double getRightY() {
        return -driveCurve(hid.getRawAxis(3));
    }

    // Button getters - Face buttons
    public boolean getButtonA() {
        return hid.getRawButton(1);
    }

    public boolean getButtonB() {
        return hid.getRawButton(0);
    }

    public boolean getButtonX() {
        return hid.getRawButton(3);
    }

    public boolean getButtonY() {
        return hid.getRawButton(2);
    }

    // Shoulder buttons
    public boolean getL1() {
        return hid.getRawButton(4);
    }

    public boolean getR1() {
        return hid.getRawButton(5);
    }

    public boolean getL2() {
        return hid.getRawButton(6);
    }

    public boolean getR2() {
        return hid.getRawButton(7);
    }

    // Directional buttons
    public boolean getButtonUp() {
        return hid.getRawButton(12);
    }

    public boolean getButtonDown() {
        return hid.getRawButton(13);
    }

    public boolean getButtonLeft() {
        return hid.getRawButton(14);
    }

    public boolean getButtonRight() {
        return hid.getRawButton(15);
    }

    public boolean getButtonCenter() {
        return hid.getRawButton(13);
    }

    public void testPrintAllValues() {
        System.out.println("=== VEXCONTROLLER PORT 0 DEBUG ===");
        System.out.println("Axes - LeftX: " + getLeftX() + " LeftY: " + getLeftY() + " RightX: " + getRightX() + " RightY: " + getRightY());
        for (int axis = 0; axis < 4; axis++) {
            System.out.println("  Raw Axis " + axis + ": " + hid.getRawAxis(axis));
        }
        System.out.println("Buttons:");
        for (int button = 1; button <= 16; button++) {
            if (hid.getRawButton(button)) {
                System.out.println("  Button " + button + " PRESSED");
            }
        }
    }

    // Utility
    public GenericHID getHID() {
        return hid;
    }

    public boolean isConnected() {
        return hid.isConnected();
    }

    public void setRumble(RumbleType type, double value) {
        hid.setRumble(type, value);
    }

    private static double driveCurve(double value) {
        if (Math.abs(value) < AXIS_DEADBAND) {
            return 0.0;
        }

        double scaledValue = (Math.abs(value) - AXIS_DEADBAND) / (1.0 - AXIS_DEADBAND);
        double curvedValue = Math.sqrt(scaledValue);
        return Math.copySign(curvedValue, value);
    }
}
