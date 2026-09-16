package first.robot.custom;

import org.wpilib.driverstation.GenericHID;
import org.wpilib.driverstation.GenericHID.RumbleType;

public class VexController {
    private final GenericHID hid;
    private double deadband = 0.1;

    // Constructor
    public VexController(int port) {
        this.hid = new GenericHID(port);
    }

    public VexController(GenericHID hid) {
        this.hid = hid;
    }

    // Getters for axes (left stick, right stick, triggers, etc.)
    public double getLeftX() {
        double value = hid.getRawAxis(0);
        return Math.abs(value) < deadband ? 0 : value;
    }

    public double getLeftY() {
        double value = hid.getRawAxis(1);
        return Math.abs(value) < deadband ? 0 : value;
    }

    public double getRightX() {
        double value = hid.getRawAxis(4);
        return Math.abs(value) < deadband ? 0 : value;
    }

    public double getRightY() {
        double value = hid.getRawAxis(5);
        return Math.abs(value) < deadband ? 0 : value;
    }

    // Button getters
    public boolean getButtonA() {
        return hid.getRawButton(1);
    }

    public boolean getButtonB() {
        return hid.getRawButton(2);
    }

    public boolean getButtonX() {
        return hid.getRawButton(3);
    }

    public boolean getButtonY() {
        return hid.getRawButton(4);
    }

    public boolean getL1() {
        return hid.getRawButton(5);
    }

    public boolean getR1() {
        return hid.getRawButton(6);
    }

    // Deadband control
    public void setDeadband(double deadband) {
        this.deadband = Math.max(0, Math.min(deadband, 1));
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
}
