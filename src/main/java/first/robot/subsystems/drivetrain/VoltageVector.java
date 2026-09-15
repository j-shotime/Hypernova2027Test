package first.robot.subsystems.drivetrain;

import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.Voltage;
import org.wpilib.units.Units;

public record VoltageVector(
    Voltage voltage,
    Angle angle
) 
{
    public VoltageVector(Voltage voltage, Angle angle) 
    {
        this.voltage = voltage;
        this.angle = normalize(angle);
    }

    private static Angle normalize(Angle angle) {
        double radians = angle.in(Units.Radians);

        radians %= 2 * Math.PI;

        if (radians < 0) {
            radians += 2 * Math.PI;
        }

        return Units.Radians.of(radians);
    }

    public Voltage x() 
    {
        return voltage.times(Math.cos(angle.in(Units.Radians)));
    }

    public Voltage y() 
    {
        return voltage.times(Math.sin(angle.in(Units.Radians)));
    }
}