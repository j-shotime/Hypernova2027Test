package first.robot.subsystems.drivetrain;

import org.wpilib.units.Units;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.LinearVelocity;
import org.wpilib.units.measure.Voltage;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

public class SwerveModule {
    private final TalonFX driveMotor;
    private final TalonFX steerMotor;
    private Angle currentSteerAngle = Units.Radians.zero();

    public SwerveModule(TalonFX driveMotor, TalonFX steerMotor) {
        this.driveMotor = driveMotor;
        this.steerMotor = steerMotor;
    }
    public SwerveModule(int driveMotorId, int steerMotorId) {
        this(new TalonFX(driveMotorId, CANBus.systemcore(0)), new TalonFX(steerMotorId, CANBus.systemcore(0)));
    }

    public SwerveModule(int driveMotorId, int steerMotorId, int busId) {
        this(new TalonFX(driveMotorId, CANBus.systemcore(busId)), new TalonFX(steerMotorId, CANBus.systemcore(busId)));
    }

    public SwerveModule(int driveMotorId, int driveBusId, int steerMotorId, int steerBusId) {
        this(new TalonFX(driveMotorId, CANBus.systemcore(driveBusId)), new TalonFX(steerMotorId, CANBus.systemcore(steerBusId)));
    }

    public void set(VoltageVector voltageVector) {
        Voltage driveVoltage = voltageVector.voltage();
        Angle steerAngle = voltageVector.angle();

        double currentRadians = currentSteerAngle.in(Units.Radians);
        double targetRadians = steerAngle.in(Units.Radians);
        double deltaRadians = targetRadians - currentRadians;

        while (deltaRadians > Math.PI) {
            deltaRadians -= 2 * Math.PI;
        }
        while (deltaRadians < -Math.PI) {
            deltaRadians += 2 * Math.PI;
        }

        if (Math.abs(deltaRadians) > Math.PI / 2) {
            driveVoltage = driveVoltage.times(-1);
            steerAngle = steerAngle.plus(Units.Degrees.of(180));
        }

        driveMotor.setVoltage(driveVoltage);
        if(voltageVector.voltage().in(Units.Volts) == 0) {
            return;
        }
        steerMotor.setControl(new PositionVoltage(steerAngle));
        currentSteerAngle = steerAngle;
    }
    public void set(VelocityVector velocityVector) {
        throw new UnsupportedOperationException("VelocityVector is not yet implemented");
    }

    public void resetEncoders() {
        if (driveMotor != null) {
            driveMotor.setPosition(0);
        }
        if (steerMotor != null) {
            steerMotor.setPosition(0);
        }
        currentSteerAngle = Units.Radians.zero();
    }
}
