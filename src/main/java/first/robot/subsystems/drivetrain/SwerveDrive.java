package first.robot.subsystems.drivetrain;

import org.wpilib.command2.SubsystemBase;
import org.wpilib.units.Units;
import org.wpilib.units.measure.Voltage;

public class SwerveDrive extends SubsystemBase {
    private final double widthInches;
    private final double heightInches;
    private final SwerveModule frontLeft;
    private final SwerveModule frontRight;
    private final SwerveModule backLeft;
    private final SwerveModule backRight;

    public SwerveDrive(double widthInches, double heightInches, SwerveModule frontLeft, SwerveModule frontRight, SwerveModule backLeft, SwerveModule backRight) {
        this.widthInches = widthInches;
        this.heightInches = heightInches;
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;
    }

    public SwerveDrive(
        double widthInches,
        double heightInches,
        int frontLeftDriveId, int frontLeftSteerId,
        int frontRightDriveId, int frontRightSteerId,
        int backLeftDriveId, int backLeftSteerId,
        int backRightDriveId, int backRightSteerId
    ) {
        this(
            widthInches,
            heightInches,
            new SwerveModule(frontLeftDriveId, frontLeftSteerId),
            new SwerveModule(frontRightDriveId, frontRightSteerId),
            new SwerveModule(backLeftDriveId, backLeftSteerId),
            new SwerveModule(backRightDriveId, backRightSteerId)
        );
    }

    public SwerveDrive(
        double widthInches,
        double heightInches,
        int frontLeftDriveId, int frontLeftDriveBusId, int frontLeftSteerId, int frontLeftSteerBusId,
        int frontRightDriveId, int frontRightDriveBusId, int frontRightSteerId, int frontRightSteerBusId,
        int backLeftDriveId, int backLeftDriveBusId, int backLeftSteerId, int backLeftSteerBusId,
        int backRightDriveId, int backRightDriveBusId, int backRightSteerId, int backRightSteerBusId
    ) {
        this(
            widthInches,
            heightInches,
            new SwerveModule(frontLeftDriveId, frontLeftDriveBusId, frontLeftSteerId, frontLeftSteerBusId),
            new SwerveModule(frontRightDriveId, frontRightDriveBusId, frontRightSteerId, frontRightSteerBusId),
            new SwerveModule(backLeftDriveId, backLeftDriveBusId, backLeftSteerId, backLeftSteerBusId),
            new SwerveModule(backRightDriveId, backRightDriveBusId, backRightSteerId, backRightSteerBusId)
        );
    }

    public double widthInches() {
        return widthInches;
    }

    public double heightInches() {
        return heightInches;
    }

    public SwerveModule frontLeft() {
        return frontLeft;
    }

    public SwerveModule frontRight() {
        return frontRight;
    }

    public SwerveModule backLeft() {
        return backLeft;
    }

    public SwerveModule backRight() {
        return backRight;
    }

    public void setModuleVelocities(VelocityVector frontLeftVelocity, VelocityVector frontRightVelocity, VelocityVector backLeftVelocity, VelocityVector backRightVelocity) {
        frontLeft.set(frontLeftVelocity);
        frontRight.set(frontRightVelocity);
        backLeft.set(backLeftVelocity);
        backRight.set(backRightVelocity);
    }

    public void setModuleVoltages(VoltageVector frontLeftVoltage, VoltageVector frontRightVoltage, VoltageVector backLeftVoltage, VoltageVector backRightVoltage) {
        frontLeft.set(frontLeftVoltage);
        frontRight.set(frontRightVoltage);
        backLeft.set(backLeftVoltage);
        backRight.set(backRightVoltage);
    }

    public void stop() {
        frontLeft.set(new VoltageVector(Units.Volts.zero(), Units.Radians.zero()));
        frontRight.set(new VoltageVector(Units.Volts.zero(), Units.Radians.zero()));
        backLeft.set(new VoltageVector(Units.Volts.zero(), Units.Radians.zero()));
        backRight.set(new VoltageVector(Units.Volts.zero(), Units.Radians.zero()));
    }

    public void setDriveVoltage(Voltage voltage) {
        stop();
        frontLeft.set(new VoltageVector(voltage, Units.Radians.zero()));
        frontRight.set(new VoltageVector(voltage, Units.Radians.zero()));
        backLeft.set(new VoltageVector(voltage, Units.Radians.zero()));
        backRight.set(new VoltageVector(voltage, Units.Radians.zero()));
    }

    public void localArcadeDrive(double forward, double strafe, double rotation, double maxVoltageVolts) {
        double halfWidth = widthInches / 2.0;
        double halfHeight = heightInches / 2.0;
        double maxDrive = Math.max(1.0, Math.abs(forward) + Math.abs(strafe) + Math.abs(rotation));

        double flX = (strafe + rotation * halfWidth) / maxDrive;
        double flY = (forward + rotation * halfHeight) / maxDrive;
        double frX = (strafe - rotation * halfWidth) / maxDrive;
        double frY = (forward + rotation * halfHeight) / maxDrive;
        double blX = (strafe + rotation * halfWidth) / maxDrive;
        double blY = (forward - rotation * halfHeight) / maxDrive;
        double brX = (strafe - rotation * halfWidth) / maxDrive;
        double brY = (forward - rotation * halfHeight) / maxDrive;

        double flMagnitude = Math.hypot(flX, flY);
        double frMagnitude = Math.hypot(frX, frY);
        double blMagnitude = Math.hypot(blX, blY);
        double brMagnitude = Math.hypot(brX, brY);
        double largestMagnitude = Math.max(1.0, Math.max(flMagnitude, Math.max(frMagnitude, Math.max(blMagnitude, brMagnitude))));

        setModuleVoltages(
            new VoltageVector(Units.Volts.of((flMagnitude / largestMagnitude) * maxVoltageVolts), Units.Radians.of(Math.atan2(flY, flX))),
            new VoltageVector(Units.Volts.of((frMagnitude / largestMagnitude) * maxVoltageVolts), Units.Radians.of(Math.atan2(frY, frX))),
            new VoltageVector(Units.Volts.of((blMagnitude / largestMagnitude) * maxVoltageVolts), Units.Radians.of(Math.atan2(blY, blX))),
            new VoltageVector(Units.Volts.of((brMagnitude / largestMagnitude) * maxVoltageVolts), Units.Radians.of(Math.atan2(brY, brX)))
        );
    }

    public void localArcadeDrive(double forward, double strafe, double rotation) {
        localArcadeDrive(forward, strafe, rotation, 12.0);
    }
}
