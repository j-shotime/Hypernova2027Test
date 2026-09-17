package first.robot.subsystems.drivetrain;

import static org.wpilib.units.Units.Degrees;

import org.wpilib.command2.SubsystemBase;
import org.wpilib.units.Units;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.Voltage;

public class SwerveDrive extends SubsystemBase {
    private static final double TRANSLATION_ANGLE_HOLD_THRESHOLD = 0.08;
    private static final double TRANSLATION_ANGLE_DENOISE_ALPHA = 0.18;
    private final double widthInches;
    private final double heightInches;
    private final SwerveModule frontLeft;
    private final SwerveModule frontRight;
    private final SwerveModule backLeft;
    private final SwerveModule backRight;
    private Angle rotationAngle;
    private Angle lastTranslationAngle = Units.Radians.zero();

    public SwerveDrive(double widthInches, double heightInches, SwerveModule frontLeft, SwerveModule frontRight, SwerveModule backLeft, SwerveModule backRight) {
        this.widthInches = widthInches;
        this.heightInches = heightInches;
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;
        rotationAngle = Units.Radians.of(Math.atan2(widthInches, heightInches)+(Math.PI/2));
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
        if (Math.abs(forward) < 1e-6 && Math.abs(strafe) < 1e-6 && Math.abs(rotation) < 1e-6) {
            stop();
            return;
        }

        double translationMagnitude = Math.hypot(forward, strafe);
        if (translationMagnitude >= TRANSLATION_ANGLE_HOLD_THRESHOLD) {
            Angle targetTranslationAngle = Units.Radians.of(Math.atan2(strafe, forward));
            lastTranslationAngle = blendAngles(lastTranslationAngle, targetTranslationAngle, TRANSLATION_ANGLE_DENOISE_ALPHA);
        }

        VoltageVector DriveVector = new VoltageVector(Units.Volts.of(translationMagnitude * maxVoltageVolts / 2), lastTranslationAngle);

        // Get rotation vector components
        double rotX = rotation * maxVoltageVolts / 2 * Math.cos(rotationAngle.in(Units.Radians));
        double rotY = rotation * maxVoltageVolts / 2 * Math.sin(rotationAngle.in(Units.Radians));

        // Module 1 (frontLeft): horizontal flip (negate x)
        Angle angle1 = Units.Radians.of(Math.atan2(rotY, -rotX));
        // Module 2 (frontRight): normal
        Angle angle2 = rotationAngle;
        // Module 3 (backLeft): normal (180 offset)
        Angle angle3 = rotationAngle.plus(Degrees.of(180));
        // Module 4 (backRight): vertical flip (negate y)
        Angle angle4 = Units.Radians.of(Math.atan2(-rotY, rotX));

        setModuleVoltages(
            DriveVector.addVector(new VoltageVector(Units.Volts.of(Math.abs(rotation)*maxVoltageVolts/2), angle1)),
            DriveVector.addVector(new VoltageVector(Units.Volts.of(rotation*maxVoltageVolts/2), angle2)),
            DriveVector.addVector(new VoltageVector(Units.Volts.of(rotation*maxVoltageVolts/2), angle3)),
            DriveVector.addVector(new VoltageVector(Units.Volts.of(Math.abs(rotation)*maxVoltageVolts/2), angle4))
        );
    }

    public void localArcadeDrive(double forward, double strafe, double rotation) {
        localArcadeDrive(forward, strafe, rotation, 12.0);
    }

    public void resetEncoders() {
        frontLeft.resetEncoders();
        frontRight.resetEncoders();
        backLeft.resetEncoders();
        backRight.resetEncoders();
    }

    private static Angle blendAngles(Angle current, Angle target, double alpha) {
        double currentRadians = current.in(Units.Radians);
        double targetRadians = target.in(Units.Radians);
        double deltaRadians = targetRadians - currentRadians;

        while (deltaRadians > Math.PI) {
            deltaRadians -= 2 * Math.PI;
        }
        while (deltaRadians < -Math.PI) {
            deltaRadians += 2 * Math.PI;
        }

        return Units.Radians.of(currentRadians + deltaRadians * alpha);
    }
}
