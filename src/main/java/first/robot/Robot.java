// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package first.robot;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Rotation;
import static org.wpilib.units.Units.Rotations;
import static org.wpilib.units.Units.Volt;

import java.io.PrintStream;
import java.lang.annotation.ElementType;
import java.util.Vector;

import org.opencv.core.Mat;
import org.wpilib.command2.Command;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.command2.InstantCommand;
import org.wpilib.framework.TimedRobot;
import org.wpilib.networktables.DoublePublisher;
import org.wpilib.networktables.DoubleSubscriber;
import org.wpilib.networktables.LogMessage;
import org.wpilib.networktables.NetworkTable;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.networktables.NetworkTableValue;
import org.wpilib.networktables.NetworkTablesJNI;
import org.wpilib.smartdashboard.SmartDashboard;
import org.wpilib.system.Timer;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.Voltage;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import first.robot.subsystems.drivetrain.*;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends TimedRobot {
  private Command autonomousCommand;

  private final RobotContainer robotContainer;
  
  TalonFX BackRightSteer = new TalonFX(0, CANBus.systemcore(0)); 
  TalonFX BackRightDrive = new TalonFX(1, CANBus.systemcore(0));
  TalonFX FrontRightSteer = new TalonFX(2, CANBus.systemcore(0));
  TalonFX FrontRightDrive = new TalonFX(3, CANBus.systemcore(0));
  TalonFX BackLeftSteer = new TalonFX(0, CANBus.systemcore(1));
  TalonFX BackLeftDrive = new TalonFX(1, CANBus.systemcore(1));
  TalonFX FrontLeftSteer = new TalonFX(2, CANBus.systemcore(1));
  TalonFX FrontLeftDrive = new TalonFX(3, CANBus.systemcore(1));
  SwerveModule BackRightModule = new SwerveModule(BackRightDrive, BackRightSteer);
  SwerveModule FrontRightModule = new SwerveModule(FrontRightDrive, FrontRightSteer);
  SwerveModule BackLeftModule = new SwerveModule(BackLeftDrive, BackLeftSteer);
  SwerveModule FrontLeftModule = new SwerveModule(FrontLeftDrive, FrontLeftSteer);
  SwerveDrive swerveDrive = new SwerveDrive(21.75, 19.75, FrontLeftModule, FrontRightModule, BackLeftModule, BackRightModule);

  private final TalonFXConfiguration steerConfiguration = new TalonFXConfiguration()
  .withMotorOutput(
      new MotorOutputConfigs()
          .withNeutralMode(NeutralModeValue.Brake)
  )
  .withCurrentLimits(
      new CurrentLimitsConfigs()
          .withStatorCurrentLimit(Amps.of(60))
          .withSupplyCurrentLimit(Amps.of(40))
  )
  .withFeedback(
      new FeedbackConfigs()
          .withSensorToMechanismRatio(150.0/7.0)
  )
  .withSlot0(
      new Slot0Configs()
          .withKP(215)
          .withKD(5.35)
          .withKS(0.19)
  );

  private final TalonFXConfiguration driveConfiguration = new TalonFXConfiguration()
  .withMotorOutput(
      new MotorOutputConfigs()
          .withNeutralMode(NeutralModeValue.Brake)
  )
  .withCurrentLimits(
      new CurrentLimitsConfigs()
          .withStatorCurrentLimit(Amps.of(120))
          .withSupplyCurrentLimit(Amps.of(80))
  );
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    BackLeftSteer.getConfigurator().apply(steerConfiguration);
    FrontLeftSteer.getConfigurator().apply(steerConfiguration);
    BackRightSteer.getConfigurator().apply(steerConfiguration);
    FrontRightSteer.getConfigurator().apply(steerConfiguration);
    BackLeftDrive.getConfigurator().apply(driveConfiguration);
    FrontLeftDrive.getConfigurator().apply(driveConfiguration);
    BackRightDrive.getConfigurator().apply(driveConfiguration);
    FrontRightDrive.getConfigurator().apply(driveConfiguration);
    
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    SmartDashboard.putData("Reset", resetCommand);
    robotContainer = new RobotContainer();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and utility.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() 
  {
    
  }

  @Override
  public void disabledPeriodic() 
  {
  }

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    autonomousCommand = robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(autonomousCommand);
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}
  @Override
  public void teleopInit() {
    BackRightModule.set(new VoltageVector(Voltage.ofRelativeUnits(1, Volt), Angle.ofRelativeUnits(180, Degrees)));
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }
  
  Command resetCommand = new InstantCommand(() -> {
    BackRightSteer.setPosition(0);
    BackRightDrive.setPosition(0);
  });
  
  double t = 0;
  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() 
  {
  }

  @Override
  public void utilityInit() {
    // Cancels all running commands at the start of utility mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during utility mode. */
  @Override
  public void utilityPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
