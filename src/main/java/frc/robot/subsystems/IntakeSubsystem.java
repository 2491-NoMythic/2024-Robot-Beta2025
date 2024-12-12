// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license https://software-metadata.revrobotics.com/REVLib-2024.jsonfile in the root
// directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkAnalogSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.helpers.MotorLogger;
import frc.robot.settings.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new Intake. */
  SparkMax intake1;

  ClosedLoopConfig intake1PIDConfig;
  SparkMaxConfig intake1Config;
  SparkMax intake2;
  ClosedLoopConfig intake2PIDConfig;
  SparkMaxConfig intake2Config;
  SparkMax intakeSideLeft;
  SparkMaxConfig intakeSideLeftConfig;
  SparkMax intakeSideRight;
  SparkMaxConfig intakeSideRightConfig;
  SparkAnalogSensor m_DistanceSensor;
  boolean isNoteHeld;

  MotorLogger motorLogger1;
  MotorLogger motorLogger2;
  DoubleLogEntry logDistance;
  BooleanLogEntry logNoteIn;

  double intakeRunSpeed;

  public IntakeSubsystem() {
    // creates and applies the configurations for the motor Intake1 and it's PID
    // configurator
    intake1PIDConfig = new ClosedLoopConfig();
    intake1Config = new SparkMaxConfig();
    intake1 = new SparkMax(IntakeConstants.INTAKE_1_MOTOR, SparkLowLevel.MotorType.kBrushless);
    intake1PIDConfig.pidf(
        IntakeConstants.INTAKE_1_kP,
        IntakeConstants.INTAKE_1_kI,
        IntakeConstants.INTAKE_1_kD,
        IntakeConstants.INTAKE_1_kFF);
    intake1.setInverted(true);
    intake1Config.idleMode(IdleMode.kCoast);
    intake1Config.smartCurrentLimit(25, 40, 1000);
    intake1Config.apply(intake1PIDConfig);
    intake1.configure(
        intake1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    // creates and applies the configurations for the motor Intake2 and it's PID
    // configurator
    intake2Config = new SparkMaxConfig();
    intake2PIDConfig = new ClosedLoopConfig();
    intake2 = new SparkMax(IntakeConstants.INTAKE_2_MOTOR, SparkLowLevel.MotorType.kBrushless);
    intake2PIDConfig.pidf(
        IntakeConstants.INTAKE_2_kP,
        IntakeConstants.INTAKE_2_kI,
        IntakeConstants.INTAKE_2_kD,
        IntakeConstants.INTAKE_2_kFF);
    intake2.setInverted(true);
    intake2Config.idleMode(IdleMode.kCoast);
    intake2Config.smartCurrentLimit(25, 40, 1000);
    intake2PIDConfig.apply(intake2PIDConfig);
    intake2.configure(
        intake2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    // defines the analog sensor
    m_DistanceSensor = intake1.getAnalog();

    if (Preferences.getBoolean("IntakeSideWheels", false)) {
      intakeSideLeft = new SparkMax(IntakeConstants.INTAKE_SIDE_MOTOR_LEFT, MotorType.kBrushless);
      intakeSideRight = new SparkMax(IntakeConstants.INTAKE_SIDE_MOTOR_RIGHT, MotorType.kBrushless);
      intakeSideLeft.setInverted(false);
      intakeSideRight.setInverted(true);
      intakeSideLeftConfig.idleMode(IdleMode.kCoast);
      intakeSideRightConfig.idleMode(IdleMode.kCoast);
      intakeSideLeftConfig.smartCurrentLimit(25, 40, 1000);
      intakeSideRightConfig.smartCurrentLimit(25, 40, 1000);
      intakeSideLeftConfig.encoder.positionConversionFactor(1);
      intakeSideRightConfig.encoder.positionConversionFactor(1);
      intakeSideLeft.configure(intakeSideLeftConfig, null, null);
      intakeSideRight.configure(intakeSideRightConfig, null, null);
    }

    if (Preferences.getBoolean("CompBot", true)) {
      m_DistanceSensor = intakeSideLeft.getAnalog();
    } else {
      m_DistanceSensor = intakeSideLeft.getAnalog();
    }

    DataLog log = DataLogManager.getLog();
    motorLogger1 = new MotorLogger(log, "/intake/motor1");
    motorLogger2 = new MotorLogger(log, "/intake/motor2");
    logDistance = new DoubleLogEntry(log, "/intake/noteDistance");
    logNoteIn = new BooleanLogEntry(log, "/intake/noteIn");
  }

  /**
   * sets the intakes speed. If the IntakeSideWheels preference is set to false, you can put
   * whatever number you want in as IntakeSideRunSpeed
   *
   * <p>uses percentage of full power
   *
   * @param intakeRunSpeed percentage of full power, from -1 to 1
   * @param intakeSideRunSpeed percentage of full power of the side wheels, from -1 to 1
   */
  public void intakeYes(double intakeRunSpeed, double intakeSideRunSpeed) {
    intake1.set(intakeRunSpeed);
    intake2.set(intakeRunSpeed);
    if (Preferences.getBoolean("IntakeSideWheels", false)) {
      intakeSideLeft.set(intakeSideRunSpeed);
      intakeSideRight.set(intakeSideRunSpeed);
    }
  }

  public void function(double variable) {
    intake1.set(variable);
  }

  // intakeSubsystem.function(0.5);
  public void setVelocity(double velocity) {
    intake1.getClosedLoopController().setReference(velocity, ControlType.kVelocity);
    intake2.getClosedLoopController().setReference(velocity, ControlType.kVelocity);
  }

  public void intakeSideWheels(double sideWheelRunSpeed) {
    intakeSideLeft.set(sideWheelRunSpeed);
    intakeSideRight.set(sideWheelRunSpeed);
  }

  /**
   * sets the intakes speed
   *
   * <p>uses percentage of full power
   *
   * @param intakeRunSpeed NEGATIVE percentage of full power
   */
  public void intakeNo(double intakeRunSpeed) {
    intake1.set(-intakeRunSpeed);
    intake2.set(-intakeRunSpeed);
    if (Preferences.getBoolean("IntakeSideWheels", false)) {
      intakeSideLeft.set(-intakeRunSpeed);
      intakeSideRight.set(-intakeRunSpeed);
    }
  }

  /** sets the intake's power to 0 */
  public void intakeOff() {
    intake1.set(0);
    intake2.set(0);
    if (Preferences.getBoolean("IntakeSideWheels", false)) {
      intakeSideLeft.set(0);
      intakeSideRight.set(0);
    }
  }

  /**
   * uses the distance sensor inside the indexer to tell if there is a note fully inside the indexer
   *
   * @return if the sensor sees something within it's range in front of it
   */
  public boolean isNoteSeen() {
    return m_DistanceSensor.getVoltage() < 2;
  }

  public boolean isNoteHeld() {
    return isNoteHeld;
  }

  public void setNoteHeld(boolean held) {
    isNoteHeld = held;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("voltage sensor output", m_DistanceSensor.getVoltage());
    SmartDashboard.putNumber("INTAKE/leading roller speed", intake1.getEncoder().getVelocity());
    SmartDashboard.putNumber("INTAKE/trailing roller speed", intake2.getEncoder().getVelocity());
    motorLogger1.log(intake1);
    motorLogger2.log(intake2);
    logDistance.append(m_DistanceSensor.getVoltage());
    logNoteIn.append(isNoteSeen());
    SmartDashboard.putBoolean("is note in", isNoteSeen());
    SmartDashboard.putBoolean("is note held", isNoteHeld());
    RobotState.getInstance().IsNoteHeld = isNoteSeen();
  }
}
