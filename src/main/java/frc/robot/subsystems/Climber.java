// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLimitSwitch;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.settings.Constants.ClimberConstants;

public class Climber extends SubsystemBase {
  SparkMax climbMotorR;
  SparkMax climbMotorL;
  SparkMaxConfig climbMotorRConfig;
  SparkMaxConfig climbMotorLConfig;
  RelativeEncoder climbEncoderR;
  RelativeEncoder climbEncoderL;
  EncoderConfig climbEncoderConfig;
  double speed;
  double voltage;
  double currentEncoderRotationsL;
  double currentEncoderRotationsR;
  LimitSwitchConfig limitSwitchConfig;
  double initialEncoderRotationsL;
  double initialEncoderRotationsR;
  SparkLimitSwitch hallEffectR;
  SparkLimitSwitch hallEffectL;
  Boolean leftClimberOn;
  Boolean rightClimberOn;
  double runSpeedL;
  double runSpeedR;
    /** Creates a new Climber. */
  public Climber() {
    runSpeedL = 0;
    runSpeedR = 0;
    climbEncoderConfig = new EncoderConfig();
    climbEncoderConfig.countsPerRevolution(42);
    limitSwitchConfig = new LimitSwitchConfig();
    limitSwitchConfig.forwardLimitSwitchType(Type.kNormallyOpen);
    //Creates the left and right motors, and their limit switches
    climbMotorR = new SparkMax(ClimberConstants.CLIMBER_MOTOR_RIGHT, MotorType.kBrushless);
    climbMotorL = new SparkMax(ClimberConstants.CLIMBER_MOTOR_LEFT, MotorType.kBrushless);
    climbMotorR.setInverted(true);
    climbMotorL.setInverted(true);
    //creates and defines the configurations for the Left climber motor
    climbMotorLConfig = new SparkMaxConfig();
    climbMotorLConfig.idleMode(IdleMode.kBrake);
    climbMotorLConfig.apply(climbEncoderConfig);
    climbMotorLConfig.apply(limitSwitchConfig);
    climbMotorL.configure(climbMotorLConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    //creates and defines the configurations for the right climber motor
    climbMotorRConfig = new SparkMaxConfig();
    climbMotorRConfig.idleMode(IdleMode.kBrake);
    climbMotorRConfig.apply(climbEncoderConfig);
    climbMotorRConfig.apply(limitSwitchConfig);
    climbMotorR.configure(climbMotorRConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
    climbEncoderL = climbMotorL.getEncoder();
    climbEncoderR = climbMotorR.getEncoder();
    initialEncoderRotationsL = Math.abs(climbEncoderL.getPosition());
    initialEncoderRotationsL = Math.abs(climbEncoderR.getPosition());

    hallEffectL = climbMotorL.getForwardLimitSwitch();
    hallEffectR = climbMotorR.getForwardLimitSwitch();
  }
  public void climberGoLeft(double speed){
    runSpeedL = speed;
 }
 
 public void climberGoRight(double speed){
  runSpeedR = speed;
 }

 public void climberStop(){
  climbMotorR.set(0);
  climbMotorL.set(0);
 }

 public void climberVoltage(double voltage){
  climbMotorR.setVoltage(voltage);
  climbMotorL.setVoltage(voltage);
 }

 public double getRightRPM(){
  return climbEncoderR.getVelocity();
 }

 public double getLeftRPM(){
  return climbEncoderL.getVelocity();
 }

 public boolean isClimberIn(){
  return (hallEffectL.isPressed() && hallEffectR.isPressed());
}

public void resetInitial(){
  initialEncoderRotationsL = climbEncoderL.getPosition();
  initialEncoderRotationsR = climbEncoderR.getPosition();
}
@Override
public void periodic() {
  currentEncoderRotationsL = Math.abs(Math.abs(climbEncoderL.getPosition()) - initialEncoderRotationsL);
  currentEncoderRotationsR = Math.abs(Math.abs(climbEncoderR.getPosition()) - initialEncoderRotationsR);
  SmartDashboard.putNumber("LEFT rotations since start", currentEncoderRotationsL);
  SmartDashboard.putNumber("RIGHT rotations since start", currentEncoderRotationsR);
  SmartDashboard.putBoolean("left hall effect value", hallEffectL.isPressed());
  SmartDashboard.putBoolean("right hall effect value", hallEffectR.isPressed());
  double rSpeed = 0;
  double lSpeed = 0;

  if(runSpeedL>0){
    if(!hallEffectL.isPressed()){
      lSpeed = runSpeedL;
    }
  }
  else{
    if (currentEncoderRotationsL < ClimberConstants.MAX_MOTOR_ROTATIONS){
      lSpeed = runSpeedL;
    }
  }
  
  if(runSpeedR>0){
    if(!hallEffectR.isPressed()){
      rSpeed = runSpeedR;
    }
  }
  else{
    if (currentEncoderRotationsR < ClimberConstants.MAX_MOTOR_ROTATIONS){
      rSpeed = runSpeedR;
    }
  }
  climbMotorL.set(lSpeed);
  climbMotorR.set(rSpeed);
}
}