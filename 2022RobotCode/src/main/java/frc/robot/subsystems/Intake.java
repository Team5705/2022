// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstant;

public class Intake extends SubsystemBase {
  private final WPI_TalonSRX motor = new WPI_TalonSRX(IntakeConstant.m1);
  private Solenoid left = new Solenoid(30, PneumaticsModuleType.CTREPCM, 0);
  private Solenoid right = new Solenoid(30, PneumaticsModuleType.CTREPCM, 4);

  private final double speedGlobal = 1.0;
  private final double rampRate = 1.0;

  /** Creates a new Intake. */
  public Intake() {
    motor.configFactoryDefault();
    motor.setInverted(true);
    motor.configOpenloopRamp(rampRate);

    //contractIntake();
  }

 public void extendIntake(){
    left.set(true);
    right.set(true);
  }

  public void contractIntake(){
    left.set(false);
    right.set(false);
  }

  public void foward() {
    motor.set(speedGlobal);
  }

  public void reverse() {
    motor.set(-speedGlobal);
  }

  public void neutral() {
    motor.set(0);
  }

  public void move(double speed) {
    motor.set(speed);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("intakeSpeed", motor.get());
  }
}
