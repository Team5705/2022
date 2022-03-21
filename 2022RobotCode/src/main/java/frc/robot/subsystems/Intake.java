// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstant;

public class Intake extends SubsystemBase {
  private final WPI_TalonSRX motor = new WPI_TalonSRX(IntakeConstant.m1);

  /** Creates a new Intake. */
  public Intake() {
    motor.configFactoryDefault();
    //motor.setInverted(true);
  }

  public void foward() {
    motor.set(1);
  }

  public void reverse() {
    motor.set(-1);
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
