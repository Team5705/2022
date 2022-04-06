// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GlobalConstant;
import frc.robot.Constants.IntakeConstant;

public class Intake extends SubsystemBase {
  private final WPI_TalonSRX motor = new WPI_TalonSRX(IntakeConstant.m1);
  private Solenoid left = new Solenoid(GlobalConstant.portPCM, PneumaticsModuleType.CTREPCM, 0);
  private Solenoid right = new Solenoid(GlobalConstant.portPCM, PneumaticsModuleType.CTREPCM, 4);

  private final double speedGlobal = 1.0;
  private final double rampRate = 1.0;

  /** Creates a new Intake. */
  public Intake() {
    motor.configFactoryDefault();
    motor.setInverted(true);
    motor.configOpenloopRamp(rampRate);

    contractIntake();
  }

 public void extendIntake(){
    left.set(true);
    right.set(true);
  }

  public void contractIntake(){
    left.set(false);
    right.set(false);
  }

  public boolean getStatusIntake(){
    boolean statusLeft = left.get();
    boolean statusRight = right.get();

    if (statusLeft && statusRight)
      return true;
    else
      return false;
  }

  public void foward() {
    if (getStatusIntake())
     motor.set(speedGlobal);
    else
     motor.set(0);
  }

  public void reverse() {
    if (getStatusIntake())
      motor.set(-speedGlobal);
    else
      motor.set(0); 
  }

  public void neutral() {
    motor.set(0);
  }

  public void move(double speed) {
    if (getStatusIntake())
      motor.set(speed);
    else
      motor.set(0);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("intakeSpeed", motor.get());
    SmartDashboard.putNumber("powerIntakeVoltage", motor.getBusVoltage());
  }
}
