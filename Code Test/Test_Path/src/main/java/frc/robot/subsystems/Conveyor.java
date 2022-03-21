// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ConveyorConstant;

public class Conveyor extends SubsystemBase {
  private final CANSparkMax motorSUPP = new CANSparkMax(ConveyorConstant.m1 , MotorType.kBrushed);

  /** Creates a new Conveyor. */
  public Conveyor() {
    motorSUPP.restoreFactoryDefaults();
    motorSUPP.setInverted(true);

  }

  public void forward(){
    motorSUPP.set(0.5);
  }

  public void reverse(){
    motorSUPP.set(-0.5);
  }

  public void neutral(){
    motorSUPP.set(0);
  }

  public void move(double speed){
    motorSUPP.set(speed);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("conveyorSpeed", motorSUPP.get());
  }
}
