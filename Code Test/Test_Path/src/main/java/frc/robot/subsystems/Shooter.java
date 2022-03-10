// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Shoot;

public class Shooter extends SubsystemBase {
  private final WPI_VictorSPX m1 = new WPI_VictorSPX(Shoot.mShooter);

  /** Creates a new Shooter. */
  public Shooter() {
    m1.configFactoryDefault();
    m1.configOpenloopRamp(1);
    //m1.setInverted(true);
  }

  public void shoot(double speed){
    m1.setVoltage(12*speed);
  }

  public void neutral(){
    m1.set(VictorSPXControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
