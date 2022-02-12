// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.shoot;

public class pruebas extends SubsystemBase {
  private Spark torreta = new Spark(shoot.mShooter);
 private Spark torreta2 = new Spark(shoot.mShooter2);
  
  /** Creates a new pruebas. */
  public pruebas() {}

  /* public void moverIntake(double speed){
    intake.set(speed);
  } */

  public void moverTorreta(double speed){
    torreta.set(speed);
    torreta2.set(speed);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
