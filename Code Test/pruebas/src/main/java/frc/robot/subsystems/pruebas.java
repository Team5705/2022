// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.shoot;

public class pruebas extends SubsystemBase {
  private Spark intake= new Spark(4);
  private Spark torreta = new Spark(5);
 private Spark torreta2 = new Spark(6);
  //private static final int torreta = 5;
  //private static final int torreta2 = 6;
  
  /** Creates a new pruebas. */
  public pruebas() {
  

  }

  public void moverIntake(double speed){
    intake.set(speed);
  }

  public void moverTorreta(double speed){
    torreta.set(speed);
    torreta2.set(speed);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
