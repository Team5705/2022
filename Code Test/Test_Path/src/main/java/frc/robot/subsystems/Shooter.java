// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.Shoot;

public class Shooter extends SubsystemBase {
  private WPI_VictorSPX shoot = new WPI_VictorSPX(Shoot.mShooter);
  private WPI_VictorSPX shoot2 = new WPI_VictorSPX(Shoot.mShooter2);

  private Compressor compressor = new Compressor(38, PneumaticsModuleType.CTREPCM);

  public Shooter() {
    shoot.configFactoryDefault();
    shoot.configOpenloopRamp(0.2);

    shoot2.configFactoryDefault();
    shoot2.configOpenloopRamp(0.2);

  }

  public void go() {
    // Apaga el compresor (esté o no conectado) cuando se active el disparador y así utilizar toda la
    // batería
    compressor.disable();

    shoot.set(ControlMode.PercentOutput, 1);
    shoot2.set(ControlMode.PercentOutput, 1); // 0.9 es el valor óptimo sin utilizar el valor máximo y sin ser poco

    RobotContainer.leds.sendData(2);

  }

  public void neutral() {

    shoot.set(0);
    shoot2.set(0);

    RobotContainer.leds.sendData(3);

    // Vuelve a activar el compresor si es necesario
   // compressor.start();
  }

  public void goPID(double speed) {
    if (speed >= 0.8)
      speed = 0.8;
    else if (speed <= 0)
      speed = 0;
    shoot.setVoltage(speed);
  }

  @Override
  public void periodic() {
    //SmartDashboard.putBoolean("CompressorEnable?", compressor.enabled());

  }
}