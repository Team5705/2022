// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.GlobalConstant;
import frc.robot.Constants.Shoot;

public class Shooter extends SubsystemBase {
  private final CANSparkMax m1 = new CANSparkMax(Shoot.mShooter, MotorType.kBrushless);
  private final Servo leftServo = new Servo(0);
  private final Servo rightServo = new Servo(1);
  private final CANSparkMax m2 = new CANSparkMax(Shoot.mShooter2, MotorType.kBrushless);

  private final Compressor compressor = new Compressor(GlobalConstant.portPCM, PneumaticsModuleType.CTREPCM);

  //private final WPI_CANCoder encoder = new WPI_CANCoder(52);
  private CANCoderConfiguration encoderConfigs = new CANCoderConfiguration();

  private final double wheelDiameter = 6.00; //6 pulgadas

  private final double rampRate = 0.3;

  /** Creates a new Shooter. */
  public Shooter() {
    m1.restoreFactoryDefaults();
    m1.setOpenLoopRampRate(rampRate); //Declarar en constants

    m2.restoreFactoryDefaults();
    m2.setOpenLoopRampRate(rampRate);
    m2.setInverted(true);


    //Ajustamos los valores respecto a los datos del Smart Robot Servo
    leftServo.setBounds(2.5, 0, 1.5, 0, 0.5);
    rightServo.setBounds(2.5, 0, 1.5, 0, 0.5);

    encoderConfigs.sensorDirection = false; //Dirección del valor, ajustar si está invertido
    //encoderConfigs.sensorCoefficient = ; //Coeficiente para la salida de los valores, por defecto en grados (0.087890625) e.g. 4096 * 0.087890625 = 360°
    //encoder.configAllSettings(encoderConfigs);
  }
  
  public void moveFullClockwiseHood(){
    leftServo.setSpeed(1.0);
    rightServo.setSpeed(-1.0);
  }

  public void moveFullCounterClockwiseHood(){
    leftServo.setSpeed(-1.0);
    rightServo.setSpeed(1.0);
  }

  public void neutralHood(){
    leftServo.setSpeed(0.0);
    rightServo.setSpeed(0.0);
  }

  /**
   * 
   * @param speed De -1.0 a 1.0
   */
  public void moveHood(double speed){
    leftServo.setSpeed(speed);
    rightServo.setSpeed(-speed);
  }

  /**
   * Acciona el disparador
   * @param speed -1.0 a 1.0
   */
  public void shootMove(double speed){
    m1.setVoltage(12 * speed);
    m2.setVoltage(12 * speed);
    compressor.disable();
  }

  /**
   * Modo neutral para el disparador
   */
  public void neutral(){
    m1.set(0);
    m2.set(0);
    compressor.enableDigital();
  }

  /**
   * Obtiene la velocidad del disparador
   * @return Velocidad en RPM
   */
  public double getShootVelocity(){
    return m2.getEncoder().getVelocity();
  }

  /**
   * Obtiene la velocidad del disparador
   * @return Velocidad en m/s
   */
  public double getShootVelocityMeterPerSeconds(){
    double rps = getShootVelocity() / 60;
    return (Units.inchesToMeters(wheelDiameter) * Math.PI) * rps;
  }

  /* public double getPosition(){
    return encoder.getPosition();
  } */

  public double getHoodAngle(){
    return 0; //Cambiar!
  }

  /* public void resertEncoder(){
    encoder.setPosition(70.0);
  } */

  @Override
  public void periodic() {
    //SmartDashboard.putNumber("EncoderHoodPosition", getPosition());
    SmartDashboard.putNumber("Shooter_RPM", getShootVelocity());
    SmartDashboard.putNumber("Shooter_M-S", getShootVelocityMeterPerSeconds());
    //SmartDashboard.putNumber("Projectile_M-S", getShootVelocityMeterPerSeconds() * 0.2);
    SmartDashboard.putNumber("shootSpeed", m1.get());
    SmartDashboard.putNumber("powerShooter1Voltage", m1.getBusVoltage());
    SmartDashboard.putNumber("powerShooter2Voltage", m2.getBusVoltage());
  }
}
