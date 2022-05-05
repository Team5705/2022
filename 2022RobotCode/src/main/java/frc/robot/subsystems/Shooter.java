// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kGlobal;
import frc.robot.Constants.kShooter;

public class Shooter extends SubsystemBase {
  private final CANSparkMax m1 = new CANSparkMax(kShooter.mShooterA, MotorType.kBrushless);
  private final CANSparkMax m2 = new CANSparkMax(kShooter.mShooterB, MotorType.kBrushless);

  private final Compressor compressor = new Compressor(kGlobal.portPCM, PneumaticsModuleType.CTREPCM);

  private final double wheelDiameter = 6.00; //6 pulgadas

  private final double rampRate = 0.0;

  private final double speedTransfer = 0.35; // 35%

  public Shooter() {
    m1.restoreFactoryDefaults();
    m1.setOpenLoopRampRate(rampRate); //Declarar en constants

    m2.restoreFactoryDefaults();
    m2.setOpenLoopRampRate(rampRate);
    m2.setInverted(true);

    //Añadimos el Factor de conversión a la que queremos usar nuestra velocidad, [m/s] metros por segundo
    m1.getEncoder().setVelocityConversionFactor((1/60) * Units.inchesToMeters(wheelDiameter) * Math.PI );
    m2.getEncoder().setVelocityConversionFactor((1/60) * Units.inchesToMeters(wheelDiameter) * Math.PI );

    //Establecemos los valores de PIDF de cada SparMAX, convenientemente es igual para ambos
    m1.getPIDController().setP(0.001, 0);
    m1.getPIDController().setI(0.000, 0);
    m1.getPIDController().setD(0.000, 0);
    m1.getPIDController().setFF(0.05, 0);

    m2.getPIDController().setP(0.001, 0);
    m2.getPIDController().setI(0.000, 0);
    m2.getPIDController().setD(0.000, 0);
    m2.getPIDController().setFF(0.05, 0);
  }
  
  /**
   * Ajusta la velocidad dada por medio del PIDF integrado de cada controlador.
   * @param mps [m/s] Metros por segundo / Meters per Second
   */
  public void adjustRPM(double mps){
    m1.getPIDController().setReference(mps, ControlType.kVelocity);
    m2.getPIDController().setReference(mps, ControlType.kVelocity);

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

  public boolean getPressureSwitch(){
    return compressor.getPressureSwitchValue();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter_Velocity", getShootVelocity());
    //SmartDashboard.putNumber("Shooter_M-S", getShootVelocityMeterPerSeconds());
    SmartDashboard.putNumber("Projectile_M-S", getShootVelocityMeterPerSeconds() * speedTransfer);
    SmartDashboard.putNumber("shooterSpeed", m1.get());
    SmartDashboard.putNumber("powerShooter1Voltage", m1.getBusVoltage());
    SmartDashboard.putNumber("powerShooter2Voltage", m2.getBusVoltage());
    SmartDashboard.putBoolean("PressureSwitch", getPressureSwitch());
  }
}
