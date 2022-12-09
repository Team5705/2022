// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kShooter;

public class Shooter extends SubsystemBase {
  private final CANSparkMax m1, m2;
  private SparkMaxPIDController m1_pidController, m2_pidController;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
  private RelativeEncoder encoder;

  //private final Compressor compressor = new Compressor(kGlobal.portPCM, PneumaticsModuleType.CTREPCM);

  private final double wheelDiameter = Units.inchesToMeters(6.00); //6 pulgadas

  //private final double rampRate = 0;//0.0001;
  //private final double speedTransfer = 0.35; // 35%

  public Shooter() {
    m1 = new CANSparkMax(kShooter.mShooterA, MotorType.kBrushless);
    m2 = new CANSparkMax(kShooter.mShooterB, MotorType.kBrushless);
    m1.restoreFactoryDefaults();
    m2.restoreFactoryDefaults();
    
    //m1.setOpenLoopRampRate(rampRate); //Declarar en constants
    //m2.setOpenLoopRampRate(rampRate);
    
    m2.setInverted(true);

    encoder = m2.getEncoder();

    //Añadimos el Factor de conversión a la que queremos usar nuestra velocidad, [m/s] metros por segundo
    m1.getEncoder().setVelocityConversionFactor( (wheelDiameter*Math.PI)/60 );
    encoder.setVelocityConversionFactor( (wheelDiameter*Math.PI)/60 );

    //Establecemos los valores de PIDF de cada SparMAX, convenientemente es igual para ambos
    kP = 0.0001;//170e-5;  //170e-5
    kI = 0.000001;    //5e-7
    kD = 0;  //380e-3
    kIz = 200;
    kFF = 0.0002;
    kMaxOutput = 1;
    kMinOutput = -1;

    m1_pidController = m1.getPIDController();
    m2_pidController = m2.getPIDController();
    
    m1_pidController.setP(kP);
    m1_pidController.setI(kI);
    m1_pidController.setD(kD);
    m1_pidController.setIZone(kIz);
    m1_pidController.setFF(kFF);
    m1_pidController.setOutputRange(kMinOutput, kMaxOutput);

    m2_pidController.setP(kP);
    m2_pidController.setI(kI);
    m2_pidController.setD(kD);
    m2_pidController.setIZone(kIz);
    m2_pidController.setFF(kFF);
    m2_pidController.setOutputRange(kMinOutput, kMaxOutput);

    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);
  }
  
  /**
   * Ajusta la velocidad dada por medio del PIDF integrado de cada controlador.
   * @param rpm Revoluciones por minuto
   */
  public void adjustRPM(double rpm){
    //double rpm = (mps*60) / Units.inchesToMeters(wheelDiameter*Math.PI); 
    m1.getPIDController().setReference(rpm, CANSparkMax.ControlType.kVelocity);
    m2.getPIDController().setReference(rpm, CANSparkMax.ControlType.kVelocity);

  }

  public void adjustMeterPerSecond(double mps){
    double rpm = mps*( 60 / (Math.PI*wheelDiameter) );
    m1.getPIDController().setReference(rpm, CANSparkMax.ControlType.kVelocity);
    m2.getPIDController().setReference(rpm, CANSparkMax.ControlType.kVelocity);
  }

  /**
   * Acciona el disparador
   * @param speed -1.0 a 1.0
   */
  public void shootMove(double speed){
    /* m1.setVoltage(12 * speed);
    m2.setVoltage(12 * speed); */
    m1.set(speed);
    m2.set(speed);
    //compressor.disable();
  }

  /**
   * Modo neutral para el disparador
   */
  public void neutral(){
    m1.set(0);
    m2.set(0);
    //compressor.enableDigital();
  }

  /**
   * Obtiene la velocidad del disparador
   * @return Velocidad en RPM
   */
  public double getShootVelocity(){
    //return (m1.getEncoder().getVelocity() + m2.getEncoder().getVelocity()) / 2; //Promedio
    return  encoder.getVelocity();
  }

  /**
   * Obtiene la velocidad del disparador
   * @return Velocidad en m/s
   */
  public double getShootVelocityMeterPerSeconds(){
    //return (m1.getEncoder().getVelocityConversionFactor() + m2.getEncoder().getVelocityConversionFactor())/2; //Promedio
    return  encoder.getVelocityConversionFactor();
  }

  public boolean getPressureSwitch(){
    return false;//compressor.getPressureSwitchValue();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter_Velocity", getShootVelocity());
    SmartDashboard.putNumber("Shooter_Velocity2", getShootVelocity());
    SmartDashboard.putNumber("Shooter_VelocityRPM", getShootVelocity()/m2.getEncoder().getVelocityConversionFactor());
    SmartDashboard.putNumber("Shooter_M-S", getShootVelocityMeterPerSeconds());
    //SmartDashboard.putNumber("Projectile_M-S", getShootVelocityMeterPerSeconds() * speedTransfer);
    SmartDashboard.putNumber("shooterSpeed", m2.get());
    //SmartDashboard.putNumber("powerShooter1Voltage", m1.getBusVoltage());
    SmartDashboard.putNumber("powerShooter2Voltage", m2.getBusVoltage());
    SmartDashboard.putBoolean("PressureSwitch", getPressureSwitch());

    // read PID coefficients from SmartDashboard
    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double iz = SmartDashboard.getNumber("I Zone", 0);
    double ff = SmartDashboard.getNumber("Feed Forward", 0);
    double max = SmartDashboard.getNumber("Max Output", 0);
    double min = SmartDashboard.getNumber("Min Output", 0);

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    /* if((p != kP)) { m1_pidController.setP(p); m2_pidController.setP(p); kP = p; }
    if((i != kI)) { m1_pidController.setI(i); m2_pidController.setI(i); kI = i; }
    if((d != kD)) { m1_pidController.setD(d); m2_pidController.setD(d); kD = d; }
    if((iz != kIz)) { m1_pidController.setIZone(iz); m2_pidController.setIZone(iz); kIz = iz; }
    if((ff != kFF)) { m1_pidController.setFF(ff); m2_pidController.setFF(ff); kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      m1_pidController.setOutputRange(min, max);
      m2_pidController.setOutputRange(min, max); 
      kMinOutput = min; kMaxOutput = max; 
    } */

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((p != kP)) { m2_pidController.setP(p); kP = p; }
    if((i != kI)) { m2_pidController.setI(i); kI = i; }
    if((d != kD)) { m2_pidController.setD(d); kD = d; }
    if((iz != kIz)) { m2_pidController.setIZone(iz); kIz = iz; }
    if((ff != kFF)) { m2_pidController.setFF(ff); kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      m2_pidController.setOutputRange(min, max); 
      kMinOutput = min; kMaxOutput = max; 
    }
  }
}

