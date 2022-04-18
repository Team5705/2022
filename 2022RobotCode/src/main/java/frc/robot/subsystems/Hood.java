// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.WPI_CANCoder;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kShooter;

public class Hood extends SubsystemBase {
  private final Servo leftServo = new Servo(kShooter.servoLeft);
  private final Servo rightServo = new Servo(kShooter.servoRight);

  private final WPI_CANCoder encoder = new WPI_CANCoder(52);
  private CANCoderConfiguration encoderConfigs = new CANCoderConfiguration();

  public Hood() {
    //Ajustamos los valores respecto a los datos del Smart Robot Servo
    leftServo.setBounds(2.5, 0, 1.5, 0, 0.5);
    rightServo.setBounds(2.5, 0, 1.5, 0, 0.5);
    
    encoderConfigs.sensorDirection = false; //Dirección del valor, ajustar si está invertido
    encoderConfigs.sensorCoefficient = 0.00590130267373473202554782341994; //Coeficiente para la salida de los valores, por defecto en grados (0.087890625) e.g. 4096 * 0.087890625 = 360°
    encoder.configAllSettings(encoderConfigs);

    resetEncoder();
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
   * Limitado desde 70° a 55°
   * @param speed De -1.0 a 1.0
   */
  public void moveHood(double speed){
    /* if(speed > 0){
      if (getPosition() <= 55){
        leftServo.setSpeed(0);
        rightServo.setSpeed(0);  
      }
      else{
        leftServo.setSpeed(speed);
        rightServo.setSpeed(-speed);
      }
    }

    else if(speed < 0){
      if (getPosition() >= 70){
        leftServo.setSpeed(0);
        rightServo.setSpeed(0);
      }
      else{
        leftServo.setSpeed(speed);
        rightServo.setSpeed(-speed);
      }
    }
    
    else{
      leftServo.setSpeed(0);
      rightServo.setSpeed(0);
    }  */ 
    leftServo.setSpeed(speed);
    rightServo.setSpeed(-speed);
  }

  public double getPosition(){
    return encoder.getPosition();
  }

  public void resetEncoder(){
    encoder.setPosition(70.0);
  }

  public double getHoodAngle(){
    return 0; //Cambiar!
  }


  @Override
  public void periodic() {
    SmartDashboard.putNumber("hoodAngle", getPosition());
  }
}
