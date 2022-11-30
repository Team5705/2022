// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;

public class Intake extends SubsystemBase {
  private final WPI_TalonSRX motor;
  //private DoubleSolenoid extensor = new DoubleSolenoid(kGlobal.portPCM, PneumaticsModuleType.CTREPCM, kIntake.solenoids[0], kIntake.solenoids[1]);
  //private Solenoid sole = new Solenoid(kGlobal.portPCM, PneumaticsModuleType.CTREPCM, kIntake.solenoids[0]);
  /* private Solenoid left = new Solenoid(kGlobal.portPCM, PneumaticsModuleType.CTREPCM, kIntake.solenoids[0]);
  private Solenoid right = new Solenoid(kGlobal.portPCM, PneumaticsModuleType.CTREPCM, kIntake.solenoids[1]); */

  private final double rampRate = 0.2;

  /** Creates a new Intake. */
  public Intake() {
    motor = new WPI_TalonSRX(kIntake.m1);
    motor.configFactoryDefault();
    motor.setInverted(true);
    motor.configOpenloopRamp(rampRate);
    

    //contractIntake();
  }

  /* public void extendIntake(){
   //extensor.set(Value.kReverse);
   sole.set(true);
  }

  public void contractIntake(){
    //extensor.set(Value.kForward);
    sole.set(false);
  }

  public boolean getStatusIntake(){
    //Value status =  extensor.get();
    boolean status = sole.get();
    //boolean statusLeft = left.get();
    //boolean statusRight = right.get();

    //if (status == Value.kForward)//&& statusRight)
    if(status)
      return true;
    else
      return false;
  } */

  public void forward() {
    /* if (getStatusIntake())
     motor.set(-kIntake.speed);
    else
     motor.set(0); */
     motor.set(-kIntake.speed);
  }

  public void reverse() {
    /* if (getStatusIntake())
      motor.set(kIntake.speed);
    else
      motor.set(0); */
      motor.set(kIntake.speed);
  }

  public void neutral() {
    motor.set(0);
  }

  /* public void move(double speed) {
    if (getStatusIntake())
      motor.set(-speed);
    else
      motor.set(0);
  } */

  @Override
  public void periodic() {
    SmartDashboard.putNumber("intakeSpeed", motor.get());
    //SmartDashboard.putBoolean("intakeDeployed", getStatusIntake());
    //SmartDashboard.putNumber("powerIntakeVoltage", motor.getBusVoltage());
  }

  
}
