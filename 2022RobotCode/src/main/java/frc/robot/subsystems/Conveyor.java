// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.kConveyor;

public class Conveyor extends SubsystemBase {
  private final CANSparkMax motorSUPP, motorADC;

  private final DigitalInput s1;

  private boolean mainSensor = false;

  int count = 0;

  /** Creates a new Conveyor. */
  public Conveyor() {
    motorSUPP = new CANSparkMax(kConveyor.mSUPP , MotorType.kBrushless);
    motorADC = new CANSparkMax(kConveyor.mADC, MotorType.kBrushless);

    s1 = new DigitalInput(kConveyor.mainSensor);

    //motorSUPP.restoreFactoryDefaults();
    motorSUPP.setInverted(true);
    //motorADC.restoreFactoryDefaults();

    motorSUPP.setIdleMode(IdleMode.kBrake);
    motorADC.setIdleMode(IdleMode.kBrake);


    count = 0;

  }

  public void forward(){
    motorSUPP.set(kConveyor.kSpeedGlobal);
    motorADC.set(kConveyor.kSpeedGlobal);
  }

  public void reverse(){
    motorSUPP.set(-kConveyor.kSpeedGlobal);
    motorADC.set(-kConveyor.kSpeedGlobal);
  }

  public void neutral(){
    motorSUPP.set(0);
    motorADC.set(0);
  }

  public void move(double speed){
    motorSUPP.set(speed);
    motorADC.set(speed);
  }

  public void getCargoWithSensor(){
    if (!mainSensor) {// || (space_1 && space_2) ){
      neutral();
    }
    else {
      forward();
    }
  }

  /* public void updateCount(){
    if ( (space_1 && !space_2) || (!space_1 && space_2) ){
      count = 1;
    }
    else if (space_1 && space_2){
      count = 2;
    }
    else {
      count = 0;
    }
  } */

  public void resetCount(){
    count = 0;
  }

  public int getCount(){
    return count;
  }

  @Override
  public void periodic() {
    mainSensor = !s1.get();
    //space_1 = !s2.get();
    //space_2 = !s3.get();

    SmartDashboard.putNumber("conveyorSpeed", motorSUPP.get());
    SmartDashboard.putBoolean("mainSensor", mainSensor);
  }
}
