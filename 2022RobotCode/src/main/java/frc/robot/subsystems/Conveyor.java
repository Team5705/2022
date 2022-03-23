// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ConveyorConstant;

public class Conveyor extends SubsystemBase {
  private final CANSparkMax motorSUPP = new CANSparkMax(ConveyorConstant.m1 , MotorType.kBrushless);
  private final CANSparkMax motorADC = new CANSparkMax(ConveyorConstant.m2, MotorType.kBrushless);

  private final DigitalInput s1 = new DigitalInput(0),
                             s2 = new DigitalInput(1),
                             s3 = new DigitalInput(2);

  private boolean mainSensor = false,
                  space_1 = false,
                  space_2 = false;

  private final double speedGlobal = 0.4;

  private int count = 0;

  /** Creates a new Conveyor. */
  public Conveyor() {
    motorSUPP.restoreFactoryDefaults();
    motorSUPP.setInverted(true);

    motorADC.restoreFactoryDefaults();

    count = 0;

  }

  public void forward(){
    motorSUPP.set(speedGlobal);
    motorADC.set(speedGlobal);
  }

  public void reverse(){
    motorSUPP.set(-speedGlobal);
    motorADC.set(-speedGlobal);
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
    if (!mainSensor || (space_1 && space_2) ){
      neutral();
    }
    else {
      forward();
    }
  }

  public void updateCount(){
    if ( (space_1 && !space_2) || (!space_1 && space_2) ){
      count = 1;
    }
    else if (space_1 && space_2){
      count = 2;
    }
    else {
      count = 0;
    }
  }

  public int getCount(){
    return count;
  }



  @Override
  public void periodic() {
    mainSensor = !s1.get();
    space_1 = !s2.get();
    space_2 = !s3.get();

    SmartDashboard.putNumber("conveyorSpeed", motorSUPP.get());
  }
}
