// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Intake;

public class intake_test extends SubsystemBase {

  private Spark intake = new Spark(Intake.m1);
  private Spark conveyor = new Spark(Intake.m2);

  private final I2C.Port i2cPort = I2C.Port.kOnboard;
  private final ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);
  private Color detectedColor = null;

  private double red = 0;
  private double green = 0;
  private double blue = 0;
  private String colorString = null;

  private Solenoid solenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 0);

  private DigitalInput sensor1 = new DigitalInput(Intake.sensors[0]);
  private DigitalInput sensor2 = new DigitalInput(Intake.sensors[1]); 
  private DigitalInput sensor3 = new DigitalInput(Intake.sensors[2]);

  // aqui va la tableishon de verdadeishon
  private double intake_velocity = 0.9,
                 conveyor_velocity = 0.9;

  private boolean s1; //= !sensor1.get()
  private boolean s2; //= !sensor2.get()
  private boolean s3; //= !sensor3.get()

  private Alliance myAlliance;

  private boolean ready = false;

  private boolean intakeDeployed = false; // estado del intake 


  /** Creates a new intake_test. */
  public intake_test() {
    intake.setInverted(true); //asignar si es invertido o no 
    conveyor.setInverted(true);

    solenoid.set(false);

    myAlliance = DriverStation.getAlliance();

  }

  public boolean ready(){
    return ready;
  }

  public boolean intakeDeployed(){
    return intakeDeployed;
  }

  public void take(){
    intake.set(intake_velocity);
    conveyor.set(conveyor_velocity);
    
    solenoid.set(true);
  }

  public void neutral(){
    intake.set(0);
    conveyor.set(0);
    
  }

  public void takeBallsWithSensors(){
    if( !(myAlliance == detectColor()) ){
      conveyor.set(0);
      intake.set(-intake_velocity);
    } else {
      if (s1 && !s2 && s3  ||  !s1 && s2 && s3) {
        conveyor.set(0);
        intake.set(0);

        solenoid.set(false);
        ready = true;
      
      } else if(s1 && s2 && !s3) {
        conveyor.set(conveyor_velocity);
        intake.set(0);

        solenoid.set(false);
        ready = true;

      } else if((!s1 && !s2 && s3)  ||  (!s1 && s2 && !s3)) {
        conveyor.set(0);
        intake.set(intake_velocity);

        solenoid.set(true);
        ready = false;

      } else {
        conveyor.set(conveyor_velocity);
        intake.set(intake_velocity);

        solenoid.set(true);
        ready = false;
        
      }
    }
  }

  public void solenoideOn(){
    solenoid.set(true);
    intakeDeployed = true;
  }

  public void solenoideOff(){
    solenoid.set(false);
    intakeDeployed = false;
  }

  public Alliance detectColor(){
    SmartDashboard.putString("Color", colorString);

    if ((red > 0.30 && red < 0.42)  &&  (green > 0.35 && green < 0.45)  &&  (blue > 0.15 && blue < 0.23)) {
      colorString = "Red";
      return Alliance.Red;
     } else if ((red > 0.17 && red < 0.22)  &&  (green > 0.40 && green < 0.48)  &&  (blue > 0.34 && blue < 0.39)) {
       colorString = "Blue";
       return Alliance.Blue;
     } else {
       colorString = "None";
       return Alliance.Invalid;
     }
  }

  @Override
  public void periodic(){
    s1 = !sensor1.get();
    s2 = !sensor2.get();
    s3 = !sensor3.get();

    detectedColor = colorSensor.getColor();
    red = detectedColor.red;
    green = detectedColor.green;
    blue = detectedColor.blue;
    detectColor();


    SmartDashboard.putBoolean("1", s1);
    SmartDashboard.putBoolean("2", s2);
    SmartDashboard.putBoolean("3", s3);
  }
    // This method will be called once per scheduler run
  
 
}