package frc.robot.subsystems;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Intake;

public class intake_test extends SubsystemBase {

  private Spark intake = new Spark(Intake.m1);
  private Spark conveyor = new Spark(Intake.m2);

  private final ColorSensorV3 colorSensor = new ColorSensorV3(Intake.colorSensorPort);
  private Color detectedColor = null;
  private double red = 0;
  private double green = 0;
  private double blue = 0;
  private String colorString = null;

  private Solenoid solenoid = new Solenoid(PneumaticsModuleType.CTREPCM, Intake.solenoidPort);

  private DigitalInput sensor1 = new DigitalInput(Intake.sensors[0]);
  private DigitalInput sensor2 = new DigitalInput(Intake.sensors[1]); 
  private DigitalInput sensor3 = new DigitalInput(Intake.sensors[2]);

  private double intake_velocity = Intake.intakeVelocity,
                 conveyor_velocity = Intake.conveyorVelocity;

  private boolean s1 = false;
  private boolean s2 = false;
  private boolean s3 = false;

  private Alliance myAlliance = null;

  public static boolean readyShooter = false;
  private boolean intakeDeployed = false; // estado del intake 
  private boolean savedIntake = false;


  /** Creates a new intake_test. */
  public intake_test() {
    intake.setInverted(true); //asignar si es invertido o no 
    conveyor.setInverted(true);

    solenoid.set(false);

    myAlliance = DriverStation.getAlliance();
  }

  public boolean ready(){
    return readyShooter;
  }

  public boolean intakeDeployed(){
    return intakeDeployed;

  
  }

  public boolean savedIntake(){
    return savedIntake;
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

  /**
   * Funcion que ejecuta la rutina para tomar las cargas. Con ayuda del sensor de color de REV version 3 detecta el color de la pelota y
   * decide si debe expulsarla o no. Acomoda las pelotas según vayan entrando con ayuda de 3 sensores IR y de ya no poder tomar más cargas
   * guarda e inhabilita el intake para ya no tomar más.
   */
  public void takeBallsWithSensors(){
    if( !(myAlliance == detectBallColor()) ){
      conveyor.set(0);
      intake.set(-intake_velocity);
      Timer.delay(0.2); //delay para dalre tiempo al intake de sacar la pelota.
    } else {
        if (s1 && !s2 && s3  ||  !s1 && s2 && s3) {
          conveyor.set(0);
          intake.set(0);

          solenoid.set(false);
          readyShooter = true;
        
        } else if(s1 && s2 && !s3) {
          conveyor.set(conveyor_velocity);
          intake.set(0);

          solenoid.set(false);
          readyShooter = true;

          } else if((!s1 && !s2 && s3)  ||  (!s1 && s2 && !s3)) {
              conveyor.set(0);
              intake.set(intake_velocity);

              solenoid.set(true);
              readyShooter = false;

            } else {
                conveyor.set(conveyor_velocity);
                intake.set(intake_velocity);

                solenoid.set(true);
                readyShooter = false;
                
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

  public Alliance detectBallColor(){
    if ((red > 0.30 && red < 0.42)  &&  (green > 0.35 && green < 0.45)  &&  (blue > 0.15 && blue < 0.23)) {
      colorString = "Red";
      return Alliance.Red;
     } else if ((red > 0.17 && red < 0.22)  &&  (green > 0.40 && green < 0.48)  &&  (blue > 0.34 && blue < 0.39)) {
       colorString = "Blue";
       return Alliance.Blue;
     } else {
       colorString = "None";
       return myAlliance; //Se declara igual para no afectar la lógica de la funcion detectBallColor()
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
    detectBallColor();
    
    SmartDashboard.putString("Color", colorString);
    SmartDashboard.putBoolean("1", s1);
    SmartDashboard.putBoolean("2", s2);
    SmartDashboard.putBoolean("3", s3);
  }
}