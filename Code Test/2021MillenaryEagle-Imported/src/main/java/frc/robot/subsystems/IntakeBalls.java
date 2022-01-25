  /*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

//import java.util.Arrays;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.Constants.Intake;

/**
 * Aqui se declara los componentes del Intake
 */
public class IntakeBalls extends SubsystemBase {
  private Spark intake = new Spark(Intake.m1); //Intake
  private Spark motorBandaA = new Spark(Intake.m2); //Banda A (Bufer)
  private Spark motorBandaB = new Spark(Intake.m3); //Banda B

  private Solenoid pistonA = new Solenoid(PneumaticsModuleType.CTREPCM, Intake.solenoids[0]);

  private DigitalInput sensor1 = new DigitalInput(Intake.sensors[0]), //Sensor Bufer
                       sensor2 = new DigitalInput(Intake.sensors[1]),
                       sensor3 = new DigitalInput(Intake.sensors[2]),
                       sensor4 = new DigitalInput(Intake.sensors[3]); //Sensor de salida

 //private boolean[] sensors = new boolean[]{};

                                                    //  s1     s2      s3    s4
  /*private boolean[][] tabladeverdad = new boolean[][]{{false, false, false, false}, // 1
                                                      {false, false, false, true},  // 2
                                                      {false, false, true,  false}, // 3
                                                      {false, false, true,  true},  // 4
                                                      {true,  false, true,  false}, // 5
                                                      {false, true,  false, false}, // 6
                                                      {false, true,  false, true},  // 7
                                                      {false, true,  true,  false}, // 8
                                                      {true,  false, true,  true},  // 9
                                                      {false, true,  true,  true},  // 10 
                                                      {true,  false, false, false}, // 11
                                                      {true,  false, false, true},  // 12
                                                      {true,  true,  false, false}, // 13
                                                      {true,  true,  true,  true},  // 14
                                                      {true,  true,  true,  false}, // 15
                                                      {true,  true,  false, true}}; // 16*/
                       
  //Velocidad de los motores
  private double intake_Velocity = 0.9,
  //regular la velocidad para procurar tener siempre el mismo disparo de salida y no aumentar la potencia con todo el recorrido
  bandaA_Velocity = 0.9,
  bandaB_Velocity = 0.9;
                       
  private boolean s1; //= !sensor1.get();
  private boolean s2; //= !sensor2.get();
  private boolean s3; //= !sensor3.get();
  private boolean s4; //= !sensor4.get();

  private boolean ready = false; 

  private boolean intakeDeployed = false; // Estado del intake (false = retraido | true = extendido)
  
  //TOMAR 80% AL TIRADOR!!! NO 100%  --->  AL FINAL SIEMPRE QUEDO 100%
  //LA VELOCIDAD DEL TIRADOR PODRIA SER VARIADA POR EL VALOR DADO POR LA LIMELIGHT

  public IntakeBalls() {
    
    //Asignar si es invertido o no
    intake.setInverted(true);
    motorBandaA.setInverted(true);
    motorBandaB.setInverted(false);

  }

  /**
   * Tomar pelotas
   */
  public void take(){
    intake.set(intake_Velocity);
    motorBandaA.set(bandaA_Velocity);
    motorBandaB.set(bandaB_Velocity);

  }

  public void neutral(){
    intake.set(0);
    motorBandaA.set(0);
    motorBandaB.set(0);
  }
 
  /**
   * Dada una lógica sobre cuántas power cells tenemos en posesión asegura que mientras se ejecuta este método no existan espacios muertos
   * y pasen poco a poco sin llegar a tocar el disparador.
   */
  public void takeBalls(){

    /**
     * Para la toma de decisiones de esta logica se tomo como referencia una tabla de verdad para 
     * los casos posibles y redundantes.
     * 
     * Modo Simplificado
     */
    if (s1 == true && s4 == true){ //1--1
      //Se detienen las bandas
      RobotContainer.leds.sendData2(4); //Prender Led de control (Listo para tirar)
      intake.set(intake_Velocity);
      motorBandaA.set(0);
      motorBandaB.set(0);

      ready = true;
    } 

    else if (s1 == false && s4 == true ||
             s1 == false && s2 == true && s4 == false ||
             s1 ==  false && s2 == false && s3 == true && s4 == false
             ){ // 0 - - 1
      intake.set(intake_Velocity);
      motorBandaA.set(bandaA_Velocity);
      motorBandaB.set(0);

      ready = false;
    }

    else if(s1 == true && s2 == false && s3 == false && s4 == false ||
            s1 == true && s2 == false && s3 == true && s4 == false ||
            s1 == true && s2 == true && s3 == false && s4 == false ||
            s1 == true && s2 == true && s3 == true && s4 == false ||
            s1 == false && s2 == true && s3 == true && s4 == false || 
            s1 == false && s2 == false && s3 == false && s4 == false 
            ){//1 0 0 0
      intake.set(intake_Velocity);
      motorBandaA.set(bandaA_Velocity);
      motorBandaB.set(bandaB_Velocity);

      ready = false;
    }
    
    else {
      intake.set(intake_Velocity);
      motorBandaA.set(0);
      motorBandaB.set(0);

      ready = false;
    }

    if (ready) RobotContainer.leds.sendData(4);
    else RobotContainer.leds.sendData(1);

  }
  
  /**
   * Expulsa las power cells hacia el disparador.
   */
  public void ejectBallstoShooter(){
    double ejectSpeed = 1.0;//0.8

    intake.set(ejectSpeed);
    motorBandaA.set(ejectSpeed);
    motorBandaB.set(ejectSpeed);
  }

  public void ejectBallstoShooterAuto(){
    double ejectSpeed = 0.25;

    intake.set(ejectSpeed);
    motorBandaA.set(ejectSpeed);
    motorBandaB.set(ejectSpeed);
  }


  /**
   * Expulsa las power cells hacía fuera del robot.
   */
  public void ejectBallstoOut(){
    intake.set(-1);
      motorBandaA.set(-0.9);
      motorBandaB.set(-0.9);

  }

  /**
   * Extiende el intake.
   */
  public void toExtendIntake(){
    pistonA.set(true);
    intakeDeployed = true;
  }

  /**
   * Guarda el intake.
   */
  public void saveIntake(){
    pistonA.set(false);
    intakeDeployed = false;
  }

  @Override
  public void periodic() {
    s1 = !sensor1.get();
    s2 = !sensor2.get();
    s3 = !sensor3.get();
    s4 = !sensor4.get();

    /* sensors[0] = !sensor1.get();
    sensors[1] = !sensor2.get();
    sensors[2] = !sensor3.get();
    sensors[3] = !sensor4.get(); */

    SmartDashboard.putBoolean("1", s1);
    SmartDashboard.putBoolean("2", s2);
    SmartDashboard.putBoolean("3", s3);
    SmartDashboard.putBoolean("4", s4);
    SmartDashboard.putBoolean("READY?", ready);

    SmartDashboard.putBoolean("intakeDeployed", intakeDeployed); // registra el estado del intake si esta o no extendido

    /* Dashboard.sendBoolean("sens1", s1);
    Dashboard.sendBoolean("sens2", s2);
    Dashboard.sendBoolean("sens3", s3);
    Dashboard.sendBoolean("sens4", s4);
    Dashboard.sendBoolean("ready", ready); */

  }
}
