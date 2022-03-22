/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.PID;
import frc.robot.subsystems.Powertrain;
import frc.robot.subsystems.Vision;

public class Tracking extends CommandBase {
  private final Vision vision;
  private final Powertrain powertrain;
  private static PID pidX = new PID(0.02, 0, 8, 0, 0.27, false);
  private boolean finished = false;
  private double distance = 0;
  private final double range = 0.1;

  private final double minimumDistance = 2.70; //meters
  private final double maximumDistance = 4.60; //meterss

  private final double upperHUBVisionTargetDiameter = Units.inchesToMeters(53.38) / 2; //.25 tolerance

  /**
   * Ejecuta el seguimiento al centro del objetivo sin un fin establecido.
   * 
   * @param powertrain Subsistema motriz
   * @param vision Subsistema de vision
   */
  public Tracking(Powertrain powertrain, Vision vision) {
    this.powertrain = powertrain;
    this.vision = vision;
    addRequirements(powertrain);
  }

  /**
   * Ejecuta el seguimiento cerca del centro del objetivo. Este modo es para que el comando funcione solo hasta que esté dentro
   * del rango establecido para luego pasar al siguiente comando.
   * 
   * @param powertrain Subsistema motriz
   * @param vision Subsistema de vision
   * @param finished Indica si queremos que el comando termine o no, de ser falso nunca terminará hasta que sea interrumpido,
   *                 de ser verdadero terminará cuando el robot esté alineado con el objetivo. Dejar en verdadero.
   */
  public Tracking(Powertrain powertrain, Vision vision, boolean finished) {
    this.powertrain = powertrain;
    this.vision = vision;
    this.finished = finished;

    addRequirements(powertrain);
  }

  @Override
  public void initialize() {
    vision.ledsOn();

   }

  @Override
  public void execute() {
    //Ajustar la distancia para el tiro en el ultimo valor           | Aqui
    distance = vision.getDistance() + upperHUBVisionTargetDiameter + 0;
    double xS;

    if(distance < minimumDistance){
      //Distancia muy cercana!
      xS = 0.5;
    }
    else if(distance > maximumDistance){
      //Distancia muy lejana!
      xS = -0.5;
    }
    else{
      //Distancia no filtrada!
      xS = 0;
    }

    pidX.runPIDErr(vision.getX());

    if (vision.availableTarget()) {

      double turn = pidX.valuePID();

      powertrain.arcadeDrive(xS, turn);

    } 
    else{
      powertrain.arcadeDrive(0, 0.5);

    }
  }

  @Override
  public void end(boolean interrupted) {

  }

  @Override
  public boolean isFinished() {

    if (finished){

      return (Math.abs(vision.getX()) <= range && vision.getX() != 0) && (distance > minimumDistance && distance < maximumDistance);
    }
    else {
      return false;
    }
  }
}
