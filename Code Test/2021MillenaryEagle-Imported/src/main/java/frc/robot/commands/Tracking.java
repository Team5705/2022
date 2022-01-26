/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.PID;
import frc.robot.subsystems.Powertrain;
import frc.robot.subsystems.Vision;

public class Tracking extends CommandBase {
  private final Vision vision;
  private final Powertrain powertrain;
  private static PID pidX = new PID(0.04, 0, 8, 0, 0.27, false); //turn 0.00001dl
  private static PID pidY = new PID(0.06, 0, 8, 0, 0.3, true);
  private boolean finished = false;
  private double x, y;
  private double range = 0.1;

  /**
   * Ejecuta el seguimiento al centro del objetivo sin un fin establecido.
   * 
   * @param powertrain Subsistema motriz
   * @param vision Subsistema de vision
   */
  public Tracking(Powertrain powertrain, Vision vision) {
    this.powertrain = powertrain;
    this.vision = vision;
    addRequirements(this.powertrain, this.vision);
  }

  /**
   * Ejecuta el seguimiento cerca del centro del objetivo. Este modo es para que el comando funcione solo hasta que esté dentro
   * del rango establecido para luego pasar al siguiente comando.
   * 
   * @param powertrain Subsistema motriz
   * @param vision subsistema de vision
   * @param finished Indica si queremos que el comando termine o no, de ser falso nunca terminará hasta que sea interrumpido,
   *                 de ser verdadero terminará cuando el robot esté alineado con el objetivo. Dejar en verdadero.
   */
  public Tracking(Powertrain powertrain, Vision vision, boolean finished) {
    this.powertrain = powertrain;
    this.vision = vision;
    this.finished = finished;

    addRequirements(this.powertrain, this.vision);
  }

  @Override
  public void initialize() {
    vision.ledsOn();
   }

  @Override
  public void execute() {
    x = vision.getX();
    y = vision.getY();

    pidX.runPIDErr(x);
    pidY.runPIDErr(y);

    if (vision.availableTarget()) {

      double xS = pidY.valuePID();
      double turn = pidX.valuePID();

      powertrain.arcadeDrive(xS, turn);

    } else{
          powertrain.arcadeDrive(0, 0.5);

    }
  }

  @Override
  public void end(boolean interrupted) {
    if (finished != true){
      vision.blinkLeds();
      Timer.delay(1);
      vision.ledsOff();
    }

    new PrintCommand("FinishedVision");
  }

  @Override
  public boolean isFinished() {

    if (finished){

      return (Math.abs(x) <= range && x != 0) && (Math.abs(y) <= range && y != 0);
    }
    else {
      return false;
    }
  }
}
