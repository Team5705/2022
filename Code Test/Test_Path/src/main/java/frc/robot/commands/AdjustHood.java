// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.ParableShot;

public class AdjustHood extends CommandBase {
  private static ParableShot parableShot = new ParableShot();
  private double angle, 
                 velocity;
  
  /** Creates a new ShotWithAngulator. */
  public AdjustHood() {
    addRequirements();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //Leds limelightON()
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //Comprobar que haya objetivo para la limelight sino girar el robot. -> Mejor ejecutar un comando a parte para asegurar siempre el valor.

    //Comprobar las distancias mínimas y máximas, de ser verdadero avanzar o retroceder -> Igual que arriba, corroborar antes. Así al ejecutar
    //este comando tendremos valores verdaderos.

    //Timer.delay(0.3);//Tiempo para reconocer el objetivo
    parableShot.setDistance(3.00);//Limelight distance report
    parableShot.executeAlgorithm();

    if (parableShot.getAngle() == 0)
      end(true);
    else if (parableShot.getVelocity() == 0)
      end(true);
    else{
      angle = parableShot.getAngle();
      velocity = parableShot.getVelocity();
    }

    //set degrees on the servos for adjust
    //set value in the shooter motors for adjust
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true; //Grados de la capucha iguales al calculado
  }
}
