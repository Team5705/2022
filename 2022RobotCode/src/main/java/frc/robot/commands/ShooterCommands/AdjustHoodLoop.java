// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.PID;
import frc.robot.subsystems.Hood;

public class AdjustHoodLoop extends CommandBase {
  private final Hood hood;

  private static PID pidHood;

  private double angle;
  
  /** Creates a new ShotWithAngulator. */
  public AdjustHoodLoop(Hood hood, double angle) {
    this.hood = hood;
    this.angle = angle;
    addRequirements(hood);

    pidHood = new PID(0.04, 0, 0, 0.0, -0.06, false);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //Leds limelightON()
    //vision.ledsOn();

    //Comprobar que haya objetivo para la limelight sino girar el robot. -> Mejor ejecutar un comando a parte para asegurar siempre el valor.

    //Comprobar las distancias mínimas y máximas, de ser verdadero avanzar o retroceder -> Igual que arriba, corroborar antes. Así al ejecutar
    //este comando tendremos valores verdaderos.
    pidHood.setDesiredValue(angle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //parableShot.setDistance(vision.getDistance());//Limelight distance report
    //parableShot.executeAlgorithm();

    /* if (parableShot.getAngle() == 0)
      end(true);
    else if (parableShot.getVelocity() == 0)
      end(true);
    else{
      angle = parableShot.getAngle();
    } */


    //set degrees on the servos for adjust
    //set value in the shooter motors for adjust
    pidHood.runPID(hood.getPosition());

    double speed = pidHood.valuePID();

    hood.moveHood(speed);
 /*    if (speed > 0.8)
      speed = 0.8;
    else if (speed < -0.8)
      speed = -0.8; */

    SmartDashboard.putNumber("AngleDesired", angle);
    SmartDashboard.putNumber("PIDHood", speed);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    hood.moveHood(0);
    //shooter.shootMove(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (hood.getPosition() > angle - 0.2) && (hood.getPosition() < angle + 0.2);
  }
}
