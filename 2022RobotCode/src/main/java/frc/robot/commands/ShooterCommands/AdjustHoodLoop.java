// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCommands;

import javax.lang.model.util.ElementScanner6;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.PID;
import frc.robot.ParableShot;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;

public class AdjustHoodLoop extends CommandBase {
  private final Hood hood;
  private final Vision vision;

  private static ParableShot parableShot = new ParableShot();
  private static PID pidHood = new PID(0.005, 0, 0, true); //AJUSTAR!

  private double angle;
  
  /** Creates a new ShotWithAngulator. */
  public AdjustHoodLoop(Hood hood, Vision  vision) {
    this.hood = hood;
    this.vision = vision;
    addRequirements(hood);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //Leds limelightON()
    vision.ledsOn();

    //Comprobar que haya objetivo para la limelight sino girar el robot. -> Mejor ejecutar un comando a parte para asegurar siempre el valor.

    //Comprobar las distancias mínimas y máximas, de ser verdadero avanzar o retroceder -> Igual que arriba, corroborar antes. Así al ejecutar
    //este comando tendremos valores verdaderos.
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    parableShot.setDistance(vision.getDistance());//Limelight distance report
    parableShot.executeAlgorithm();

    if (parableShot.getAngle() == 0)
      end(true);
    else if (parableShot.getVelocity() == 0)
      end(true);
    else{
      angle = parableShot.getAngle();
    }

    pidHood.setDesiredValue(angle);

    //set degrees on the servos for adjust
    //set value in the shooter motors for adjust
    pidHood.runPID(hood.getHoodAngle());

    double speed = pidHood.valuePID();

    hood.moveHood(speed);
    if (speed > 0.8)
      speed = 0.8;
    else if (speed < -0.8)
      speed = -0.8;
    //shooter.shootMove(speedShooter);

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
    return false;
  }
}
