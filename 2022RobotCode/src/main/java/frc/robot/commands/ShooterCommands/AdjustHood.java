// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.PID;
import frc.robot.ParableShot;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Vision;

public class AdjustHood extends CommandBase {
  private final Hood hood;
  private final Vision vision;

  private static ParableShot parableShot;
  private static PID pidHood, pidShoot;

  private double angle, 
                 velocity;
  
  /** Creates a new ShotWithAngulator. */
  public AdjustHood(Hood hood, Vision vision) {
    this.hood = hood;
    this.vision = vision;
    addRequirements(hood);

    parableShot = new ParableShot();
    
    pidHood = new PID(0, 0, 0, false); //AJUSTAR!
    pidShoot = new PID(0, 0, 0, false); //AJUSTAR!
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //Leds limelightON()

    //Comprobar que haya objetivo para la limelight sino girar el robot. -> Mejor ejecutar un comando a parte para asegurar siempre el valor.

    //Comprobar las distancias mínimas y máximas, de ser verdadero avanzar o retroceder -> Igual que arriba, corroborar antes. Así al ejecutar
    //este comando tendremos valores verdaderos.

    parableShot.setDistance(vision.getDistance() + 0);//Limelight distance report
    parableShot.executeAlgorithm();

    if (parableShot.getAngle() == 0)
      end(true);
    else if (parableShot.getVelocity() == 0)
      end(true);
    else{
      angle = parableShot.getAngle();
      velocity = parableShot.getVelocity();
    }

    pidHood.setDesiredValue(angle);
    pidShoot.setDesiredValue(velocity);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //set degrees on the servos for adjust
    //set value in the shooter motors for adjust
    //pidHood.runPID(shooter.getHoodAngle());
    //pidShoot.runPID(shooter.getShootVelocityMeterPerSeconds());

    double speedHood = pidHood.valuePID();
    double speedShooter = pidShoot.valuePID();

    hood.moveHood(speedHood);

    SmartDashboard.putNumber("PIDHood", speedHood);
    SmartDashboard.putNumber("PIDShooter", speedShooter);

    SmartDashboard.putNumber("Angle", angle);
    SmartDashboard.putNumber("Velocity", velocity);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    hood.moveHood(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    //Grados de la capucha iguales al calculado
    return (hood.getPosition() >= angle);
  }
}
