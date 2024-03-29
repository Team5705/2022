// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.PID;
import frc.robot.subsystems.Shooter;

public class AdjustShotVelocity extends CommandBase {
  private final Shooter shooter;

  private static PID pidShoot;

  private double velocity;

  private boolean finished = false;
  
  /**
   * 
   * @param shooter
   * @param velocity Velocidad del proyectil en m/s
   */
  public AdjustShotVelocity(Shooter shooter, double velocity) {
    this.shooter = shooter;
    this.velocity = velocity;
    addRequirements(shooter);

    pidShoot = new PID(0, 0, 0, false); //AJUSTAR!
  }

  /**
   * 
   * @param shooter
   * @param velocity Velocidad del proyectil en m/s
   * @param finished Marcar para terminar al ajustar
   */
  public AdjustShotVelocity(Shooter shooter, double velocity, boolean finished) {
    this.shooter = shooter;
    this.velocity = velocity;
    this.finished = finished;
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pidShoot.setDesiredValue(velocity);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pidShoot.runPID(shooter.getShootVelocityMeterPerSeconds());

    double speedShooter = pidShoot.valuePID();

    shooter.shootMove(speedShooter);
;
    SmartDashboard.putNumber("PIDShooter", speedShooter);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.shootMove(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (finished)
      return (shooter.getShootVelocityMeterPerSeconds() >= velocity);
    else
      return false;
  }
}
