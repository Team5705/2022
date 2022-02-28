/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Shooter;

public class Shootv2 extends CommandBase {
  private final Shooter shooter;
  private boolean finished = false;

  /**
   * Ejecuta el disparador hasta cuando se indique
   * 
   * @param shooter Subsistema del disparador
   */
  public Shootv2(Shooter shooter) {
    this.shooter = shooter;
    addRequirements(this.shooter);
  }

  /**
   * Ejecuta el disparador hasta que el comando que expulsa las powercells acabe.
   * Esto se usa para comandos de automatización.
   * 
   * @param shooter Subsistema del disparador
   * @param finished Valor para activar el paro. Dejar en true.
   */
  public Shootv2(Shooter shooter, boolean finished) {
    this.shooter = shooter;
    this.finished = finished;
    addRequirements(this.shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.leds.sendData2(2);
    TakeAll.finishedShoot = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.go();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.neutral();

    RobotContainer.leds.sendData2(3);
    RobotContainer.leds.sendData(2);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(finished == true)
      return TakeAll.finishedShoot;
    else 
      return false;
  }
}
