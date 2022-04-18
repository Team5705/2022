// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Shooter;

public class ConveyorReverse extends CommandBase {
  private final Conveyor conveyor;
  private final Shooter shooter;

  public ConveyorReverse(Conveyor conveyor, Shooter shooter) {
    this.conveyor = conveyor;
    this.shooter = shooter;
    addRequirements(conveyor, shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    conveyor.reverse();
    shooter.shootMove(-0.2);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    conveyor.neutral();
    shooter.neutral();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
