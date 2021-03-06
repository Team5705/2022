// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Conveyor;

public class GetBalls extends CommandBase {
  /** Creates a new GetCommand. */
  private final Conveyor conveyor;

  public GetBalls(Conveyor conveyor) {
    this.conveyor = conveyor;
    addRequirements(conveyor);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //intake.extendIntake();
    //Timer.delay(0.250); // 250 ms
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //intake.forward();
    conveyor.getCargoWithSensor();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //intake.contractIntake();
    //intake.neutral();
    conveyor.neutral();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;// conveyor.getCount() == 2;
  }
}
