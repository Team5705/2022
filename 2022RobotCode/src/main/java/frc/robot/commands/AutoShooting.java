// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Powertrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoShooting extends SequentialCommandGroup {
  /** Creates a new AutoShooting. */
  public AutoShooting(Powertrain powertrain, Vision vision, Shooter shooter, Conveyor conveyor) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      /* new ParallelCommandGroup(
          new ConveyorReverse(conveyor, shooter).withTimeout(1.5),
          new SimpleTracking(powertrain, vision, true)
        ),
        new ParallelCommandGroup(
          new ShootON(shooter).deadlineWith(
            new SequentialCommandGroup(
              new WaitCommand(1.0), 
              new Conveyor_input(conveyor).withTimeout(1.2)
            )
          )
        ) */
        new ConveyorReverse(conveyor, shooter).withTimeout(1.0),
        new ParallelDeadlineGroup(
            new SequentialCommandGroup(
              new SimpleTracking(powertrain, vision, true),//.withTimeout(2),
              new WaitCommand(1.0),
              new Conveyor_input(conveyor).withTimeout(1.5)
            ), 
            new ShootON(shooter)
          )
    );
  }
}
