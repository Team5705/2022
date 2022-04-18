// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.RoutinesCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.ConveyorReverse;
import frc.robot.commands.Conveyor_input;
import frc.robot.commands.SimpleTracking;
import frc.robot.commands.ShooterCommands.AdjustShotVelocity;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Powertrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SimpleShoot extends SequentialCommandGroup {
  private double recoveryTime = 1.0;
  private double shootVelocity = 14.0;
  /** Creates a new SimpleShoot. */
  public SimpleShoot(Powertrain powertrain, Vision vision, Shooter shooter, Conveyor conveyor) {
    addCommands(new SimpleTracking(powertrain, vision).deadlineWith(new AdjustShotVelocity(shooter, shootVelocity, true)),
                new ParallelCommandGroup(new AdjustShotVelocity(shooter, shootVelocity).withTimeout(2.6),
                                        new SequentialCommandGroup(new WaitCommand(0.5),
                                                                    new ConveyorReverse(conveyor, shooter).withTimeout(0.15),//Calcular
                                                                    new Conveyor_input(conveyor).withTimeout(0.3),//Calcular
                                                                    new WaitCommand(recoveryTime),
                                                                    new Conveyor_input(conveyor).withTimeout(0.6)))
    );
  }
}
