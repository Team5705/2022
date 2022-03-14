// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.RoutinesCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Tracking;
import frc.robot.commands.ShooterCommands.AdjustShot;
import frc.robot.commands.ShooterCommands.AdjustShotLoop;
import frc.robot.subsystems.Powertrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootwithHood extends SequentialCommandGroup {
  /** Creates a new ShootwithHood. */
  public ShootwithHood(Powertrain powertrain, Vision vision, Shooter shooter) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new Tracking(powertrain, vision, true), new AdjustShot(shooter, vision), 
      new ParallelCommandGroup(new Tracking(powertrain, vision),
                               new AdjustShotLoop(shooter, vision),
                               new PrintCommand("Lanzar cargos")) //Poner como el MillenaryEagle de que el conveyor mande un booleano cuando todos los sensores sean falsos, así para frenar al shooter en un diferente constructor
                                                                  //En el shooter, de ser verdadero el cosntructor donde se detiene, en la funcion de end() poner un delay con el tiempo necesario para que la última pelota salga del mecanismo y no se detengga antes de que salga
    );
  }
}
