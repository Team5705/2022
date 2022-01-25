/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.IntakeBalls;
import frc.robot.subsystems.Powertrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class Shoot extends ParallelCommandGroup {

  /**
   * Comando paralelo para accionar el shooter mientras espera a que la limelight encuentre el objetivo.
   * Es un comando mixto donde se realizan acciones paralelas y a su vez acciones
   * secuenciales.
   * No hace faltar mantenerlo presionado, solo con hacerlo una vez el comando acabará si es que encuentra un objetivo.
   * Para interrumpirlo basta con presionar el botón A.
   */
  public Shoot(Shooter shooter, IntakeBalls intake, Powertrain powertrain, Vision vision) {

  super(new Shootv2(shooter, true),
        new SequentialCommandGroup(new Tracking(powertrain, vision, true), new ParallelCommandGroup(new Tracking(powertrain, vision).withTimeout(3), 
                                                                                                    new TakeAll(intake).withTimeout(2)))
         );
  }
}
