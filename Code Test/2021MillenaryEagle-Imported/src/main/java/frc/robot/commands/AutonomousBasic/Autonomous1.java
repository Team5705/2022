/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.AutonomousBasic;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Distance;
import frc.robot.commands.Shoot;
import frc.robot.commands.TurnPID;
import frc.robot.subsystems.IntakeBalls;
import frc.robot.subsystems.Powertrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class Autonomous1 extends SequentialCommandGroup {
  public static final int timeDelay = 1; // Tiempo de espera en segundos | por seguridad

/**
 * Autónomo básico que trata de dejar las powercells precargadas (3) e ir a la trinchera sin tomar pelotas,
 * solamente estar posicionado.
 */
  public Autonomous1(Powertrain powertrain, Vision vision, Shooter shooter, IntakeBalls intake) {
    super(new Shoot(shooter, intake, powertrain, vision), // Ejecutamos el comando para disparar las powercells
          new TurnPID(powertrain, 100), // Giramos hasta 100°
          new Distance(powertrain, 1), // Avanzamos 1 m
          new TurnPID(powertrain, 180), // Giramos hasta 180°
          new Distance(powertrain, 2) // Avanzamos 2 m
         );
  }
}
