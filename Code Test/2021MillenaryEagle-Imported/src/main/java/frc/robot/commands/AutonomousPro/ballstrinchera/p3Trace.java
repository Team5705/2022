/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.AutonomousPro.ballstrinchera;

import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Distance;
import frc.robot.commands.Shoot;
import frc.robot.commands.TakeWithSensor;
import frc.robot.commands.TurnPID;
import frc.robot.subsystems.IntakeBalls;
import frc.robot.subsystems.Powertrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class p3Trace extends SequentialCommandGroup {
  public static final double timeDelay = 0.5; //Tiempo de espera en segundos

  public p3Trace(Powertrain powertrain, IntakeBalls intake, Shooter shooter, Vision vision) {
    super(new Shoot(shooter, intake, powertrain, vision).withTimeout(3.5),
          new TurnPID(powertrain, 120),                           new WaitCommand(timeDelay), //
          new Distance(powertrain, 2),                            new WaitCommand(timeDelay), //
          new TurnPID(powertrain, 60),                            new WaitCommand(timeDelay), //
          new ParallelDeadlineGroup(new Distance(powertrain, 1.5, 0, 0, 0, 0.30), 
                                    new TakeWithSensor(intake)),  new WaitCommand(timeDelay),
          new TurnPID(powertrain, 180),                           new WaitCommand(timeDelay),
          new Distance(powertrain, 3),                            new WaitCommand(timeDelay), 
          new TurnPID(powertrain, -30),                           new WaitCommand(timeDelay), //
          new Shoot(shooter, intake, powertrain, vision).withTimeout(3.5)
          );
  }
}
