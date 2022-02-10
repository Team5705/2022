// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.commands.DRIVE;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.intake_test;
import frc.robot.subsystems.pruebas;
import frc.robot.subsystems.pt;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem); 
  private final pt chasis = new pt(); 
  private final pruebas prueba = new pruebas();
  private final DRIVE comandito = new DRIVE(chasis);
  public static XboxController drive = new XboxController(0);
  private final intake_test intake = new intake_test();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    chasis.setDefaultCommand(comandito);
  }

   

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(drive, 5).whileHeld(new RunCommand(() -> prueba.moverIntake(0.7), prueba));
    new JoystickButton(drive, 5).whenReleased((new RunCommand(() -> prueba.moverIntake(0.0), prueba)));

    new JoystickButton(drive, 1).whileHeld(new RunCommand(() -> prueba.moverTorreta(drive.getRawAxis(3)), prueba));
    new JoystickButton(drive, 1).whenReleased((new RunCommand(() -> prueba.moverTorreta(0.0), prueba)));

    new JoystickButton(drive, 2).whenPressed(new RunCommand(() -> intake.intakeDeployed(), intake));
    new JoystickButton(drive, 3).whenPressed(new RunCommand(() -> intake.savedIntake(), intake));

   // new JoystickButton(drive, 2).whenPressed(new Turn(chasis, 110));
   // new JoystickButton(drive, 1).whenPressed(new Turn(chasis, 30));
    
   
    }
  

  /**                 
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
}
