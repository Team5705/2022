/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import frc.robot.Constants.OIConstant;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls). Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Powertrain powertrain = new Powertrain();
  private final Vision vision = new Vision();
  private final IntakeBalls intake = new IntakeBalls();
  private final Shooter shooter = new Shooter();
  private final Climber climber = new Climber();
  public static Leds leds = new Leds();

  private final Drive drive = new Drive(powertrain);
  private final Climberr climberr = new Climberr(climber);

  public static XboxController driverController = new XboxController(OIConstant.controllerPort);

  SendableChooser<String> autonomous = new SendableChooser<String>();
  
  ArrayList<String> trajectoryPaths = new ArrayList<String>();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    powertrain.setDefaultCommand(drive);
    climber.setDefaultCommand(climberr);

    autonomous.addOption("Trench", "trench");
    autonomous.addOption("Mid", "mid");
    autonomous.addOption("Simple", "simple");
    autonomous.addOption("Emergency", "emergency");
    autonomous.addOption("Test", "test");
    SmartDashboard.putData("autoMode", autonomous);



    trajectoryPaths.add(0, "");
    trajectoryPaths.add(1, "");

    trajectoryPaths.add(2, "");
    trajectoryPaths.add(3, "");
    trajectoryPaths.add(4, "");
    trajectoryPaths.add(5, "");

    trajectoryPaths.add(6, "paths/output/test.wpilib.json");
  }

  /**
   * 
   * @param trajectoryPaths Número del path en la lista declarada
   * @return  Comando ramsete para su utilización en el seguimiento del path
   */
  public Command ramseteC(String trajectoryPaths) {
        String path = trajectoryPaths;
        Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(path);
        try {
            Trajectory trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);

            RamseteCommand command = new RamseteCommand(trajectory, powertrain::getPosition,
                    new RamseteController(2.0, .7), powertrain.getFeedFoward(), powertrain.getDifferentialDriveKinematics(),
                    powertrain::getWheelSpeeds, powertrain.getLeftPIDController(), powertrain.getRightPIDController(),
                    powertrain::setVolts, powertrain);
                    powertrain.zeroHeading();

                    System.out.println("Path successfully read");
                    new PrintCommand("Path successfully read");
                    
            return command;
    
        } catch (IOException e) {
            System.out.println("Unable to open trajectory: " + path);
            new PrintCommand("Unable to open trajectory: " + path);

            return null;
        }
  }
  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by instantiating a {@link GenericHID} or one of its subclasses
   * ({@link edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then
   * passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    //Buttons
    new JoystickButton(driverController, 1).whileHeld(new Shootv2(shooter, false));
    new JoystickButton(driverController, 2).whenPressed(new TurnPIDB(powertrain, 50));
    new JoystickButton(driverController, 3).whenPressed(new TurnPIDB(powertrain, 110));
    //new JoystickButton(driverController, 2).whenPressed(new InstantCommand(intake::toExtendIntake, intake));
    //new JoystickButton(driverController, 3).whenPressed(new InstantCommand(intake::saveIntake, intake));
    //new JoystickButton(driverController, 4).whenPressed(new Shoot(shooter, intake, powertrain, vision));
    new JoystickButton(driverController, 4).whileHeld(new Tracking(powertrain, vision));
    new JoystickButton(driverController, 5).whileHeld(new TakeAll(intake));
    new JoystickButton(driverController, 6).toggleWhenPressed(new TakeWithSensor(intake));
    //new JoystickButton(driverController, 6).toggleWhenPressed(new TakeWithSensor(intake));

    //new JoystickButton(driverController, 8).whenPressed(new InstantCommand(powertrain::resetGyro)); // Boton Select

    //POV
    new POVButton(driverController, 270).whileHeld(new EjectBalls(intake));
  }

  //-----------------------------------------------------------------------------------------------------------------------------------------

  public Command getAutonomousCommand(){
    if(autonomous.getSelected() == "trench"){ 
      
      return new SequentialCommandGroup(new Shoot(shooter, intake, powertrain, vision).withTimeout(3.6), 
                                        ramseteC(trajectoryPaths.get(0)),
                                        new PrintCommand("3"), 
                                        new PrintCommand("4")
                                        );
    }

    else if(autonomous.getSelected() == "mid"){
      
      return new SequentialCommandGroup(new PrintCommand("1"), new PrintCommand("2"), new PrintCommand("3"), new PrintCommand("4"));
    }
    
    else if(autonomous.getSelected() == "test"){
      //return ramseteC(trajectoryPaths.get(6)); //6 is test
      return new SequentialCommandGroup(new ParallelCommandGroup(new Distance(powertrain, 3), new TakeWithSensor(intake).withTimeout(2)));
                                     /*   new WaitCommand(0.25),
                                        new TurnPIDB(powertrain, 90),
                                        new WaitCommand(0.25),
                                        new ParallelCommandGroup(new Distance(powertrain, 3), new TakeWithSensor(intake).withTimeout(2)));*/
      
    }
    else
    return null;
  }
}
