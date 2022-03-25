// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.OIConstant;
import frc.robot.Constants.pathWeaver;
import frc.robot.commands.Drive;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Powertrain;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Vision;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  //Subsystems
  private final Powertrain powertrain = new Powertrain();
  private final Vision vision = new Vision();
  private final Shooter shooter = new Shooter();
  private final Conveyor conveyor = new Conveyor();
  private final Intake intake = new Intake();
  //public static Leds leds = new Leds();

  //Commands
  private final Drive drive = new Drive(powertrain);

  //Trajectory [] trajectories = new Trajectory[] {trajectory1};

  String trajectoryJSON1 = "paths/output/a.wpilib.json";
  Trajectory trajectory1 = new Trajectory();

  String trajectoryJSON2 = "paths/output/d2.wpilib.json";
  Trajectory trajectory2 = new Trajectory();

  public static XboxController driverController = new XboxController(OIConstant.controllerPort);

  SendableChooser<String> autonomous = new SendableChooser<String>();

  ArrayList<String> trajectoryPaths = new ArrayList<String>();

  
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    vision.ledsOn();
    readPaths();
    
    // Configure the button bindings
    configureButtonBindings();
    
    powertrain.setDefaultCommand(drive);

    autonomous.addOption("Mid", "mid");
    autonomous.addOption("OnlyBack", "onlyback");
    autonomous.addOption("oneball", "oneball");
    //autonomous.addOption("Emergency", "emergency");
    //autonomous.addOption("Test", "test");
    autonomous.setDefaultOption("NULL", null);
    SmartDashboard.putData("autoMode?", autonomous);
    
  }

  public void readPaths(){
    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON1);
      trajectory1 = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
      System.out.println(trajectoryJSON1 + " successfully read :D");
      
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open path: " + trajectoryJSON1, ex.getStackTrace());
      
    }

    try {
      Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON2);
      trajectory2 = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
      System.out.println(trajectoryJSON2 + " successfully read :D");
      
    } catch (IOException ex) {
      DriverStation.reportError("Unable to open path: " + trajectoryJSON2, ex.getStackTrace());
      
    }
  }
  
  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    //new JoystickButton(driverController, 1).whileHeld(new RunCommand(() -> shooter.shoot(driverController.getRawAxis(3)), shooter).
    //andThen(new InstantCommand(shooter::neutral, shooter)));

    //new JoystickButton(driverController, 1).whileHeld(new Tracking(powertrain, vision));
    //new JoystickButton(driverController, 2).whileHeld(new AdjustShot(shooter, vision));
    
    new JoystickButton(driverController, 1)
      .whileHeld(new RunCommand(() -> conveyor.move( driverController.getRawAxis(4)), conveyor))
      .whenReleased(new RunCommand(() -> conveyor.neutral()));

    new JoystickButton(driverController, 2)
      .whileHeld(new RunCommand(() -> intake.move( driverController.getRawAxis(4) ), intake))
      .whenReleased(new RunCommand(() -> intake.neutral()));

    new JoystickButton(driverController, 3)
      .whileHeld(new RunCommand(() -> shooter.shootMove( driverController.getRawAxis(4) ), shooter))
      .whenReleased(new RunCommand(() -> shooter.neutral()));

    new JoystickButton(driverController, 4)
      .whileHeld(new RunCommand(() -> shooter.moveHood( driverController.getRawAxis(4) ), shooter))
      .whenReleased(new RunCommand(() -> shooter.neutralHood()));

    

    new JoystickButton(driverController, 7).whenPressed(new InstantCommand(powertrain::neutralModeBrake, powertrain)); //Chasis Brake mode
    new JoystickButton(driverController, 8).whenPressed(new InstantCommand(powertrain::neutralModeCoast, powertrain)); //Chasis Coast mode

    //POV
  }
  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    //return new SequentialCommandGroup(runPath(trajectory1), runPath(trajectory2)
    //);

    if(autonomous.getSelected() == "mid"){
      return null;
    }
    else if(autonomous.getSelected() == "onlyback"){
      return null;
    }
    else if(autonomous.getSelected() == "oneball"){
      return null;
    }
    else
      return null;
  }




  public Command runPath(Trajectory myTrajectory){
    RamseteCommand ramseteCommand =
    new RamseteCommand(
        myTrajectory,
        powertrain::getPose,
        new RamseteController(pathWeaver.kRamseteB, pathWeaver.kRamseteZeta),
        new SimpleMotorFeedforward(
          pathWeaver.ksVolts,
          pathWeaver.kvVoltSecondsPerMeter,
          pathWeaver.kaVoltSecondsSquaredPerMeter),
          pathWeaver.kDriveKinematics,
          powertrain::getWheelSpeeds,
          new PIDController(pathWeaver.kPDriveVel, 0, 0),
          new PIDController(pathWeaver.kPDriveVel, 0, 0),
          // RamseteCommand passes volts to the callback
          powertrain::setVolts,
      powertrain);

    // Reset odometry to the starting pose of the trajectory.
    /*powertrain.resetOdometry(myTrajectory.getInitialPose());*/
    
    // Run path following command, then stop at the end.
    return new SequentialCommandGroup(new InstantCommand(() -> powertrain.resetOdometry(myTrajectory.getInitialPose()), powertrain), 
                                      ramseteCommand.andThen(() -> powertrain.setVolts(0, 0)));
    //return ramseteCommand;
  }
  
}
        