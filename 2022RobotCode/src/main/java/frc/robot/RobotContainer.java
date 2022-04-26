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
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.kOI;
import frc.robot.Constants.pathWeaver;
import frc.robot.commands.AutoShooting;
import frc.robot.commands.ConveyorReverse;
import frc.robot.commands.Conveyor_input;
import frc.robot.commands.Drive;
import frc.robot.commands.GetBalls;
import frc.robot.commands.IntakeToggle;
import frc.robot.commands.ShootON;
import frc.robot.commands.SimpleTracking;
import frc.robot.commands.SimpleTrackingOnlyX;
import frc.robot.commands.ShooterCommands.AdjustHoodLoop;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.ControlEnergySystem;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Hood;
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
  private final Shooter shooter = new Shooter();
  private final Conveyor conveyor = new Conveyor();
  private final Intake intake = new Intake();
  public final Climber climber = new Climber();
  private final Hood hood = new Hood();

  public final Vision vision = new Vision();
  public final ControlEnergySystem controlEnergySystem = new ControlEnergySystem();
  //public static Leds leds = new Leds();

  //Commands
  private final Drive drive = new Drive(powertrain);
  private final GetBalls getBalls = new GetBalls(conveyor);

  public static XboxController driverController = new XboxController(kOI.controllerPort);
  public static XboxController secondController = new XboxController(kOI.controllerPort2);

  //Trajectory [] trajectories = new Trajectory[] {trajectory1};

  String trajectoryJSON1 = "paths/output/oi.wpilib.json";
  Trajectory trajectory1 = new Trajectory();

  String trajectoryJSON2 = "paths/output/io.wpilib.json";
  Trajectory trajectory2 = new Trajectory();


  SendableChooser<String> autonomous = new SendableChooser<String>();

  ArrayList<String> trajectoryPaths = new ArrayList<String>();

  
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    vision.ledsOn();
    readPaths();
    
    // Configure the button bindings
    configureButtonBindings();
    
    powertrain.setDefaultCommand(drive);
    conveyor.setDefaultCommand(getBalls);

    autonomous.addOption("mid", "mid");
    autonomous.addOption("onlyBack", "onlyback");
    autonomous.addOption("oneball", "oneball");
    //autonomous.addOption("Emergency", "emergency");
    //autonomous.addOption("Test", "test");
    SmartDashboard.putData("autoMode", autonomous);
    
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

    /* DRIVER 1 */
    //BUTTONS
    new JoystickButton(driverController, 1).whileHeld(new SimpleTracking(powertrain, vision));
    new JoystickButton(driverController, 3).whileHeld(new ShootON(shooter));
    new JoystickButton(driverController, 4).whenPressed(new AutoShooting(powertrain, vision, shooter, conveyor));
    
    new JoystickButton(driverController, 5).whileHeld(new Conveyor_input(conveyor));
    new JoystickButton(driverController, 6).toggleWhenPressed(new IntakeToggle(intake));
    
    new JoystickButton(driverController, 7).whenPressed(new InstantCommand(powertrain::neutralModeBrake, powertrain)); //Chasis Brake mode
    new JoystickButton(driverController, 8).whenPressed(new InstantCommand(powertrain::neutralModeCoast, powertrain)); //Chasis Coast mode
    
    new JoystickButton(driverController, 9).whileHeld(new ConveyorReverse(conveyor, shooter));
    //POV
    new POVButton(driverController, 90).whenPressed(new InstantCommand(vision::ledsOff, vision));
    new POVButton(driverController, 270).whenPressed(new InstantCommand(vision::ledsOn, vision));
    
    new POVButton(driverController, 0).whenPressed(new InstantCommand(climber::extend, climber));
    new POVButton(driverController, 180).whenPressed(new InstantCommand(climber::contract, climber));
    
    
    /*DRIVER 2*/
    
    new JoystickButton(secondController, 1).whenPressed(new AdjustHoodLoop(hood, 58.0));
    new JoystickButton(secondController, 2).whenPressed(new AdjustHoodLoop(hood, 70.0));
    new JoystickButton(secondController, 4).whileHeld(new RunCommand(() -> hood.moveHood(driverController.getRawAxis(5)), shooter))
      .whenReleased(new InstantCommand(hood::neutralHood, shooter));
  }
  
  public void updateAutonomous(){
      autonomous.getSelected();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
      return new SequentialCommandGroup(
        /* new ParallelCommandGroup(
          runPath(trajectory1)//, 
          //new IntakeToggle(intake).withTimeout(5)
        ),
        new ParallelCommandGroup(
          //new ConveyorReverse(conveyor, shooter).withTimeout(1.5), 
          new SimpleTracking(powertrain, vision, true).withTimeout(1.2)
        ),
        new ParallelCommandGroup(
          new ShootON(shooter).withTimeout(2),
          new SequentialCommandGroup(
            new WaitCommand(1.5), new Conveyor_input(conveyor).withTimeout(0.5)
          )
        ),
        new SequentialCommandGroup(
          runPath(trajectory2)
        ) */

        new ParallelCommandGroup(
          runPath(trajectory1)//, 
          //new IntakeToggle(intake).withTimeout(2)
        ),
        new AutoShooting(powertrain, vision, shooter, conveyor),
        runPath(trajectory2)
      );
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