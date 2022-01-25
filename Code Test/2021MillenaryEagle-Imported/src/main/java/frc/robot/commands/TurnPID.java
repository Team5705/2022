/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.PID;
import frc.robot.subsystems.Powertrain;

public class TurnPID extends CommandBase {
  private final Powertrain powertrain;

  private static PID pidTurn;

  private double gyro;
  private double angle;
  private boolean direction; // true = clockwise | false = counter-clockwise

  double kP = 0.009, kI = 0.00000, kD = 0, bias = 0; //.008

  /**
   * unu
   * 
   * @param powertrain     Subsistema motriz
   * @param angle          Angulo deseado teniendo en cuenta que el angulo se
   *                       resetea y un giro inverso es necesario el angulo
   *                       negativo
   */
  public TurnPID(Powertrain powertrain, int angle, double kP, double kI, double kD) {
    this.powertrain = powertrain;
    this.angle = angle;
    this.kP = kP;
    this.kI = kI;
    this.kD = kD;
    addRequirements(this.powertrain);
  }

  /**
   * unu
   * 
   * En este apartado controlamos el giro del robot hacia el ángulo deseado. Es importante saber
   * que el giroscopio no se reinicia, por ende hay que tener en cuenta en qué sentido se encuentra el 
   * robot actualmente y a que grado queremos llegar conforme al grado 0 que el robot inicia mirando al 
   * objetivo de las powercells.
   * 
   * @param powertrain Subsistema motriz
   * @param angle Angulo deseado entre los 360 grados del robot.
   */
  public TurnPID(Powertrain powertrain, int angle) {
    this.powertrain = powertrain;
    this.angle = angle;
    

    addRequirements(this.powertrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    gyro = powertrain.angleNormalized();

    double difference = Math.abs(gyro - angle);

    /**
     * Qué pasa aquí? Sencillo, si la direncia es mayor a 180° de donde nos encontramos actualmente hasta donde queremos llegar 
     * quiere decir que si giramos en sentido horario vamos a recorres más que si giramos en sentido antihorario. Con esto al saber
     * si la diferencia es mayor a 180° podemos decidir a dónde girar y solo definir el PID.
     */

    if (difference > 180){
      direction = true; //Clockwise | Sentido horario

      pidTurn = new PID(kP, kI, kD, angle, bias, false);
    }

    else{
      direction = false; //Counter-clockwise | Sentido antihorario 

      pidTurn = new PID(kP, kI, kD, angle - 360, bias, true);
    }

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    if(direction){
      gyro = powertrain.angleNormalized(); //Devuelve el valor en un rango de 0 a 359
      
    }
    else {
      gyro = -powertrain.angleNormalized(); //Negativo

    }
    
    pidTurn.runPID(gyro);

    double turn = pidTurn.valuePID();

    powertrain.arcadeDrive(0, turn);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (powertrain.angleNormalized() >= (angle - 1) && powertrain.angleNormalized() < (angle + 1));
  }
}
