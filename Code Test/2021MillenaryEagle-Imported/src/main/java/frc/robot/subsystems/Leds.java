/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Leds extends SubsystemBase {
  
  private I2C nano1 = new I2C(Port.kOnboard, 10);
  private I2C nano2 = new I2C(Port.kOnboard, 0xF4);

  public Leds() {
    
    //connection();

  }

  /********************************************************
   *  Tabla de datos programado para el arduino
   *    0 -> Robot Disable (Rainbow)
   *    1 -> Robot Enable  
   *    2 -> Shoot Enable
   *    3 -> Shoot Disable
   *    4 -> PowerCells Ready
   *    5 -> Rulette Color Red (GameData)
   *    6 -> Rulette Color Yellow (GameData)
   *    7 -> Rulette Color Green (GameData)
   *    8 -> Rulette Color Blue (GameData)
   *    9 -> Badass Mode  
   *    10 -> Begin 
   *    11 -> Led OFF
   *******************************************************/

  public void sendData(int data){

    byte[] datas = {(byte) data};
    
    nano1.writeBulk(datas);

  }

  public void sendData2(int data){

    byte[] datas = {(byte) data};
    
    nano2.writeBulk(datas);

  }

 /* private void connection(){
    if(nano1.addressOnly())
      SmartDashboard.putBoolean("ArduinoConnected?", true);
    else 
    SmartDashboard.putBoolean("ArduinoConnected?", false);
  }*/
  

  @Override
  public void periodic() {
  }
}
