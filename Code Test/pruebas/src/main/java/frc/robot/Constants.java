// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SPI.Port;




/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
public static final class DriveConstant {
    //Motors
    public static final int[] portsMotors = new int[] { 0,
                                                        1,
                                                        2,
                                                        3};
    //Sensors                                                    
    public static final Port Gyro = Port.kOnboardCS0;                                                    
}

public static final class OIconstant {
    public static final int controllerPort = 0;
} 

public static final class Intake {
    //Constants
    public static final double intakeVelocity = 0.9,
                               conveyorVelocity = 0.9;
    //Motors
    public static final int m1 = 4; //intake 
    public static final int m2 = 5; //conveyorRollers1
    //Pneumatic
    public static final int solenoidPort = 0;
    //Sensors
    public static final int[] sensors = new int[] {0,  //s1
                                                   1,  //s2
                                                   2}; //s3

    public static final I2C.Port colorSensorPort = I2C.Port.kOnboard;

    
}

public static final class shoot {
    //Motors
    public static final int mShooter = 6;
    public static final int mShooter2 = 7;
}




}
