// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.motorcontrol.Spark;




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
    public static final int[] portsMotors = new int[] { 4,
                                                        32,
                                                        3,
                                                        23};
                                                        
    public static final Port Gyro = Port.kOnboardCS0;                                                    
}

public static final class OIconstant {
    public static final int controllerPort = 0;
} 

public static final class Intake {
    public static final int m1 = 2; //intake 
    public static final int m2 = 0; //conveyorRollers1
    public static final int m3 = 1; //conveyorRollers1
    public static final int m4 = 1; //conveyorRollers1

    public static final int[] sensors = new int[] {0,
                                                    1,
                                                    2,
                                                    3};

    
}

public static final class shoot {
    public static final int mShooter = 6;
    public static final int mShooter2 = 8;
}




}
