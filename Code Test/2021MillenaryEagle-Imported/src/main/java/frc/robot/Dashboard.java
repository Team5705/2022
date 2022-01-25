package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * Apartado donde se configura el flujo de datos hacia la Dashboard pór medio de NetworkTables
 */
public class Dashboard {
    private static NetworkTable table = NetworkTableInstance.getDefault().getTable("data");

//***************************************************************/
//              TABLA DE DATOS ENVIADOS A LA DASHBOARD          //
//***************************************************************/
//
//  sens1       //Sensor 1 Powercells, es el primer sensor que detecta cuando entra la primera powercell
//  sens2       //Sensor 2 Powercells
//  sens3       //Sensor 3 Powercells
//  sens4       //Sensor 4 Powercells
//  ready       //Booleano que indica cuando el robot está listo para disparar
//  gyro        //Giroscopio del navX
//  rateL       //Velocidad en m/s del diferencial izquierdo
//  rateR       //Velocidad en m/s del diferencial derecho
//  navReady    //Booleano que indica cuando el navX esté calibrado
//  x           //Valor en el eje x conforme al error que lanza la limelight  //NO
//  y           //Valor en el eje y conforme al error que lanza la limelight  //NO
//  area        //area del objetivo en pixeles                                //NO
//  gamedata    //valor dado por la FMS del dato en juego (tercera etapa)
//  B           //Booleano que indica el color del gamedata
//  G           //Booleano que indica el color del gamedata
//  R           //Booleano que indica el color del gamedata
//  Y           //Booleano que indica el color del gamedata

    public static void sendBoolean(String key, boolean value){
        table.getEntry(key).setBoolean(value);
    }

    public static void sendDouble(String key, double value){
        table.getEntry(key).setDouble(value);
    }

    public static void sendString(String key, String value){
        table.getEntry(key).setString(value);
    }

}






    /* public static void sens1(boolean value) { //Sensor 1 Powercells, es el primer sensor que detecta cuando entra la primera powercell
        table.getEntry("s1").setBoolean(value);
    }

    public static void sens2(boolean value){//Sensor 2 Powercells
        table.getEntry("s2").setBoolean(value);
    }

    public static void sens3(boolean value){//Sensor 3 Powercells
        table.getEntry("s3").setBoolean(value);
    }

    public static void sens4(boolean value){//Sensor 4 Powercells
        table.getEntry("s4").setBoolean(value);
    }

    public static void ready(boolean value){//Booleano que indica cuando el robot está listo para disparar
        table.getEntry("ready").setBoolean(value);
    }

    public static void gyro(double value){//Giroscopio del navX
        table.getEntry("gyro").setDouble(value);
    }

    public static void rateL(double value){//Velocidad en m/s del diferencial izquierdo
        table.getEntry("rateL").setDouble(value);
    }

    public static void rateR(double value){//Velocidad en m/s del diferencial derecho
        table.getEntry("rateR").setDouble(value);
    }

    public static void navReady(boolean value){//Booleano que indica cuando el navX esté calibrado
        table.getEntry("navReady").setBoolean(value);
    }

    public static void x(double value){//Valor en el eje x conforme al error que lanza la limelight
        table.getEntry("x").setDouble(value);
    }

    public static void y(double value){//Valor en el eje y conforme al error que lanza la limelight
        table.getEntry("y").setDouble(value);
    }

    public static void area(double value){//area del objetivo en pixeles
        table.getEntry("area").setDouble(value);
    }

    public static void gamedata(String value){//valor dado por la FMS del dato en juego (tercera etapa)
        table.getEntry("gamedata").setString(value);
    }

    public static void B(boolean value){//Booleano que indica el color del gamedata
        table.getEntry("B").setBoolean(value);
    }

    public static void G(boolean value){//Booleano que indica el color del gamedata
        table.getEntry("G").setBoolean(value);
    }

    public static void R(boolean value){//Booleano que indica el color del gamedata
        table.getEntry("R").setBoolean(value);
    }

    public static void Y(boolean value){//Booleano que indica el color del gamedata
        table.getEntry("Y").setBoolean(value);
    } 
} */
