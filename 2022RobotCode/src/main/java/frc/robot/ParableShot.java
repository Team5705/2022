// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** 
 * Clase para calcular parábolas diseñada por el equipo de programación | FRC 5705
 */
public class ParableShot {
    //private final double kSpeedTransfer = 0.2055; // 35%
    private final double g = -9.80665; // m/s^2
    private final double initialHeight = 0.82804; // m
    private final int range = 10;

    private double kInitialVelocity = 7.05; // m/s
    private  double distance; // m
    
    static double [] t = new double[1000]; // Vector con los datos del tiempo
    static double [] t_squared = new double[1000]; // Vector con los datos del tiempo elevados al cuadrado
    static double [] t_plus = new double[1000];
    static double [] x_Data = new double[450];
    static double [] y_Data = new double[450];
    static int [] y_DataToInt = new int[450];
    
    static double vy0; // velocidad inicial en y [m/sg]
    static double vx0; // velocidad inicial en x [m/sg]
    static double t_MAX;
    
    static double slope;
    
    private boolean statusError = false;
    private boolean finished = false;

    private double angle = 0;
    private double velocity = 0;

    private int index = 0;
    private final double minimumDistance = 2.70; //meters
    private final double normalDistance = 3.15;  //meters
    private final double highDistance = 3.95;    //meters
    private final double maximumDistance = 4.60; //meterss

    private final double minimumAngle = 60.0;
    private final double maximumAngle = 70.0;

    private double desiredDistance = 0; // min = 2.7m    max =4.6m

    public ParableShot(){

    }

    /**
     * Establece la distancia para el algoritmo. Preferiblemente la distancia hacia el centro del HUB
     * @param distance Distancia hacia el centro del objetivo en metros
     */
    public void setDistance(double distance){
        desiredDistance = distance;
    }

    /**
     * Obtiene el estado de si el algoritmo ha encontrado un error. Recomendado llamar mientras se ejecuta el algoritmo.
     * @return El valor del error
     */
    public boolean getErrorStatus(){
        return statusError;
    }

    public boolean getFinishedStatus(){
        return finished;
    }

    /**
     * Obtiene el ángulo que ha encontrado el algoritmo. Tenga en cuenta que debe llamar este método al finalizar el algoritmo.
     * @return El ángulo en grados entre 60° a 70°
     */
    public double getAngle(){
        if (statusError)
            return 0;
        else
            return angle;
    }

    /**
     * Obtiene la velocidad a la que debe girar el tirador entre una de las 3 velocidades preestablecidas [7.38, 7.80, 8.26]
     * dada la distancia. Tenga en cuenta que debe llamar este método al finalizar el algoritmo.
     * @return La velocidad en metros por segundos.
     */
    public double getVelocity(){
        if (statusError)
            return 0;
        else
            return velocity;
    }

    /**
     * Ejecuta el algortimo de la parábola, tenga en cuenta que debe declarar la distancia antes de llamar este método.
     */
    public void executeAlgorithm(){
        statusError = false;
        finished = false;

        double myNewAngle = maximumAngle; //Max angle
        //kInitialVelocity = 7.38; // 2.7m - 3.2m Velocity = 7.38m/s 
                                 // 3.2 > Velocity = 8.26m/s

        distance = 0;

        if(desiredDistance < minimumDistance){
            System.out.println("Distancia muy cercana!\n");
            statusError = true; //System.exit(0); //Command finished
        }
        else if(desiredDistance > maximumDistance){
            System.out.println("Distancia muy lejana!\n");
            statusError = true; //System.exit(0); //Command finished
        }
        else if(desiredDistance >= minimumDistance && desiredDistance <= normalDistance){
            kInitialVelocity = 7.38; // m/s
        }
        else if(desiredDistance > normalDistance && desiredDistance <= highDistance){
            kInitialVelocity = 7.80; // m/s
        }
        else if(desiredDistance > highDistance){
            kInitialVelocity = 8.26; // m/s
        }
        else{
            System.out.println("Distancia no filtrada!");
            statusError = true; //System.exit(0); //Command finished
        }

        for (myNewAngle = maximumAngle; distance < desiredDistance; myNewAngle-=0.1) { //Saltos de 0.1 por la resolución del shooter
            vy0 = kInitialVelocity * Math.sin(Math.toRadians(myNewAngle));
            vx0 = kInitialVelocity * Math.cos(Math.toRadians(myNewAngle));
            
            t_MAX = Math.max(grado2(g*0.5, vy0, initialHeight)[0], grado2(g*0.5, vy0, initialHeight)[1]);
            
                for (int i = 0; i < 1000; i++) {
                    t[i] = (t_MAX/1000) * i;
                }
                for (int i = 0; i < 1000; i++) {
                    t_squared[i] = Math.pow(t[i], 2);
                }
                for (int i = 0; i < 450; i++) {
                    y_Data[i] = initialHeight + vy0*t[499+i] + 0.5*g*t_squared[499+i];
                }
                for (int i = 0; i < 450; i++) {
                    y_DataToInt[i] = (int)(y_Data[i] * 100);
                }

                index = existeEnArreglo(y_DataToInt, 264);

                if (index == -1){
                    System.out.println("\n\tERROR! - Altura no encontrada!\n");
                    break;
                }

                if (myNewAngle < minimumAngle){
                    System.out.println("ERROR! Angulo fuera de alcance");
                    break;
                }

                distance = vx0 * t[index+499];
                //System.out.println(distance + "\t" + myNewAngle);
                
                slope = calculateSlope(y_Data[index], y_Data[index-range], vx0 * t[index+500], vx0 * t[index-range+500]);
            }
            if(index == -1){

            }
            else{
                angle = myNewAngle;
                velocity = kInitialVelocity;
                finished = true;
                // System.out.println("\n/******************************************/");
                // System.out.println(" For " + desiredDistance + " meters");
                // System.out.println("  Velocity: " + kInitialVelocity + " m/s");
                // System.out.println("  Angle: " + myNewAngle + " °");
                // System.out.println("\tSlope: " + slope);
                // System.out.println("\twith a distance of: " + distance);
                // System.out.println("/******************************************/\n\n");
            }
    }

    /**
     * Método para resolver ecuaciones cuadráticas
     * @param a -> Ax^2
     * @param b -> Bx
     * @param c -> C
     * @return Los valores de las incógnitas
     */
    public static double [] grado2(double a, double b, double c){
        double delta = Math.pow(b, 2) - 4*a*c;
        double a1 = 0;
        double a2 = 0;

        if (delta < 0){
            //Valores de las raices son complejos
            //a1 = (-b+1i*Math.sqrt(-delta))/(2*a);
            //a1 = (-b-1i*Math.sqrt(-delta))/(2*a);
        }
        else{
            a1 = (-b + Math.sqrt(delta)) / (2*a);
            a1 = (-b - Math.sqrt(delta)) / (2*a);
        }

        return new double [] {a1, a2};
    }

    /**
     * Busca en el arreglo dicho número desde el final hasta el comienzo del Vector.
     * @param arreglo Vector en el cual buscará
     * @param busqueda Valor a buscar
     * @return El índice donde se encuentra dicho valor.
     */
    public static int existeEnArreglo(int[] arreglo, int busqueda) {
        int value = -1;
        for (int z = 449; z >= 11; z--) {
          if (arreglo[z] == busqueda) {
            //System.out.println("\tIndex: " + z);
            //System.out.println("\t\tValue: " + arreglo[z]);
            value = z;
            break;
          }
        }
        return value;
      }
    
    /**
     * Calcula la pendiente con la cual el proyectil entra al objetivo.
     * @param y2    y2 en la ecuación de la pendiente.
     * @param y1    y1 en la ecuación de la pendiente.
     * @param x2    x2 en la ecuación de la pendiente.
     * @param x1    x1 en la ecuación de la pendiente.
     * @return La pendiente en unidades iguales a la entrada.
     */
    public static double calculateSlope(double y2, double y1, double x2, double x1){
        double value;
        value = (y2 - y1) / (x2 - x1);
        return value;
    }

}
