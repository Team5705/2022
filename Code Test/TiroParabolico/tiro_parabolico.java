class tiro_parabolico {
    static final double kSpeedTransfer = 0.2055; // %
    static final double g = -9.80665; // m/s^2
    static final double initialHeight = 0.82804; // m
    static final int range = 10;
    
    static double kInitialVelocity = 7.38; // m/s
    static double myAngle = 70.85; // degrees °
    static double distance; // m
    static double maxVelocity = 8.26;
    static double minVelocity = 7.05;

    static double [] t = new double[1000]; // Vector con los datos del tiempo
    static double [] t_squared = new double[1000]; // Vector con los datos del tiempo elevados al cuadrado
    static double [] t_plus = new double[1000];
    static double [] x_Data = new double[300];
    static double [] y_Data = new double[300];
    static int [] y_DataToInt = new int[300];

    static double vy0; // velocidad inicial en y [m/sg]
    static double vx0; // velocidad inicial en x [m/sg]
    static double t_MAX;

    static double slope;
    static double myDistance;

    public static void main(String[] args) {
        vy0 = kInitialVelocity * Math.sin(Math.toRadians(myAngle));
        vx0 = kInitialVelocity * Math.cos(Math.toRadians(myAngle));

        t_MAX = Math.max(grado2(g*0.5, vy0, initialHeight)[0], grado2(g*0.5, vy0, initialHeight)[1]);

        for (int i = 0; i < 1000; i++) {
            t[i] = (t_MAX/1000) * i;
        }
        for (int i = 0; i < 1000; i++) {
            t_squared[i] = Math.pow(t[i], 2);
        }
        for (int i = 0; i < 300; i++) {
            y_Data[i] = initialHeight + vy0*t[499+i] + 0.5*g*t_squared[499+i];
        }
        for (int i = 0; i < 300; i++) {
            y_DataToInt[i] = (int)(y_Data[i] * 100);
        }
        
        int index = existeEnArreglo(y_DataToInt, 264);
        System.out.println("Index: " + index);
        System.out.println("Valor en el índice: " + y_DataToInt[index]);
        System.out.println("Valor real: " + y_Data[index] + "\n\n");

        slope = calculateSlope(y_Data[index], y_Data[index-range], vx0*t[index+500], vx0*t[index-range+500]);
        distance = vx0*t[index+500];

        System.out.println("Pendiente: " + slope);
        System.out.println("Distancia: " + distance + " metros");
        System.out.println(Math.toDegrees( Math.acos(4/(t[index+500] * kInitialVelocity))) ); 
        //System.out.println(  );
    }

    /**
     * 
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

    public static int existeEnArreglo(int[] arreglo, int busqueda) {
        for (int x = 0; x < arreglo.length; x++) {
          if (arreglo[x] == busqueda) {
            return x;
          }
        }
        return -1;
      }

    public static double calculateSlope(double y2, double y1, double x2, double x1){
        double value;
        value = (y2 - y1) / (x2 - x1);
        return value;
    }
}