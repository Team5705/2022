/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * Clase PID diseñada por el equipo FRC 5705
 */
public class PID {
    private double bias = 0;

    private final double kP;
    private final double kI;
    private final double kD;

    private boolean valueInverted = false;
    private double desiredValue;

    private final double kT = 20; // Tiempo de ejecución del comnado en línea, son 0.020 segundos por ciclo
    private double err = 0; // Error
    private double errI = 0; // Integral
    private double errD = 0; // Diferencial
    private double errPp = 0; // Error pasado del pasado
    private double errP = 0; // Error pasado

    private double PID;

    public PID(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;

    }

    /**
     * PID.
     * 
     * @param kP           Valor Proporcional
     * @param kI           Valor Integral
     * @param kD           Valor Derivativo
     * @param desiredValue Valor deseado o punto de ajuste
     */
    public PID(double kP, double kI, double kD, double desiredValue) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.desiredValue = desiredValue;

    }

    /**
     * PID. Usar valueInverted en caso de que el PID funcione de manera invertida.
     * 
     * @param kP           Valor Proporcional
     * @param kI           Valor Integral
     * @param kD           Valor Derivativo
     * @param desiredValue Valor deseado o punto de ajuste
     * @param bias         Valor que tomara de awebo
     */
    public PID(double kP, double kI, double kD, double desiredValue, double bias, boolean valueInverted) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.desiredValue = desiredValue;
        this.bias = bias;
        this.valueInverted = valueInverted;

    }

    /**
     * PID. Usar valueInverted en caso de que el PID funcione de manera invertida.
     * 
     * @param kP            Valor Proporcional
     * @param kI            Valor Integral
     * @param kD            Valor Derivativo
     * @param desiredValue  Valor deseado o punto de ajuste
     * @param valueInverted Error invertido
     */
    public PID(double kP, double kI, double kD, double desiredValue, boolean valueInverted) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.desiredValue = desiredValue;
        this.valueInverted = valueInverted;

    }

    /**
     * PID. Usar valueInverted en caso de que el PID funcione de manera invertida.
     * 
     * @param kP            Valor Proporcional
     * @param kI            Valor Integral
     * @param kD            Valor Derivativo
     * @param desiredValue  Valor deseado o punto de ajuste
     * @param valueInverted Error invertido
     */
    public PID(double kP, double kI, double kD, boolean valueInverted) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.valueInverted = valueInverted;

    }

    /**
     * Inicia el procesamiento del PID siempre y cuando se mande a llamar. Mande a
     * llamar antes de obtener el valor.
     * 
     * @param value Valor del sensor a seguir
     */
    public void runPID(double value) {
        err = (valueInverted ? -1.0 : 1.0) * (desiredValue - value);

        errI = (err * kT) + errP;

        errD = (err - errPp) / kT;

        errP = errI;
        errPp = err;

        /*--------------------------------------*/
        /* PID */
        /*--------------------------------------*/

        PID = ((err * kP) + (errI * kI) + (errD * kD) + ((valueInverted ? -1.0 : 1.0) * bias));

    }

    public void runPIDErr(double error) {
        double biass = bias;
        err = (valueInverted ? -1.0 : 1.0) * error;

        errI = (err * kT) + errP;

        errD = (err - errPp) / kT;

        errP = errI;
        errPp = err;

        if (err < 0)
            biass = -bias;
        /*--------------------------------------*/
        /* PID */
        /*--------------------------------------*/

        PID = ((err * kP) + (errI * kI) + (errD * kD) + biass);

    }

    /**
     * Devuelve el valor del PID.
     * 
     * @return PID
     */
    public double valuePID() {
        return PID;

    }

}
