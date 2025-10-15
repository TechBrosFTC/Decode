package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public class Intake {
    private DcMotorEx sugador, esteira;
    public double powersugador = 0.65, poweresteira = 0.7;
    public Intake(DcMotorEx sugador, DcMotorEx esteira){
        this.sugador = sugador;
        this.esteira = esteira;
    }
    public void sugar(){
        sugador.setPower(powersugador);
        esteira.setPower(poweresteira);
    }
    public void sugar(double power_sugador , double power_esteira){
        sugador.setPower(powersugador);
        esteira.setPower(poweresteira);
    }
    public void stop(){
        sugador.setPower(0);
        esteira.setPower(0);
    }
}
