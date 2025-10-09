package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Intake {
    private DcMotorEx sugador;
    private DcMotorEx esteira;
    public Intake(DcMotorEx sugador, DcMotorEx esteira){
        this.sugador = sugador;
        this.esteira = esteira;
        this.sugador.setDirection(DcMotorSimple.Direction.REVERSE);
        this.esteira.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    public void ativar(double power){
        sugador.setPower(power);
        esteira.setPower(power);
    }
    public void desativar(){
        sugador.setPower(0);
        esteira.setPower(0);
    }
}
