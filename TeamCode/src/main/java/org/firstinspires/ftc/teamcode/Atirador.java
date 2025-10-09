package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.deeptrack.Seletor;


public class Atirador {
    private DcMotorEx atirador1, atirador2;

    public Atirador(DcMotorEx atirador1, DcMotorEx atirador2){
        this.atirador1 = atirador1;
        this.atirador2 = atirador2;
        this.atirador1.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    public void atirar(double power){
        motoresAtirador(power);
    }

    private void motoresAtirador(double power){
        atirador1.setPower(power);
        atirador2.setPower(power);
    }
    public void pararAtirador(){
        atirador1.setPower(0);
        atirador2.setPower(0);
    }


}
