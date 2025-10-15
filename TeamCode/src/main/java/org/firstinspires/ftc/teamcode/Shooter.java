package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class Shooter {
    public DcMotorEx atirador1, atirador2;//torno de 2200 pra atirar de longe
    public Shooter(DcMotorEx atirador1, DcMotorEx atirador2){
        this.atirador1 = atirador1;
        this.atirador2 = atirador2;
    }
    double power = 0.85;
    double intake_power = 0;
    double powercorreia = 0.5;
    int sentido = 1;
    double previousError;
    public double getVel(){
        return (Math.abs(atirador1.getVelocity())+Math.abs(atirador2.getVelocity()))/2;
    }
    public void acelerarAtirador(double velocity){
        double integral = 0;
        double kp = 0.2, kd = 0, ki = 0;
        double currentVelocity = (Math.abs(atirador1.getVelocity())+Math.abs(atirador2.getVelocity()))/2;
        double error = velocity - currentVelocity;
        double derivative = (error - previousError);

        double output = kp * error + ki * integral + kd * derivative;
        previousError = error;

// Limita o output para [-1, 1]
        output = Math.max(0, Math.min(1, output));

// Aplica aos dois motores
        atirador1.setPower(-output);
        atirador2.setPower(output);
    }
}
