package org.firstinspires.ftc.teamcode;

import android.webkit.WebHistoryItem;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.deeptrack.Armazenamento;

import java.util.List;

@TeleOp (name = "telemetry")
public class Telemetry extends LinearOpMode {
    private IMU imu;
    private DcMotorEx direitaFrente, direitaTras, esquerdaFrente, esquerdaTras;//instancia dos motores de movimento
    private DcMotorEx sugador, esteira, encoderseletor;//instância dos motores do intake
    private DcMotorEx atirador1, atirador2;//instancia dos motores do outtake
    private Servo cremalheira, seletor;//
    private Limelight3A limelight; //instancia da camera
    private Armazenamento armazenamento = new Armazenamento();//
    double powersugador = 0.8, poweresteira = 0.8;
    double power = 1, curvapower = 0.4, multiplicadorx = 1, multiplicador = 0.6, multiplicadorcurva = 1;//variáveis de movimentação
    double proporcional, derivativa, integral, erro, direcao = 0, ultimoerro = 0, alvo = 0; //Variáveis de cáluclo do PID
    boolean curva = false, andando = false;//Controle de movimento
    double kp = 2.2, kd = 2, ki = 0;//Constantes PID movimentação
    double lastpower = 0; //controle de movimento
    float f,t,d,e;//Direções do joystick
    boolean modolento = false;//controle de movimento
    public RobotSetup peach;



    ///////////////////////////////////////////////////////////////////////

    public void runOpMode() throws InterruptedException {
        waitForStart();

        while (opModeIsActive()){
            telemetry.addData("imu", imu.getRobotYawPitchRollAngles().getYaw());
            telemetry.addData("encoder", encoderseletor.getCurrentPosition());
            telemetry.addData("par", sugador.getCurrentPosition());
            telemetry.addData("perp", esteira.getCurrentPosition());
            telemetry.update();
        }
    }
    /////////////////////////////////////////////////////funções///////////////////////////////////////////////////////////////////////
    public void mecanumesquerdabase(double power, double direcao){
        if (direcao < 0){
            direitaFrente.setPower(power);
            direitaTras.setPower((power-direcao)*-1);
            esquerdaFrente.setPower(power*-1);
            esquerdaTras.setPower((power-direcao));
        }else{
            direitaFrente.setPower(power);
            direitaTras.setPower((power-direcao)*-1);
            esquerdaFrente.setPower(power*-1);
            esquerdaTras.setPower((power-direcao));
        }
    }
    public void mecanumdireitabase(double power, double direcao){
        if (direcao < 0){
            direitaFrente.setPower((power-direcao)*-1);
            direitaTras.setPower(power);
            esquerdaFrente.setPower((power-direcao));
            esquerdaTras.setPower(power*-1);
        }else{
            direitaFrente.setPower(power*-1);
            direitaTras.setPower((power-direcao));
            esquerdaFrente.setPower(power);
            esquerdaTras.setPower((power-direcao)*-1);
        }
    }
    public void movDirecionado(double power, double direcao){
        if (power < 0) {
            if (direcao >= 0){
                direitaFrente.setPower(power+direcao);
                direitaTras.setPower(power+direcao);
                esquerdaFrente.setPower(power);
                esquerdaTras.setPower(power);
            }else{
                direitaFrente.setPower(power);
                direitaTras.setPower(power);
                esquerdaFrente.setPower(power-direcao);
                esquerdaTras.setPower(power-direcao);
            }
        }else{
            if (direcao >= 0) {
                direitaFrente.setPower(power);
                direitaTras.setPower(power);
                esquerdaFrente.setPower(power-direcao);
                esquerdaTras.setPower(power-direcao);

            } else {
                direitaFrente.setPower(power+direcao);
                direitaTras.setPower(power+direcao);
                esquerdaFrente.setPower(power);
                esquerdaTras.setPower(power);
            }
        }
    }
    /*public void resetarEncoderOdometria() {
        odometriaX.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        odometriaX.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        odometriaY.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        odometriaY.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }*/
    public void parar(){
        direitaFrente.setPower(0);
        direitaTras.setPower(0);
        esquerdaFrente.setPower(0);
        esquerdaTras.setPower(0);
    }
    public double gyro(){
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }
}