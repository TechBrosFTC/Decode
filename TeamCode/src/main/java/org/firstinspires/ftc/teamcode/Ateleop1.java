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

@TeleOp (name = "Ateleop1")
public class Ateleop1 extends LinearOpMode {
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
        imu = hardwareMap.get(IMU.class, "imu");


        direitaFrente = hardwareMap.get(DcMotorEx.class, "rightFront");
        direitaTras = hardwareMap.get(DcMotorEx.class, "rightBack");
        esquerdaFrente = hardwareMap.get(DcMotorEx.class, "encoderseletor");
        esquerdaTras = hardwareMap.get(DcMotorEx.class, "leftBack");


        sugador = hardwareMap.get(DcMotorEx.class, "par");
        esteira = hardwareMap.get(DcMotorEx.class, "perp");
        encoderseletor = hardwareMap.get(DcMotorEx.class, "encoderseletor");

        atirador1 = hardwareMap.get(DcMotorEx.class, "atirador1");
        atirador2 = hardwareMap.get(DcMotorEx.class, "atirador2");

        cremalheira = hardwareMap.get(Servo.class, "cremalheira");
        seletor = hardwareMap.get(Servo.class, "seletor");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        peach = new RobotSetup(hardwareMap, esquerdaFrente, esquerdaTras, direitaFrente, direitaTras, sugador, esteira);
        RevHubOrientationOnRobot revOrientaion = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.UP);
        imu.initialize(new IMU.Parameters(revOrientaion));

        direitaFrente.setDirection(DcMotorSimple.Direction.REVERSE);
        direitaTras.setDirection(DcMotorSimple.Direction.REVERSE);
        esquerdaFrente.setDirection(DcMotorSimple.Direction.REVERSE);

        sugador.setDirection(DcMotorSimple.Direction.REVERSE);
        esteira.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        imu.resetYaw();


        limelight.setPollRateHz(90);
        //0 = identificação do motif pattern, 1 = identificação do goal vermelho, 2 = identificação do goal azul
        limelight.pipelineSwitch(1);
        limelight.start();

        while (opModeIsActive()) {
            if (gamepad1.left_stick_y != 0) {
                if (gamepad1.left_stick_y > 0) {
                    f = 0;
                    t = Math.abs(gamepad1.left_stick_y);
                } else {
                    f = Math.abs(gamepad1.left_stick_y);
                    t = 0;
                }
            } else {
                f = 0;
                t = 0;
            }
            if (gamepad1.left_stick_x != 0) {
                if (gamepad1.left_stick_x > 0) {
                    e = 0;
                    d = Math.abs(gamepad1.left_stick_x);
                } else {
                    e = Math.abs(gamepad1.left_stick_x);
                    d = 0;
                }
            } else {
                d = 0;
                e = 0;
            }

            if (f > t && f > d && f > e) { //andar para frente
                if (!andando) {
                    imu.resetYaw();
                }
                erro = alvo - gyro();
                proporcional = erro * kp;
                derivativa = (erro - ultimoerro) * kd;
                integral = erro + ki;
                direcao = (proporcional + derivativa + integral) / 100;
                movDirecionado(Math.abs(gamepad1.left_stick_y) * multiplicador, direcao);
                ultimoerro = erro;
                andando = true;
            }
            if (t > f && t > d && t > e) { //andar para trás
                if (!andando) {
                    imu.resetYaw();
                }
                erro = alvo - gyro();
                proporcional = erro * kp;
                derivativa = (erro - ultimoerro) * kd;
                integral = erro + ki;
                direcao = (proporcional + derivativa + integral) / 100;
                lastpower = gamepad1.left_stick_y * multiplicador;
                movDirecionado(Math.abs(gamepad1.left_stick_y) * multiplicador * -1, direcao);
                ultimoerro = erro;
                andando = false;
            }
            if (d > f && d > t && d > e) { //andar para Direita
                if (!andando) {
                    imu.resetYaw();
                }
                erro = alvo - gyro() ;
                proporcional = erro * 1;
                derivativa = (erro - ultimoerro) * 1;
                integral = erro + ki;
                direcao = (proporcional + derivativa + integral) / 100;
                mecanumdireitabase(Math.abs(gamepad1.left_stick_x)*multiplicador, -direcao);
                andando = true;
            }
            if (e > f && e > t && e > d) { //andar para Esquerda
                if (!andando) {
                    imu.resetYaw();
                }
                erro = alvo - gyro() ;
                proporcional = erro * 1;
                derivativa = (erro - ultimoerro) * 1;
                integral = erro + ki;
                direcao = (proporcional + derivativa + integral) / 100;
                mecanumesquerdabase(Math.abs(gamepad1.left_stick_x)*multiplicador, direcao);
                andando = true;
            }
            if (f == 0 && t == 0 && d == 0 && e == 0) {
                parar();
                andando = false;
            }
            if (gamepad1.right_stick_x > 0) {
                direitaFrente.setPower(curvapower * multiplicadorcurva * -1);
                direitaTras.setPower(curvapower * multiplicadorcurva * -1);
                esquerdaFrente.setPower(curvapower * multiplicadorcurva);
                esquerdaTras.setPower(curvapower * multiplicadorcurva);
                curva = true;
            } else if (gamepad1.right_stick_x < 0) {
                direitaFrente.setPower(curvapower * multiplicadorcurva);
                direitaTras.setPower(curvapower * multiplicadorcurva);
                esquerdaFrente.setPower(curvapower * -1 * multiplicadorcurva);
                esquerdaTras.setPower(curvapower * -1 * multiplicadorcurva);
                curva = true;
            } else {
                if (curva) {
                    parar();
                    sleep(50);
                    imu.resetYaw();
                    curva = false;
                    andando = false;
                }
            }
            if(gamepad1.cross){
                multiplicadorx = 0.5;
                multiplicador = 0.5;
                multiplicadorcurva = 0.5;
                modolento = true;
                sleep(100);
            }else{
                multiplicadorx = 1;
                multiplicador = 0.8;
                multiplicadorcurva = 1;
                modolento = false;
                sleep(100);
            }
            if(gamepad1.b){
                LLResult result = limelight.getLatestResult();
                if(result.isValid()){
                    double x = 0;
                    double y = 0;
                    int id;
                    List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                    for (LLResultTypes.FiducialResult fiducial : fiducials) {
                        id = fiducial.getFiducialId(); // The ID number of the fiducial
                        x = fiducial.getTargetXDegrees() -3  ; // Where it is (left-right)
                        y = fiducial.getTargetYDegrees(); // Where it is (up-down)
                        telemetry.addData("x", x);
                        telemetry.addData("y", y);
                        telemetry.update();
                    }
                    if(y > 8){
                        x -= -2;
                        if(Math.abs(x) > 1){
                            if(x > 0){
                                peach.mecanumDrive.curve(-x, 0.25, 0.25 );
                            }else{
                                peach.mecanumDrive.curve(-x, 0.25, 0.25 );
                            }
                        }
                        atirador1.setPower(-0.95);
                        atirador2.setPower(0.95);
                        sleep(1000);
                        cremalheira.setPosition(1);
                        sleep(450);
                        cremalheira.setPosition(0);
                        atirador1.setPower(0);
                        atirador2.setPower(0);
                        sleep(500);
                        cremalheira.setPosition(0.5);
                    }else{
                        if(Math.abs(x) > 1){
                            if(x > 0){
                                peach.mecanumDrive.curve(-x, 0.25, 0.25 );
                            }else{
                                peach.mecanumDrive.curve(-x, 0.25, 0.25 );
                            }
                        }
                        atirador1.setPower(-0.95);
                        atirador2.setPower(0.95
                        );
                        sleep(1600);
                        cremalheira.setPosition(1);
                        sleep(450);
                        cremalheira.setPosition(0);
                        atirador1.setPower(0);
                        atirador2.setPower(0);
                        sleep(500);
                        cremalheira.setPosition(0.5);
                    }
                }
            }
            if(gamepad2.a){
                sugador.setPower(powersugador);
                esteira.setPower(poweresteira);
            }else{
                sugador.setPower(0);
                esteira.setPower(0);
            }
            if(gamepad2.right_trigger > 0){
                atirador1.setPower(1);
                atirador2.setPower(-1);
            }else if(gamepad2.left_trigger > 0){
                atirador1.setPower(-1);
                atirador2.setPower(1);
            }else{
                atirador1.setPower(0);
                atirador2.setPower(0);
            }
            /*if(gamepad2.x) {
                cremalheira.setPosition(1);
                sleep(700);
                cremalheira.setPosition(0.5);
            }else if (gamepad2.y) {
                cremalheira.setPosition(0);
                sleep(700);
                cremalheira.setPosition(0.5);
            }*/
            if(gamepad2.right_bumper){
                seletor.setPosition(1);
                sleep(50);
            } else if (gamepad2.left_bumper) {
                seletor.setPosition(0);
                sleep(50);
            }else{
                seletor.setPosition(0.5);
            }
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