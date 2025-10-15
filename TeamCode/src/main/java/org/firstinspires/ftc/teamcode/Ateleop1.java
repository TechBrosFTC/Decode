package org.firstinspires.ftc.teamcode;

import android.webkit.WebHistoryItem;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.deeptrack.Armazenamento;
import org.firstinspires.ftc.teamcode.deeptrack.Seletor;

import java.util.List;

@TeleOp (name = "Ateleop1")
public class Ateleop1 extends LinearOpMode {
    private IMU imu;
    private DcMotorEx direitaFrente, direitaTras, esquerdaFrente, esquerdaTras;//instancia dos motores de movimento
    private DcMotorEx sugador, esteira, encoderseletor;//instância dos motores do intake
    private DcMotorEx atirador1, atirador2;//instancia dos motores do outtake
    private Servo cremalheira, helice;//
    private Limelight3A limelight; //instancia da camera
    private ColorSensor sensorPos2, sensorPos4;
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
    Shooter shooter;
    Boolean sugar = false, collecting = true;
    Boolean acelerando = false;
    double shootervelocity = 0;
    int slotEmColeta = -1;

    // debounce
    boolean prevA = false;
    boolean prevX = false;
    int red, green, blue, alpha;

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
        helice = hardwareMap.get(Servo.class, "seletor");

        sensorPos2 = hardwareMap.get(ColorSensor.class, "sensoresquerda");
        sensorPos4 = hardwareMap.get(ColorSensor.class, "sensordireita");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        shooter = new Shooter(atirador1, atirador2);

        peach = new RobotSetup(hardwareMap, esquerdaFrente, esquerdaTras, direitaFrente, direitaTras, sugador, esteira);
        RevHubOrientationOnRobot revOrientaion = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.UP);
        imu.initialize(new IMU.Parameters(revOrientaion));

        direitaFrente.setDirection(DcMotorSimple.Direction.REVERSE);
        direitaTras.setDirection(DcMotorSimple.Direction.REVERSE);
        esquerdaFrente.setDirection(DcMotorSimple.Direction.REVERSE);

        armazenamento.setSlot(0, Armazenamento.EstadoSlot.VERDE);
        armazenamento.setSlot(1, Armazenamento.EstadoSlot.ROXO);
        armazenamento.setSlot(2, Armazenamento.EstadoSlot.ROXO);

        Seletor seletor = new Seletor(armazenamento, helice, encoderseletor);
        Intake intake = new Intake(sugador, esteira);

        Thread garra = new Thread(() -> {
            while (opModeIsActive() && !Thread.currentThread().isInterrupted()) {
                if (gamepad2.dpad_left) {
                    intake.sugar();
                    sleep(350);
                    seletor.verde();
                    intake.stop();
                }
                if (gamepad2.dpad_right) {
                    intake.sugar();
                    sleep(350);
                    seletor.roxo();
                    intake.stop();
                }
                if(gamepad2.a){
                    intake.sugar();
                    sugar = true;
                    while(sensorPos4.green() < 100){}
                    int slotcheio = seletor.getSlotNaPosicaoColeta();
                    telemetry.addData("slot cheio" ,slotcheio);
                    armazenamento.setSlot(slotcheio, Armazenamento.EstadoSlot.CHEIO);
                    boolean vazio = false;
                    int slotvazio = 3;
                    for (int i = 0; i < 3; i++) {
                        if (armazenamento.getSlot(i) == Armazenamento.EstadoSlot.VAZIO) {
                            vazio = true;
                            slotvazio = i;
                            break;
                        }
                    }
                    green = 0;
                    blue = 0;
                    red = 0;
                    alpha = 0;
                    sleep(200);
                    for(int i = 0; i < 200; i++){
                        green += sensorPos4.green();
                        red += sensorPos4.red();
                        blue += sensorPos4.blue();
                        alpha += sensorPos4.alpha();
                    }
                    green /= 199;
                    red /= 199;
                    blue /= 199;
                    alpha /= 199;
                    if(blue > 500 && green < 300){
                        armazenamento.setSlot(slotcheio, Armazenamento.EstadoSlot.ROXO);
                    }else{
                        armazenamento.setSlot(slotcheio, Armazenamento.EstadoSlot.VERDE);
                    }
                    sleep(400);
                    if (vazio) {
                        seletor.posicaoColeta(slotvazio);
                    }
                    sugar = false;
                    intake.stop();
                }
                if(gamepad2.right_bumper && sugar){
                    seletor.rotacionarRelativo(1);
                }
                if(gamepad2.left_bumper && sugar){
                    seletor.rotacionarRelativo(-1);
                }
                if(gamepad2.x){

                }
            }
        });

        waitForStart();

        imu.resetYaw();


        limelight.setPollRateHz(90);
        //0 = identificação do motif pattern, 1 = identificação do goal vermelho, 2 = identificação do goal azul
        limelight.pipelineSwitch(1);
        limelight.start();

        armazenamento.setSlot(0, Armazenamento.EstadoSlot.VAZIO);
        armazenamento.setSlot(1, Armazenamento.EstadoSlot.VAZIO);
        armazenamento.setSlot(2, Armazenamento.EstadoSlot.VAZIO);

        garra.start();

        while (opModeIsActive() && !Thread.currentThread().isInterrupted()) {
            telemetry.addData("imu", imu.getRobotYawPitchRollAngles().getYaw());
            telemetry.addData("encoder", encoderseletor.getCurrentPosition());
            telemetry.addData("armazenamento 0", armazenamento.getSlot(0));
            telemetry.addData("armazenamento 1", armazenamento.getSlot(1));
            telemetry.addData("armazenamento 2", armazenamento.getSlot(2));
            telemetry.addData("par", sugador.getCurrentPosition());
            telemetry.addData("perp", esteira.getCurrentPosition());
            telemetry.update();
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
                erro = alvo - imu.getRobotYawPitchRollAngles().getYaw();
                proporcional = erro * kp;
                derivativa = (erro - ultimoerro) * kd;
                integral = erro + ki;
                direcao = (proporcional + derivativa + integral) / 100;
                movDirecionado(Math.abs(gamepad1.left_stick_y) * multiplicador, direcao);
                ultimoerro = erro;
                andando = true;
            }//frente
            if (t > f && t > d && t > e) { //andar para trás
                if (!andando) {
                    imu.resetYaw();
                }
                erro = alvo - imu.getRobotYawPitchRollAngles().getYaw();
                proporcional = erro * kp;
                derivativa = (erro - ultimoerro) * kd;
                integral = erro + ki;
                direcao = (proporcional + derivativa + integral) / 100;
                lastpower = gamepad1.left_stick_y * multiplicador;
                movDirecionado(Math.abs(gamepad1.left_stick_y) * multiplicador * -1, direcao);
                ultimoerro = erro;
                andando = false;
            }//tras
            if (d > f && d > t && d > e) { //andar para Direita
                if (!andando) {
                    imu.resetYaw();
                }
                erro = alvo - imu.getRobotYawPitchRollAngles().getYaw() ;
                proporcional = erro * 1;
                derivativa = (erro - ultimoerro) * 1;
                integral = erro + ki;
                direcao = (proporcional + derivativa + integral) / 100;
                mecanumdireitabase(Math.abs(gamepad1.left_stick_x)*multiplicador, -direcao);
                andando = true;
            }//direita
            if (e > f && e > t && e > d) { //andar para Esquerda
                if (!andando) {
                    imu.resetYaw();
                }
                erro = alvo - imu.getRobotYawPitchRollAngles().getYaw();
                proporcional = erro * 1;
                derivativa = (erro - ultimoerro) * 1;
                integral = erro + ki;
                direcao = (proporcional + derivativa + integral) / 100;
                andando = true;
            }//esquerda
            if (f == 0 && t == 0 && d == 0 && e == 0) {
                parar();                mecanumesquerdabase(Math.abs(gamepad1.left_stick_x)*multiplicador, direcao);

                andando = false;
            }//parar
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
            }else{//modolento config
                multiplicadorx = 1;
                multiplicador = 0.8;
                multiplicadorcurva = 1;
                modolento = false;
                sleep(100);
            }
            if (gamepad2.b) {
                if ((seletor.posicaoAtual % 2) != 0) {
                    LLResult result = limelight.getLatestResult();

                    if (result.isValid()) {
                        double x = 0;
                        double y = 0;
                        int id;
                        double txRobo = 0;

                        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                        for (LLResultTypes.FiducialResult fiducial : fiducials) {
                            id = fiducial.getFiducialId();            // ID do fiducial
                            x = fiducial.getTargetXDegrees();         // posição horizontal
                            y = fiducial.getTargetYDegrees();         // posição vertical

                            // telemetry.addData("x", x);
                            // telemetry.addData("y", y);
                            // telemetry.update();

                            double txCam = x;                         // ângulo da câmera
                            double distance = getDistanceToTarget(y); // calcula distância ao alvo
                            double xOffset = -0.1065;                 // deslocamento da câmera em relação ao centro

                            txRobo = Math.toDegrees(
                                    Math.atan(Math.tan(Math.toRadians(txCam)) + (xOffset / distance))
                            );
                        }

                        if (y > 8) {
                            // Mira com ajuste para alvo alto
                            peach.mecanumDrive.curve(-txRobo, 0.25, 0.25);
                            telemetry.addData("txrobo", txRobo);

                            Thread acel = new Thread(() -> {
                                while (opModeIsActive()) {
                                    shooter.acelerarAtirador(1850);
                                }
                            });
                            acel.start();

                            while (shooter.getVel() < 1800) { }
                            cremalheira.setPosition(1);

                            while (atirador2.getVelocity() > 1600) { }
                            cremalheira.setPosition(0);

                            sleep(700);
                            acel.interrupt();
                            cremalheira.setPosition(0.5);
                        } else {
                            // Mira com ajuste para alvo baixo
                            peach.mecanumDrive.curve(-txRobo, 0.25, 0.25);

                            Thread acel = new Thread(() -> {
                                while (opModeIsActive()) {
                                    shooter.acelerarAtirador(2250);
                                }
                            });
                            acel.start();

                            while (shooter.getVel() < 2250) { }
                            cremalheira.setPosition(1);

                            while (atirador2.getVelocity() > 1800) { }
                            acel.interrupt();
                            cremalheira.setPosition(0);

                            sleep(700);
                            cremalheira.setPosition(0.5);
                        }

                        // Atualiza slots do armazenamento
                        if (seletor.posicaoAtual == 1) {
                            armazenamento.setSlot(1, Armazenamento.EstadoSlot.VAZIO);
                        } else if (seletor.posicaoAtual == 3) {
                            armazenamento.setSlot(0, Armazenamento.EstadoSlot.VAZIO);
                        } else if (seletor.posicaoAtual == 5) {
                            armazenamento.setSlot(2, Armazenamento.EstadoSlot.VAZIO);
                        }
                    }
                }
            } // Fim do if gamepad2.b



            if(gamepad2.right_trigger > 0){
                acelerando = true;
            }else if(gamepad2.left_trigger > 0){
                acelerando = false;
            }
            if(acelerando){
                shooter.acelerarAtirador(shootervelocity);
            }else{
                shooter.atirador1.setPower(0);
                shooter.atirador2.setPower(0);
            }
            if(gamepad2.dpad_left){
                shootervelocity += 10;
            }else if(gamepad2.dpad_right){
                shootervelocity -= 10;
            }
            if(gamepad2.right_trigger > 0) {
                cremalheira.setPosition(1);
            }else if (gamepad2.left_trigger > 0) {
                cremalheira.setPosition(0);
            }else{
                cremalheira.setPosition(0.5);
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

    public double getDistanceToTarget(double TY) {
        double hCam = 0.39; // altura da câmera em metros
        double hTarget = 0.77; // altura do alvo em metros
        double camAngle = 0; // ângulo de inclinação da câmera
        double ty = Math.toRadians(TY); // ângulo vertical do Limelight

        // distância horizontal até o alvo
        double distance = (hTarget - hCam) / Math.tan(camAngle + ty);
        return distance; // em metros
    }

    public void parar(){
        direitaFrente.setPower(0);
        direitaTras.setPower(0);
        esquerdaFrente.setPower(0);
        esquerdaTras.setPower(0);
    }
    private Armazenamento.EstadoSlot classificarPorSensor(ColorSensor sensor) {
        int r = sensor.red();
        int g = sensor.green();
        int b = sensor.blue();

        // simples heurística:
        if (g > 130 && b > 20 && r > 20) {
            return Armazenamento.EstadoSlot.VERDE;
        } else if (g < 130 && b > 20 && r > 20) {
            return Armazenamento.EstadoSlot.ROXO;
        } else {
            return Armazenamento.EstadoSlot.VAZIO; // não identificada
        }
    }

    private int calcularSentido(int posicaoAtual, int slotVazio) {
        // posição de coleta (0, 2 ou 4) -> 6 posições no total
        // diferença mínima no círculo de 6 posições
        int diferenca = slotVazio * 2 - posicaoAtual; // slotVazio -> 0,1,2 mapeado para 0,2,4
        if (diferenca > 3) diferenca -= 6;
        if (diferenca < -3) diferenca += 6;
        return (diferenca >= 0) ? 1 : -1; // +1 = horário, -1 = anti-horário
    }

    private ColorSensor sensorParaSentido(int sentido) {
        return (sentido > 0) ? sensorPos2 : sensorPos4; // sensor da direita para horário, esquerda para anti-horário
    }

}