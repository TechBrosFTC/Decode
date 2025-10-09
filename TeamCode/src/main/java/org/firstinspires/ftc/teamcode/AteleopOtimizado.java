package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "AteleopOtimizado")
public class AteleopOtimizado extends LinearOpMode {

    private IMU imu;
    private DcMotorEx direitaFrente;
    private DcMotorEx direitaTras;
    private DcMotorEx esquerdaFrente;
    private DcMotorEx esquerdaTras;

    double kp = 2.2;
    double kd = 2;
    double ki = 0.0006;

    double alvo = 0;
    double ultimoerro = 0;
    double integral = 0;

    double integralMax = 1000;
    double integralMin = -1000;

    final double deadzone = 0.05;

    @Override
    public void runOpMode() throws InterruptedException {
        imu = hardwareMap.get(IMU.class, "imu");
        direitaFrente = hardwareMap.get(DcMotorEx.class, "rightFront");
        direitaTras = hardwareMap.get(DcMotorEx.class, "rightBack");
        esquerdaFrente = hardwareMap.get(DcMotorEx.class, "leftFront");
        esquerdaTras = hardwareMap.get(DcMotorEx.class, "leftBack");

        direitaFrente.setDirection(DcMotorSimple.Direction.REVERSE);
        direitaTras.setDirection(DcMotorSimple.Direction.REVERSE);
        esquerdaFrente.setDirection(DcMotorSimple.Direction.REVERSE);
        esquerdaTras.setDirection(DcMotorSimple.Direction.REVERSE);

        RevHubOrientationOnRobot revOrientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT);
        imu.initialize(new IMU.Parameters(revOrientation));

        waitForStart();
        imu.resetYaw();
        alvo = getYaw();

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rxInput = gamepad1.right_stick_x;

            boolean moveXY = Math.abs(y) > deadzone || Math.abs(x) > deadzone;
            boolean rotateManual = Math.abs(rxInput) > deadzone;

            double atual = getYaw();
            double erro = alvo - atual;

            // Normalizar erro para faixa [-180, 180]
            while (erro > 180) erro -= 360;
            while (erro < -180) erro += 360;

            if (rotateManual) {
                // Quando gira manualmente, aplica direto o joystick no rx e atualiza alvo
                integral = 0;
                ultimoerro = 0;
                alvo = atual;
            } else if (moveXY) {
                // PID para correção de giro somente se está se movendo e não girando manualmente
                integral += erro;
                if (integral > integralMax) integral = integralMax;
                else if (integral < integralMin) integral = integralMin;

                double derivativa = erro - ultimoerro;
                ultimoerro = erro;

                double proporcional = kp * erro;
                double integralTerm = ki * integral;
                double derivativaTerm = kd * derivativa;

                double rxPID = (proporcional + integralTerm + derivativaTerm) / 100;

                rxInput = rxPID;
            } else {
                // Sem movimento ou rotação, zera potencia e integral
                integral = 0;
                ultimoerro = 0;
                alvo = atual;
                rxInput = 0;
                y = 0;
                x = 0;
            }

            // Cálculo potência dos motores para mecanum drive
            double flPower = y + x + rxInput;
            double frPower = y - x - rxInput;
            double blPower = y - x + rxInput;
            double brPower = y + x - rxInput;

            double max = Math.max(Math.max(Math.abs(flPower), Math.abs(frPower)),
                    Math.max(Math.abs(blPower), Math.abs(brPower)));
            if (max > 1.0) {
                flPower /= max;
                frPower /= max;
                blPower /= max;
                brPower /= max;
            }

            esquerdaFrente.setPower(flPower);
            direitaFrente.setPower(frPower);
            esquerdaTras.setPower(blPower);
            direitaTras.setPower(brPower);

            telemetry.addData("Erro (°)", erro);
            telemetry.addData("Proporcional", kp * erro);
            telemetry.addData("Integral", ki * integral);
            telemetry.addData("Derivativa", kd * (erro - ultimoerro));
            telemetry.addData("Alvo (°)", alvo);
            telemetry.addData("Atual (°)", atual);
            telemetry.addData("Joystick (y,x,rx)", "%.2f, %.2f, %.2f", y, x, rxInput);
            telemetry.update();
        }
    }
    private double getYaw() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }
}
