package org.firstinspires.ftc.teamcode;
import static java.lang.Thread.sleep;

import android.util.JsonReader;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

@Autonomous (name = "testeLL")

public class TesteLL extends LinearOpMode {
    public DcMotorEx direitaFrente, direitaTras, esquerdaFrente, esquerdaTras;
    private Limelight3A limelight;
    private Servo servo;
    double lastPosition = 1;
    double position = 0;
    double rest = 1;
    double lasttx;
    boolean atirando = true;
    int tirostep = 0;
    double power = 0.85;
    double intake_power = 0;
    double powercorreia = 0.5;
    int sentido = 1;
    double previousError;
    DcMotorEx atirador1, atirador2;


    @Override
    public void runOpMode() throws InterruptedException {
        servo = hardwareMap.get(Servo.class, "servo");
        atirador1 = hardwareMap.get(DcMotorEx.class, "atirador1");
        atirador2 = hardwareMap.get(DcMotorEx.class, "atirador2");
        direitaFrente = hardwareMap.get(DcMotorEx.class, "rightFront");
        direitaTras = hardwareMap.get(DcMotorEx.class, "encoderseletor");
        esquerdaFrente = hardwareMap.get(DcMotorEx.class, "leftFront");
        esquerdaTras = hardwareMap.get(DcMotorEx.class, "leftBack");
        //limelight = hardwareMap.get(Limelight3A.class, "limelight");
        /*limelight.pipelineSwitch(1);
        limelight.setPollRateHz(90);
        limelight.start(); // This tells Limelight to start looking!*/
        servo.setPosition(rest);
        waitForStart();
        while (opModeIsActive()){
            telemetry.addData("step:", tirostep);
            telemetry.addData("velocity", (Math.abs(atirador1.getVelocity())+Math.abs(atirador2.getVelocity()))/2);
            telemetry.update();

            direitaFrente.setPower(1);
            esquerdaFrente.setPower(1);
        }
    }

    public double txToServoPos(double tx) {
        double txMin = -22.0, txMax = 22.0;
        double posMin = .05, posMax = .6;
        double slope = (posMax - posMin) / (txMax - txMin); // 0.65 / 46
        double pos = (tx - txMin) * slope + posMin;         // (tx + 23) * slope
        if (pos < posMin) pos = posMin;
        if (pos > posMax) pos = posMax;
        return pos;
    }
    /*public double velocity() {
        return (Math.abs(atirador1.getVelocity()) + Math.abs(atirador2.getVelocity())) /2;
    }*/
}

