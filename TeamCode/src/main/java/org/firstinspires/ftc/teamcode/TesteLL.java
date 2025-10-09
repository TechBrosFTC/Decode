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
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

@Autonomous (name = "testeLL")

public class TesteLL extends LinearOpMode {
    private Limelight3A limelight;
    private Servo servo;
    double lastPosition = 1;
    double position = 0;
    double rest = 1;
    double lasttx;

    @Override
    public void runOpMode() throws InterruptedException {
        servo = hardwareMap.get(Servo.class, "servo");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(1);
        limelight.setPollRateHz(90);
        limelight.start(); // This tells Limelight to start looking!
        servo.setPosition(rest);
        waitForStart();
        while (opModeIsActive()){
            //range do servo vai de 0 a 0.65, 1 posição recolhida
            LLResult result = limelight.getLatestResult();

            double tx = Math.round(result.getTx()); // How far left or right the target is (degrees)
            double ty = result.getTy(); // How far up or down the target is (degrees);
            double ta = result.getTa(); // How big the target looks (0%-100% of the image)
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fiducial : fiducials) {
                int id = fiducial.getFiducialId(); // The ID number of the fiducial
                double x = fiducial.getTargetXDegrees(); // Where it is (left-right)
                double y = fiducial.getTargetYDegrees(); // Where it is (up-down)
                double StrafeDistance_3D = fiducial.getRobotPoseTargetSpace().getPosition().y;
                telemetry.addData("Fiducial " + id, "is " + StrafeDistance_3D + " meters away");
            }
            telemetry.addData("Target X", tx);
            telemetry.addData("Target Y", ty);
            telemetry.addData("Target Area", ta);
            telemetry.addData("position",position);
            telemetry.update();
            if(Math.abs(position - lastPosition) > .18 && result.getTx() != 0){
                servo.setPosition(position);
                lastPosition = position;
            }

            sleep(100);


        /*if(Math.abs(position - lastPosition) > 0.03){
            servo.setPosition(position);
            lastPosition = position;
        }*/
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

}

