package org.firstinspires.ftc.teamcode;

// RR-specific imports
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
        import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.deeptrack.Armazenamento;
import org.firstinspires.ftc.teamcode.deeptrack.Seletor;
import java.util.List;

@Autonomous (name="Auto")
public class Auto extends LinearOpMode{
    private DcMotorEx sugador, esteira, encoderseletor;//instância dos motores do intake
    private DcMotorEx atirador1, atirador2;//instancia dos motores do outtake
    private Servo cremalheira, servoseletor;//
    Atirador atirador;
    Intake intake;
    Armazenamento armazenamento = new Armazenamento();
    Seletor seletor;
    Limelight3A limelight;
    int id = 0;
    int ordem;
    public void runOpMode(){

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        sugador = hardwareMap.get(DcMotorEx.class, "par");
        esteira = hardwareMap.get(DcMotorEx.class, "perp");
        encoderseletor = hardwareMap.get(DcMotorEx.class, "encoderseletor");

        atirador1 = hardwareMap.get(DcMotorEx.class, "atirador1");
        atirador2 = hardwareMap.get(DcMotorEx.class, "atirador2");

        cremalheira = hardwareMap.get(Servo.class, "cremalheira");
        servoseletor = hardwareMap.get(Servo.class, "seletor");

        /*Atirador atirador = new Atirador(atirador1, atirador2);
        Intake intake = new Intake(sugador, esteira);*/
        seletor = new Seletor(armazenamento, encoderseletor, servoseletor, 5);


        Pose2d initialPose = new Pose2d(54.7, -51.1, Math.toRadians(135));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .strafeToSplineHeading(new Vector2d(24, -24), Math.toRadians(180))
                .waitSeconds(0.5);
        TrajectoryActionBuilder tab2 = drive.actionBuilder(initialPose)
                .strafeToSplineHeading(new Vector2d(12, -12), Math.toRadians(135))
                .waitSeconds(0.5);

        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.addData("Position during Init", "pos");
            telemetry.update();
        }


        waitForStart();


        if (isStopRequested()) return;

        limelight.setPollRateHz(90);
        //0 = identificação do motif pattern, 1 = identificação do goal vermelho, 2 = identificação do goal azul
        limelight.pipelineSwitch(0);
        limelight.start();

        Action trajectory1 = tab1.build();
        Action trajectory2 = tab2.build();

        Actions.runBlocking(new SequentialAction(
                trajectory1
        ));
        abc: for(int x = 0; x < 10; x++){
            LLResult result = limelight.getLatestResult();
            if (result.isValid()) {
                id = 0;
                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                for (LLResultTypes.FiducialResult fiducial : fiducials) {
                    id = fiducial.getFiducialId(); // The ID number of the fiducial
                    telemetry.addData(" tag id", id);
                    telemetry.update();
                }
                switch (id) {
                    case 21:
                        break abc;
                    case 22:
                        break abc;
                    case 23:
                        break abc;
                    default:
                        break;
                }
            }
        }
        Actions.runBlocking(
                new SequentialAction(
                        trajectory2
                )
        );
        switch (id){
            case 0:
                atirador1.setPower(-0.95);
                atirador2.setPower(0.95);
                sleep(1000);
                cremalheira.setPosition(1);
                sleep(650);
                cremalheira.setPosition(0);
                atirador1.setPower(0);
                atirador2.setPower(0);
                sleep(750);
                seletor.goToPosition(3);
                atirador1.setPower(-0.95);
                atirador2.setPower(0.95);
                sleep(1000);
                cremalheira.setPosition(1);
                sleep(450);
                cremalheira.setPosition(0);
                atirador1.setPower(0);
                atirador2.setPower(0);
                sleep(500);
                seletor.goToPosition(1);
                break;
            case 21:
                break;
            case 22:
                break;
            case 23:
                break;
            default:
                break;
        }






    }
}