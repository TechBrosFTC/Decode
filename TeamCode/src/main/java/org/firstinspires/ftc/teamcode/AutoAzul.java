package org.firstinspires.ftc.teamcode;

// RR-specific imports

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.deeptrack.Armazenamento;
import org.firstinspires.ftc.teamcode.deeptrack.Seletor;

import java.util.List;

@Autonomous (name="autoazul")
public class AutoAzul extends LinearOpMode{
    DcMotorEx atirador1;
    DcMotorEx atirador2;
    Shooter shooter;

    Intake intake;


        /**Função para alocar um slot específico para posição de abastecimento.

    /*public class Atirador{

        Servo cremalheira;
        public Atirador(HardwareMap hardwareMap) {
            atirador1 = hardwareMap.get(DcMotorEx.class, "atirador1");
            atirador2 = hardwareMap.get(DcMotorEx.class, "atirador2");
            cremalheira = hardwareMap.get(Servo.class, "cremalheira");
        }

        public class Acelerar implements Action{
            double velocity;
            public Acelerar(double velocity){
                this.velocity = velocity;
            }
            private boolean initialized = false;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {

                if (!initialized) {
                    atirador1.setPower(-0.95);
                    atirador2.setPower(0.95);
                    initialized = true;
                }
                double vel = Math.abs((atirador1.getVelocity())+Math.abs(atirador2.getVelocity()))/2;
                return vel >= velocity;
            }
            public Action acelerar(double velocity) {
                return new Acelerar(velocity);
            }
        }
        public class Parar implements Action{
            private boolean initialized = false;
            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!initialized){
                    atirador1.setPower(0);
                    atirador2.setPower(0);
                }
                return true;
            }
            public Action parar(){
                return new Parar();
            }
        }

    }*/
    private DcMotorEx direitaFrente, direitaTras, esquerdaFrente, esquerdaTras;//instancia dos motores de movimento
    private DcMotorEx sugador, esteira, encoderseletor;//instância dos motores do intake
    private Servo cremalheira, servoseletor;//
    Armazenamento armazenamento = new Armazenamento();
    Seletor seletor;
    Limelight3A limelight;
    int id = 0;
    int ordem;
    public void runOpMode() {
        direitaFrente = hardwareMap.get(DcMotorEx.class, "rightFront");
        direitaTras = hardwareMap.get(DcMotorEx.class, "rightBack");
        esquerdaFrente = hardwareMap.get(DcMotorEx.class, "encoderseletor");
        esquerdaTras = hardwareMap.get(DcMotorEx.class, "leftBack");

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        sugador = hardwareMap.get(DcMotorEx.class, "par");
        esteira = hardwareMap.get(DcMotorEx.class, "perp");
        encoderseletor = hardwareMap.get(DcMotorEx.class, "encoderseletor");

        direitaFrente.setDirection(DcMotorSimple.Direction.REVERSE);
        direitaTras.setDirection(DcMotorSimple.Direction.REVERSE);
        esquerdaFrente.setDirection(DcMotorSimple.Direction.REVERSE);

        cremalheira = hardwareMap.get(Servo.class, "cremalheira");
        servoseletor = hardwareMap.get(Servo.class, "seletor");

        /*Atirador atirador = new Atirador(atirador1, atirador2);
        Intake intake = new Intake(sugador, esteira);*/
        seletor = new Seletor(armazenamento, servoseletor, encoderseletor);

        atirador1 = hardwareMap.get(DcMotorEx.class, "atirador1");
        atirador2 = hardwareMap.get(DcMotorEx.class, "atirador2");
        RobotSetup peach = new RobotSetup(hardwareMap, esquerdaFrente, esquerdaTras, direitaFrente, direitaTras, sugador, esteira);
        shooter = new Shooter(atirador1, atirador2);
        intake = new Intake(sugador, esteira);
        Pose2d initialPose = new Pose2d(54.7, -51.1, Math.toRadians(135));
        MecanumDrive drive = new MecanumDrive(hardwareMap, initialPose);

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .strafeToSplineHeading(new Vector2d(-12, 0), Math.toRadians(155))
                .waitSeconds(0.5);

        TrajectoryActionBuilder tab2 = drive.actionBuilder(new Pose2d(new Vector2d(-12, 0), Math.toRadians(155)))
                .turn(Math.toRadians(53))
                .waitSeconds(0.5);
        TrajectoryActionBuilder tab3 = drive.actionBuilder(new Pose2d(new Vector2d(-12, 0), Math.toDegrees(125)))
                .strafeToSplineHeading(new Vector2d(-68, -68), Math.toRadians(205));

        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.addData("Position during Init", "pos");
            telemetry.update();
        }


        waitForStart();
        seletor.posicaoAtual = 1;
        armazenamento.setSlot(0, Armazenamento.EstadoSlot.ROXO);
        armazenamento.setSlot(1, Armazenamento.EstadoSlot.ROXO);
        armazenamento.setSlot(2, Armazenamento.EstadoSlot.VERDE);

        if (isStopRequested()) return;

        limelight.setPollRateHz(90);
        //0 = identificação do motif pattern, 1 = identificação do goal vermelho, 2 = identificação do goal azul
        limelight.pipelineSwitch(0);
        limelight.start();

        Action trajectory1 = tab1.build();
        Action trajectory2 = tab2.build();
        Action trajectory3 = tab3.build();





        Actions.runBlocking(new SequentialAction(
                trajectory1
        ));
        LLResult result = limelight.getLatestResult();
        if (result.isValid()) {
            id = 0;
            List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
            for (LLResultTypes.FiducialResult fiducial : fiducials) {
                id = fiducial.getFiducialId(); // The ID number of the fiducial
            }
        }
        telemetry.addData(" tag id", id);
        telemetry.update();
        Actions.runBlocking(
                new SequentialAction(
                        trajectory2
                )
        );
        sleep(400);

        switch (id){
            case 21:
                intake.sugar(1, -1);
                girarseletor(-53);
                sleep(400);
                girarseletor(-50);
                intake.stop();
                atirar();
                armazenamento.setSlot(2, Armazenamento.EstadoSlot.VAZIO);
                intake.sugar(1, -1);
                sleep(600);
                seletor = new Seletor(armazenamento, servoseletor, encoderseletor);
                sleep(200);
                girarseletor(55);
                sleep(400);
                girarseletor(50);
                intake.stop();
                atirar();
                armazenamento.setSlot(0, Armazenamento.EstadoSlot.VAZIO);
                intake.sugar(1, -1);
                sleep(600);
                sleep(100);
                seletor = new Seletor(armazenamento, servoseletor, encoderseletor);
                sleep(200);
                girarseletor(60);
                sleep(400);
                girarseletor(50);
                intake.stop();
                atirar();
                break;
            case 22:
                atirar();
                sleep(200);
                intake.sugar(1, -1);
                sleep(500);
                girarseletor(-50);
                sleep(500);
                girarseletor(-50);
                sleep(200);
                intake.stop();
                atirar();
                intake.sugar(1, -1);
                sleep(500);
                girarseletor(-50);
                sleep(500);
                girarseletor(-48);
                sleep(200);
                intake.stop();
                atirar();
                break;
            case 23:
                atirar();
                sleep(200);
                intake.sugar(1, -1);
                sleep(500);
                girarseletor(55);
                sleep(500);
                girarseletor(50);
                sleep(200);
                intake.stop();
                atirar();
                intake.sugar(1, -1);
                sleep(500);
                girarseletor(55);
                sleep(500);
                girarseletor(50);
                sleep(200);
                intake.stop();
                atirar();
                break;
            default:
                atirar();
                sleep(200);
                intake.sugar(1, -1);
                sleep(500);
                girarseletor(55);
                sleep(500);
                girarseletor(50);
                sleep(200);
                intake.stop();
                atirar();
                intake.sugar(1, -1);
                sleep(500);
                girarseletor(50);
                sleep(500);
                girarseletor(55);
                sleep(200);
                intake.stop();
                atirar();
                break;
        }
        Actions.runBlocking(
                new SequentialAction(
                        trajectory3
                )
        );






    }
    public void atirar(){
        Thread acel = new Thread(() -> {
            while (opModeIsActive()) {
                shooter.acelerarAtirador(2000);
            }
        });
        acel.start();

        while (shooter.getVel() < 2000) { }
        cremalheira.setPosition(1);

        while (atirador2.getVelocity() > 1600) { }
        cremalheira.setPosition(0);

        sleep(700);
        acel.interrupt();
        cremalheira.setPosition(0.5);
    }
    public void girarseletor(double angulo){
        encoderseletor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoderseletor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if(angulo > 0){
            double power = 1;
            while (Math.abs(encoderseletor.getCurrentPosition()) < angulo * 22.5){
                if((Math.abs(angulo)*22.5) - Math.abs(encoderseletor.getCurrentPosition()) < 300){
                    power = 0.53;
                }
                servoseletor.setPosition(power);
            }
        }else{
            double power = 0;
            while (Math.abs(encoderseletor.getCurrentPosition()) < Math.abs(angulo) * 22.5){
                if((Math.abs(angulo)*22.5) - Math.abs(encoderseletor.getCurrentPosition()) < 300){
                    power = 0.47;
                }
                servoseletor.setPosition(power);
            }
        }
        servoseletor.setPosition(0.5);;
    }
}