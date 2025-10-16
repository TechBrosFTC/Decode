package org.firstinspires.ftc.teamcode;

// RR-specific imports
import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
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
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.deeptrack.Armazenamento;
import org.firstinspires.ftc.teamcode.deeptrack.Seletor;
import java.util.List;

@Autonomous (name="Auto")
public class Auto extends LinearOpMode{
    DcMotorEx atirador1;
    DcMotorEx atirador2;
    public class Intake {
        private DcMotorEx sugador;
        private DcMotorEx esteira;
        public Intake(DcMotorEx sugador, DcMotorEx esteira){
            this.sugador = sugador;
            this.esteira = esteira;
            this.sugador.setDirection(DcMotorSimple.Direction.REVERSE);
            this.esteira.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        public void ativar(double power){
            sugador.setPower(power);
            esteira.setPower(power);
        }
        public void desativar(){
            sugador.setPower(0);
            esteira.setPower(0);
        }
    }
    public class Seletor {
        public Armazenamento armazenamento;
        private DcMotorEx encoder;
        private Servo servo;//1 = sentido horário, 0 = sentido antihorario
        int posicaoatual = 0;
        int numerodegiros = 0;
        double RAMP_LENGHT = 900;
        public double a = 0;
        public double b = 0;
        public double power = 0;
        public boolean inicial = true;
        public double valoresperado = 0;
        public double alvoabsoluto = 0;
        public Seletor(Armazenamento armazenamento, DcMotorEx encoder, Servo servo, int posicaoatual){
            this.armazenamento = armazenamento;
            this.encoder = encoder;
            this.servo = servo;
            this.servo.setDirection(Servo.Direction.REVERSE);
            this.encoder.setDirection(DcMotorSimple.Direction.REVERSE);
            this.posicaoatual = posicaoatual;
        }
        /**
         * @param posicaoalvo Posição alvo do seletor, de 0 a 5.
         * */
        public void goToPosition(int posicaoalvo){
            if(posicaoalvo == 0){
                switch (posicaoatual){
                    case 0:
                        break;
                    case 1:
                        girarseletor(-60);
                        posicaoatual = 1;
                        numerodegiros += 1;
                        break;
                    case 2:
                        girarseletor(-120);
                        posicaoatual = 2;
                        numerodegiros += 2;
                        break;
                    case 3:
                        girarseletor(180);
                        posicaoatual = 3;
                        numerodegiros += 3;
                        break;
                    case 4:
                        girarseletor(120);
                        posicaoatual = 4;
                        numerodegiros -= 2;
                        break;
                    case 5:
                        girarseletor(60);
                        posicaoatual = 5;
                        numerodegiros -= 1;
                        break;
                    default:
                        break;
                }
            }else{
                int diferença = (posicaoalvo - posicaoatual);
                if(Math.abs(diferença) >= 3){
                    switch (diferença){
                        case 3:
                            if(diferença > 0){
                                girarseletor(-180);
                                numerodegiros -= 3;
                            }else{
                                girarseletor(180);
                                numerodegiros += 3;
                            }
                            break;
                        case 4:
                            if(diferença>0){
                                girarseletor(-120);
                                numerodegiros -= 2;
                            }else{
                                girarseletor(120);
                                numerodegiros += 2;
                            }
                            break;
                        case 5:
                            if (diferença > 0){
                                girarseletor(-60);
                                numerodegiros --;
                            }else{
                                girarseletor(60);
                                numerodegiros ++;
                            }
                            break;
                        default:
                            break;
                    }
                }else{
                    girarseletor(diferença*60);
                    numerodegiros += diferença;
                }
                posicaoatual = posicaoalvo;
            }
        }
        /**Função para girar o seletor
         * @param  anguloabsoluto Em graus, positivo para sentido horário e negativo para sentido anti horário.
         * */
        public void girarseletor(int anguloabsoluto) {
            valoresperado = 1365.3*numerodegiros;
            alvoabsoluto = Math.abs((anguloabsoluto * 22.75))-(valoresperado - encoder.getCurrentPosition());
            if (anguloabsoluto > 0) {
                double posicaoinicialencoder = encoder.getCurrentPosition();
                double rampa = (RAMP_LENGHT)/Math.abs(alvoabsoluto);
                a = 0.06 / (alvoabsoluto*rampa);//parametros da função afim de desaceleração do seletor
                b = 0.4;
                while ((encoder.getCurrentPosition()) < (posicaoinicialencoder + (alvoabsoluto*(1-rampa)))) {
                    servo.setPosition(1);
                }
                while ((encoder.getCurrentPosition()) < (posicaoinicialencoder + alvoabsoluto)) {
                    power = a * (Math.abs(encoder.getCurrentPosition()) - posicaoinicialencoder - (alvoabsoluto* (1-rampa))) + b;
                    if (power < 0.54) {
                        power = 0.54;
                    }
                    servo.setPosition(power);
                }
                servo.setPosition(0.5);
            } else {
                double posicaoinicialencoder = encoder.getCurrentPosition();
                double rampa = (RAMP_LENGHT)/Math.abs(alvoabsoluto);
                a = -0.06 / (Math.abs(alvoabsoluto*rampa)); // parametros da função afim de desaceleração do seletor
                b = 0.6;
                while ((encoder.getCurrentPosition()) > (posicaoinicialencoder - (alvoabsoluto*(1-rampa)))){
                    servo.setPosition(0);
                }
                while ((encoder.getCurrentPosition()) > (posicaoinicialencoder - alvoabsoluto)){
                /*power = a * (Math.abs(encoder.getCurrentPosition())-posicaoinicialencoder - (alvoabsoluto* (1-rampa))) + b;
                if (power > 0.46) {
                    power = 0.46;
                }*/
                    servo.setPosition(0.46);
                }
                servo.setPosition(0.5);
            }
        }

        /**Função para alocar um slot específico para posição de abastecimento.
         * @param slot Slot do seletor que você deseja alocar para a posição de lançamento. De 0 a 2
         * */
        public void posicaolancamento(int slot){
            int[] posicao_por_slot = {3, 1, 5};
            goToPosition(posicao_por_slot[slot]);
        }
        /** Função para alocar um slot específico para posição de abastecimento.
         * @param slot Slot do seletor que você deseja alocar para a posição de lançamento. De 0 a 2
         * */
        public void posicaocoleta(int slot){
            int[] posicao_por_slot = {0, 4, 2};
            goToPosition(posicao_por_slot[slot]);
        }
    }
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
    Intake intake;
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
        seletor = new Seletor(armazenamento, encoderseletor, servoseletor, 5);

        atirador1 = hardwareMap.get(DcMotorEx.class, "atirador1");
        atirador2 = hardwareMap.get(DcMotorEx.class, "atirador2");
        RobotSetup peach = new RobotSetup(hardwareMap, esquerdaFrente, esquerdaTras, direitaFrente, direitaTras, sugador, esteira);


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
        abc:
        for (int x = 0; x < 10; x++) {
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
        limelight.pipelineSwitch(1);
        LLResult result = limelight.getLatestResult();
        for (int i = 0; i < 10; i++) {
            if (result.isValid()) {
                double x = 0;
                double y = 0;
                int id;
                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                for (LLResultTypes.FiducialResult fiducial : fiducials) {
                    id = fiducial.getFiducialId(); // The ID number of the fiducial
                    x = fiducial.getTargetXDegrees() - 2; // Where it is (left-right)
                    y = fiducial.getTargetYDegrees(); // Where it is (up-down)
                    telemetry.addData("x", x);
                    telemetry.addData("y", y);
                    telemetry.update();
                }
                x -= -3;
                if (Math.abs(x) > 1) {
                    if (x > 0) {
                        peach.mecanumDrive.curve(-x, 0.25, 0.25);
                    } else {
                        peach.mecanumDrive.curve(-x, 0.25, 0.25);
                    }
                }
            /*Pose2d actual = new Pose2d(new Vector2d(12, -12), Math.toRadians(135+Math.round(x)));
            drive = new MecanumDrive(hardwareMap, actual);*/
            }
        }

        atirador1.setPower(-0.8);
        atirador2.setPower(0.8);
        while (atirador2.getVelocity() < 1500) {
        }
        cremalheira.setPosition(1);
        while (atirador2.getVelocity() > 1400) {
        }
        cremalheira.setPosition(0);
        sleep(700);
        cremalheira.setPosition(0.5);
        atirador1.setPower(-0.8);
        atirador2.setPower(0.8);
        while (atirador2.getVelocity() < 1500) {
        }
        cremalheira.setPosition(1);
        while (atirador2.getVelocity() > 1400) {
        }
        cremalheira.setPosition(0);
        sleep(700);
        cremalheira.setPosition(0.5);






    }
}