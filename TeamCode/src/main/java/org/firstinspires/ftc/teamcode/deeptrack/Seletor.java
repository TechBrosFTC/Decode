package org.firstinspires.ftc.teamcode.deeptrack;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

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
