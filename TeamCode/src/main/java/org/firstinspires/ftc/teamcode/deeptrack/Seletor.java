package org.firstinspires.ftc.teamcode.deeptrack;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

public class Seletor {
    private Armazenamento armazenamento;
    private DcMotorEx encoder;
    private Servo servo; // Servo contínuo
    public int posicaoAtual = 0;
    public int numeroDeGiros = 0;
    double RAMP_LENGHT = 1365;
    double a = 0, b = 0, power = 0;
    double valoresEsperado = 0, alvoAbsoluto = 0;
    public int actual_intake_slot = 0;

    // >>> ADIÇÃO: variável para compensação do erro acumulado
    private double erroAcumulado = 0;

    public Seletor(Armazenamento armazenamento, Servo servo, DcMotorEx encoder) {
        this.armazenamento = armazenamento;
        this.servo = servo;
        this.encoder = encoder;
        this.encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.encoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        posicaoAtual = 0;
        numeroDeGiros = 0;
        this.servo.setDirection(Servo.Direction.REVERSE);
    }

    /** Função para girar o seletor com desaceleração linear + compensação de erro acumulado */
    /** Função para girar o seletor com desaceleração linear + correção de erro acumulado */
    /** Função para girar o seletor com desaceleração linear + correção de erro acumulado + microcorreção */
    public void girarseletor(double angulo) {
        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        encoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Compensação de inércia — se o seletor ultrapassa 10° depois de parar,
        // paramos 10° ANTES do ângulo desejado
        double anguloCompensado;
        if (angulo > 0) {
            anguloCompensado = angulo - 10;
        } else {
            anguloCompensado = angulo + 10;
        }

        // Converte ângulo para unidades do encoder
        double alvoEncoder = Math.abs(anguloCompensado * 22.5);

        if (angulo > 0) {
            double power = 0.57;
            while (Math.abs(encoder.getCurrentPosition()) < alvoEncoder) {
                // desacelera nos últimos 300 ticks
                if (alvoEncoder - Math.abs(encoder.getCurrentPosition()) < 300) {
                    power = 0.53;
                }
                servo.setPosition(power);
            }
        } else {
            double power = 0.43;
            while (Math.abs(encoder.getCurrentPosition()) < alvoEncoder) {
                if (alvoEncoder - Math.abs(encoder.getCurrentPosition()) < 300) {
                    power = 0.47;
                }
                servo.setPosition(power);
            }
        }

        // Para o servo
        servo.setPosition(0.5);
    }





    /** Move um slot vazio para a posição de coleta*/
    public int getSlotNaPosicaoColeta() {
        int slotNaColeta = (posicaoAtual + 3) % 3;
        return slotNaColeta;
    }

    /** Rotaciona a hélice RELATIVAMENTE em N posições úteis (positivo = horário). */
    public void rotacionarRelativo(int posicoes) {
        girarseletor(60 * posicoes);
        posicaoAtual = posicaoAtual + posicoes;
    }

    /** Move o seletor para uma posição de lançamento (0 a 2) */
    public void posicaoLancamento(int slot) {
        int[] posicaoPorSlot = {3, 1, 5};
        girarseletor(posicaoPorSlot[slot] * 60 - posicaoAtual * 60);
        posicaoAtual = posicaoPorSlot[slot];
    }

    /** Move o seletor para uma posição de coleta (0 a 2)
     * Retorna positivo se for horário, negativo se for anti-horário
     * */
    /** Move o seletor para uma posição de coleta (0 a 2)
     * Gira pelo caminho mais curto e retorna verdadeiro se horário, falso se anti-horário
     */
    public void posicaoColeta(int slot) {
        int[] posicaoPorSlot = {0, 4, 2};

        // Calcula posição alvo em graus
        double alvoDeg = posicaoPorSlot[slot] * 60;
        double atualDeg = posicaoAtual * 60;

        // Calcula diferença no menor caminho [-180, 180]
        double delta = (alvoDeg - atualDeg) % 360;
        if (delta > 180) delta -= 360;
        if (delta < -180) delta += 360;

        // Gira o seletor pelo delta mais curto
        girarseletor((int) delta);

        // Atualiza a posição atual
        posicaoAtual = posicaoPorSlot[slot];

        // Retorna o sentido: true = horário, false = anti-horário
    }


    /** Envia a bola verde mais próxima para a posição de lançamento */
    public void verde() {
        for (int i = 0; i < 3; i++) {
            if (armazenamento.getSlot(i) == Armazenamento.EstadoSlot.VERDE) {
                posicaoLancamento(i);
                break;
            }
        }
    }

    /** Envia a bola roxa mais próxima para a posição de lançamento */
    public void roxo() {
        for (int i = 0; i < 3; i++) {
            if (armazenamento.getSlot(i) == Armazenamento.EstadoSlot.ROXO) {
                posicaoLancamento(i);
                break;
            }
        }
    }
}
