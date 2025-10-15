package org.firstinspires.ftc.teamcode.deeptrack;

// Classe para o armazenamento do robô
public class Armazenamento {
    public enum EstadoSlot {
        VAZIO, ROXO, VERDE, CHEIO
    }
    public EstadoSlot[] slots;

    public Armazenamento() {
        slots = new EstadoSlot[3];
        // Inicializa todos os slots como vazios
        for (int i = 0; i < slots.length; i++) {
            slots[i] = EstadoSlot.VAZIO;
        }
    }

    // Define o conteúdo de um slot
    public void setSlot(int indice, EstadoSlot estado) {
        if (indice < 0 || indice >= slots.length) {
            System.out.println("Índice inválido! Use de 0 a 2.");
            return;
        }
        slots[indice] = estado;
    }

    // Retorna o conteúdo de um slot específico
    public EstadoSlot getSlot(int indice) {
        if (indice < 0 || indice >= slots.length) {
            throw new IllegalArgumentException("Índice inválido! Use de 0 a 2.");
        }
        return slots[indice];
    }

    // Mostra o estado atual dos 3 slots
    public void mostrarEstado() {
        System.out.println("Estado atual do armazenamento:");
        for (int i = 0; i < slots.length; i++) {
            System.out.println("Slot " + i + ": " + slots[i]);
        }
    }
}
