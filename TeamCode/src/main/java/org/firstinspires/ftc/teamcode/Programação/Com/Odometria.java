package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Odometria", group = "Autonomous")
public class Odometria extends LinearOpMode {

    // Motores do Drivetrain (trem de força)
    private DcMotor esquerdo_f, direito_f, esquerdo_t , direito_t;

    // Constantes de conversão (ajuste conforme o motor e a roda do seu robô)
    static final double COUNTS_PER_REV = 537.7; // Ex: Motor REV HD Hex padrão
    static final double WHEEL_DIAMETER_INCHES = 3.77953; // 96mm em polegadas
    static final double COUNTS_PER_INCH = COUNTS_PER_REV / (WHEEL_DIAMETER_INCHES * Math.PI);

    @Override
    public void runOpMode() {
        // Inicialização dos motores através do hardwareMap
        // (Altere as strings entre aspas caso os nomes no seu Driver Station sejam diferentes)
        esquerdo_f = hardwareMap.get(DcMotor.class, "esquerdo_f");
        direito_f = hardwareMap.get(DcMotor.class, "direito_f");
        esquerdo_t = hardwareMap.get(DcMotor.class, "esquerdo_t");
        direito_t = hardwareMap.get(DcMotor.class, "direito_t");

        // Inverter os motores de um lado para o robô andar reto
        esquerdo_f.setDirection(DcMotor.Direction.REVERSE);
        esquerdo_t.setDirection(DcMotor.Direction.REVERSE);

        // Avisar que está pronto
        telemetry.addData("Status", "Programa Odometria Pronto!");
        telemetry.update();

        waitForStart();

        // 1. Andar para frente por 24 polegadas (cerca de 60 cm) com 50% de potência
        moverParaFrente(24.0, 0.5);

        // Pausa de 1 segundo
        sleep(1000);

        // 2. Parar totalmente o robô
        pararMotores();

        telemetry.addData("Status", "Missão Cumprida!");
        telemetry.update();
        sleep(2000);
    }

    /**
     * Função simples para mover o robô para frente usando encoders dos motores
     */
    public void moverParaFrente(double polegadas, double potencia) {
        // Calcular quantos "ticks" o encoder precisa contar
        int targetTicks = (int) (polegadas * COUNTS_PER_INCH);

        // Resetar os encoders dos motores e ativá-los no modo de contagem
        esquerdo_f.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        direito_f.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        esquerdo_t.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        direito_t.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Definir a posição alvo em ticks
        esquerdo_f.setTargetPosition(targetTicks);
        direito_f.setTargetPosition(targetTicks);
        esquerdo_t.setTargetPosition(targetTicks);
        direito_t.setTargetPosition(targetTicks);

        // Mudar o modo dos motores para rodar até a posição alvo
        esquerdo_f.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        direito_f.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        esquerdo_t.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        direito_t.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Ligar os motores com a potência desejada
        esquerdo_f.setPower(potencia);
        direito_f.setPower(potencia);
        esquerdo_t.setPower(potencia);
        direito_t.setPower(potencia);

        // Manter o programa rodando enquanto os motores estiverem se movendo
        while (opModeIsActive() && esquerdo_f.isBusy() && direito_f.isBusy()) {
            telemetry.addData("Alvo (ticks)", targetTicks);
            telemetry.addData("Atual Esquerda", esquerdo_f.getCurrentPosition());
            telemetry.update();
        }

        // Parar os motores ao atingir o alvo
        pararMotores();
    }

    /**
     * Função auxiliar para parar todos os motores
     */
    public void pararMotores() {
        esquerdo_f.setPower(0);
        direito_f.setPower(0);
        esquerdo_t.setPower(0);
        direito_t.setPower(0);
    }
}
