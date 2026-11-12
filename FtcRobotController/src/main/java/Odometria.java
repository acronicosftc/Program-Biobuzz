package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Odometria", group = "Autonomous")
public class Odometria extends LinearOpMode {

    // Motores do Drivetrain (trem de força)
    private DcMotor esquerdo_f, direito_f, esquerdo_t, direito_t;

    // Constantes de conversão (ajuste conforme o motor e a roda do seu robô)
    static final double COUNTS_PER_REV = 537.7; // Ex: Motor REV HD Hex padrão
    static final double WHEEL_DIAMETER_INCHES = 3.77953; // 96mm em polegadas
    static final double COUNTS_PER_INCH = COUNTS_PER_REV / (WHEEL_DIAMETER_INCHES * Math.PI);

    // Constante aproximada para giros (baseada na largura do robô - ajuste se necessário)
    static final double TRACK_WIDTH_INCHES = 14.0;
    static final double COUNTS_PER_DEGREE = (TRACK_WIDTH_INCHES * Math.PI / 360.0) * COUNTS_PER_INCH;

    @Override
    public void runOpMode() {
        // Inicialização dos motores através do hardwareMap
        esquerdo_f = hardwareMap.get(DcMotor.class, "frontLeft");
        direito_f = hardwareMap.get(DcMotor.class, "frontRight");
        esquerdo_t = hardwareMap.get(DcMotor.class, "backLeft");
        direito_t = hardwareMap.get(DcMotor.class, "backRight");

        // Inverter os motores do lado esquerdo para o robô andar reto
        esquerdo_f.setDirection(DcMotor.Direction.REVERSE);
        esquerdo_t.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Status", "Programa de Trajeto em L Pronto!");
        telemetry.update();

        waitForStart();

        // --- PASSO 1: Andar para frente por 24 polegadas ---
        moverParaFrente(24.0, 0.5);
        sleep(500); // Pausa curta de meio segundo

        // --- PASSO 2: Virar para a direita em 90 graus ---
        girarDireita(90.0, 0.4);
        sleep(500);

        // --- PASSO 3: Andar para frente novamente por 12 polegadas ---
        moverParaFrente(12.0, 0.5);

        // Fim da rotina autônoma
        telemetry.addData("Status", "Missão Completa!");
        telemetry.update();
        sleep(2000);
    }

    /**
     * Função para mover o robô para frente ou para trás usando encoders
     */
    public void moverParaFrente(double polegadas, double potencia) {
        int targetTicks = (int) (polegadas * COUNTS_PER_INCH);

        resetarEncoders();

        esquerdo_f.setTargetPosition(targetTicks);
        direito_f.setTargetPosition(targetTicks);
        esquerdo_t.setTargetPosition(targetTicks);
        direito_t.setTargetPosition(targetTicks);

        mudarModoRunToPosition();

        esquerdo_f.setPower(potencia);
        direito_f.setPower(potencia);
        esquerdo_t.setPower(potencia);
        direito_t.setPower(potencia);

        while (opModeIsActive() && esquerdo_f.isBusy() && direito_f.isBusy()) {
            telemetry.addData("Ação", "Movendo para frente");
            telemetry.addData("Atual Esquerda", esquerdo_f.getCurrentPosition());
            telemetry.update();
        }

        pararMotores();
    }

    /**
     * Função para girar o robô para a direita usando encoders
     */
    public void girarDireita(double graus, double potencia) {
        int targetTicks = (int) (graus * COUNTS_PER_DEGREE);

        resetarEncoders();

        // Para girar no próprio eixo, os motores de um lado vão para frente e do outro para trás
        esquerdo_f.setTargetPosition(targetTicks);
        direito_f.setTargetPosition(-targetTicks);
        esquerdo_t.setTargetPosition(targetTicks);
        direito_t.setTargetPosition(-targetTicks);

        mudarModoRunToPosition();

        esquerdo_f.setPower(potencia);
        direito_f.setPower(potencia);
        esquerdo_t.setPower(potencia);
        direito_t.setPower(potencia);

        while (opModeIsActive() && esquerdo_f.isBusy() && direito_f.isBusy()) {
            telemetry.addData("Ação", "Girando para a direita");
            telemetry.update();
        }

        pararMotores();
    }

    /**
     * Função auxiliar para resetar os encoders de todos os motores
     */
    public void resetarEncoders() {
        esquerdo_f.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        direito_f.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        esquerdo_t.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        direito_t.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    /**
     * Função auxiliar para mudar os motores para o modo de alvo de posição
     */
    public void mudarModoRunToPosition() {
        esquerdo_f.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        direito_f.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        esquerdo_t.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        direito_t.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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
