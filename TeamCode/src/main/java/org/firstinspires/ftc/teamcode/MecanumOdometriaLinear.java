package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "MecanumOdometriaLinear", group = "Autonomous")
public class MecanumOdometriaLinear extends LinearOpMode {

    // Declaração dos quatro motores do chassi
    private DcMotor frontLeft;
    private DcMotor backLeft;
    private DcMotor frontRight;
    private DcMotor backRight;

    // Fator de conversão: ajuste este valor (ticks por polegada ou cm) conforme a resolução dos seus encoders
    private static final double TICKS_PER_INCH = 30.5;

    @Override
    public void runOpMode() {
        // 1. Mapeamento dos motores no hardwareMap
        frontLeft  = hardwareMap.get(DcMotor.class, "esquerdo_f");
        backLeft   = hardwareMap.get(DcMotor.class, "esquerdo_t");
        frontRight = hardwareMap.get(DcMotor.class, "direito_f");
        backRight  = hardwareMap.get(DcMotor.class, "direito_t");

        // Inversão do lado esquerdo para alinhar a polaridade de tração para frente
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        // 2. Reset e configuração dos modos dos encoders
        resetEncoders();

        telemetry.addData("Status", "Inicializado com Sucesso");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            // SEQUÊNCIA DE MOVIMENTOS: (X [lateral], Y [frente/trás], Potência Mágix/Limite)

            // Passo A: Andar 24 polegadas para a FRENTE
            andarPorOdometria(0, 24, 0.5);
            sleep(250);

            // Passo B: Andar 24 polegadas para o LADO DIREITO (Strafe sem girar)
            andarPorOdometria(24, 0, 0.5);
            sleep(250);

            // Passo C: Andar 24 polegadas para TRÁS
            andarPorOdometria(0, -24, 0.5);
            sleep(250);

            // Parar todos os motores
            definirPotenciaHolonomica(0, 0, 0);
        }
    }

    /**
     * Calcula e atribui a potência holonômica para cada roda Mecanum.
     * @param x Movimento lateral (positivo = direita, negativo = esquerda)
     * @param y Movimento longitudinal (positivo = frente, negativo = trás)
     * @param rx Rotação no próprio eixo (mantido em 0 para não virar o robô)
     */
    private void definirPotenciaHolonomica(double x, double y, double rx) {
        double powerFrontLeft  = y + x + rx;
        double powerBackLeft   = y - x + rx;
        double powerFrontRight = y - x - rx;
        double powerBackRight  = y + x - rx;

        // Normalização das potências para manter a proporção entre 0.0 e 1.0
        double maxPower = Math.max(Math.abs(powerFrontLeft), Math.abs(powerFrontRight));
        maxPower = Math.max(maxPower, Math.abs(powerBackLeft));
        maxPower = Math.max(maxPower, Math.abs(powerBackRight));

        if (maxPower > 1.0) {
            powerFrontLeft  /= maxPower;
            powerBackLeft   /= maxPower;
            powerFrontRight /= maxPower;
            powerBackRight  /= maxPower;
        }

        frontLeft.setPower(powerFrontLeft);
        backLeft.setPower(powerBackLeft);
        frontRight.setPower(powerFrontRight);
        backRight.setPower(powerBackRight);
    }

    /**
     * Executa a movimentação com base na variação dos encoders.
     */
    private void andarPorOdometria(double targetXInches, double targetYInches, double power) {
        resetEncoders();

        // Conversão de polegadas para contagem de ticks target
        int targetTicksY = (int) (targetYInches * TICKS_PER_INCH);
        int targetTicksX = (int) (targetXInches * TICKS_PER_INCH);

        // Define a direção normalizada no vetor (x, y)
        double distance = Math.hypot(targetXInches, targetYInches);
        if (distance == 0) return;

        double normX = (targetXInches / distance) * power;
        double normY = (targetYInches / distance) * power;

        // Posição média do chassi
        while (opModeIsActive()) {
            int currentY = (frontLeft.getCurrentPosition() + frontRight.getCurrentPosition()
                    + backLeft.getCurrentPosition() + backRight.getCurrentPosition()) / 4;

            // Estimativa simples de deslocamento acumulado
            int currentX = (frontLeft.getCurrentPosition() - frontRight.getCurrentPosition()
                    - backLeft.getCurrentPosition() + backRight.getCurrentPosition()) / 4;

            // Condição de parada aproximada por margem de tolerância
            if (Math.abs(targetTicksY - currentY) < 30 && Math.abs(targetTicksX - currentX) < 30) {
                break;
            }

            definirPotenciaHolonomica(normX, normY, 0);

            telemetry.addData("Target Y", targetTicksY);
            telemetry.addData("Current Y", currentY);
            telemetry.update();
        }

        definirPotenciaHolonomica(0, 0, 0);
    }

    private void resetEncoders() {
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
