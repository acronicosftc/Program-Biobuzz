package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "teste2", group = "Autonomous")
class MecanumTrajetoriaComplexa extends LinearOpMode {

    // Motores do chassi
    private DcMotor frontLeft, backLeft, frontRight, backRight;

    // Resolução do sistema de odometria (ajuste conforme o hardware)
    private static final double TICKS_PER_INCH = 30.5;

    @Override
    public void runOpMode() {
        // 1. Inicialização do Hardware
        frontLeft  = hardwareMap.get(DcMotor.class, "esquerdo_f");
        backLeft   = hardwareMap.get(DcMotor.class, "direito_f");
        frontRight = hardwareMap.get(DcMotor.class, "esquerdo_t");
        backRight  = hardwareMap.get(DcMotor.class, "direito_t");

        // Inverter o lado esquerdo
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        resetEncoders();

        telemetry.addData("Status", "Pronto para Trajetória Complexa");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {

            // TRAJETÓRIA COMPLEXA:

            // 1. Curva Parabólica / S-Curve (Anda para frente enquanto faz deslocamento senoidal na lateral)
            executarSCurve(30.0, 15.0, 3.5); // Avança 30 polegadas em Y, oscila 15 polegadas em X no tempo de 3.5s
            sleep(200);

            // 2. Trajetória Diagonal com Rampa de Aceleração (Ramp-up/Ramp-down)
            // Move no vetor (X=20, Y=20) reduzindo derrapagens das rodas Mecanum
            moverDiagonalSuave(20.0, 20.0, 0.7);
            sleep(200);

            // 3. Retorno Rápido em Linha Reta Diagonal Oposta (X=-20, Y=-50)
            moverDiagonalSuave(-20.0, -50.0, 0.8);

            // Parar motores
            definirPotenciaHolonomica(0, 0, 0);
        }
    }

    /**
     * Executa uma trajetória em formato de curva S suave sem girar o robô.
     * Utiliza funções trigonométricas para calcular o vetor de velocidade instantâneo.
     */
    private void executarSCurve(double distYInches, double amplitudeXInches, double duracaoSegundos) {
        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        while (opModeIsActive() && timer.seconds() < duracaoSegundos) {
            double t = timer.seconds() / duracaoSegundos; // Progresso de 0.0 a 1.0

            // Velocidade constante em Y
            double vy = 0.5;

            // Velocidade senoidal em X para gerar o formato de S (Derivada da posição X)
            double vx = Math.cos(t * 2 * Math.PI) * (amplitudeXInches / distYInches);

            definirPotenciaHolonomica(vx, vy, 0);

            telemetry.addData("Trajetoria", "S-Curve em Execucao");
            telemetry.addData("Progresso", "%.1f%%", t * 100);
            telemetry.update();
        }
        definirPotenciaHolonomica(0, 0, 0);
    }

    /**
     * Movimento diagonal por controle de perfil de velocidade (Aceleração e Desaceleração Trapezoidal).
     */
    private void moverDiagonalSuave(double targetXInches, double targetYInches, double maxPower) {
        resetEncoders();

        int targetTicksY = (int) (targetYInches * TICKS_PER_INCH);
        int targetTicksX = (int) (targetXInches * TICKS_PER_INCH);
        double totalDistanceTicks = Math.hypot(targetTicksX, targetTicksY);

        if (totalDistanceTicks == 0) return;

        boolean emMovimento = true;

        while (opModeIsActive() && emMovimento) {
            // Posição média do chassi
            int currentY = (frontLeft.getCurrentPosition() + frontRight.getCurrentPosition()
                    + backLeft.getCurrentPosition() + backRight.getCurrentPosition()) / 4;
            int currentX = (frontLeft.getCurrentPosition() - frontRight.getCurrentPosition()
                    - backLeft.getCurrentPosition() + backRight.getCurrentPosition()) / 4;

            double currentDistanceTicks = Math.hypot(currentX, currentY);
            double progresso = currentDistanceTicks / totalDistanceTicks;

            // Perfil Trapezoidal de Potência (Acelera nos primeiros 20%, Desacelera nos últimos 20%)
            double perfilPotencia = maxPower;
            if (progresso < 0.2) {
                perfilPotencia = Math.max(0.15, maxPower * (progresso / 0.2)); // Ramp-up
            } else if (progresso > 0.8) {
                perfilPotencia = Math.max(0.15, maxPower * ((1.0 - progresso) / 0.2)); // Ramp-down
            }

            // Vetor de direção normalizado
            double vx = (targetXInches / Math.hypot(targetXInches, targetYInches)) * perfilPotencia;
            double vy = (targetYInches / Math.hypot(targetXInches, targetYInches)) * perfilPotencia;

            definirPotenciaHolonomica(vx, vy, 0);

            // Condição de parada
            if (progresso >= 0.98 || Math.abs(totalDistanceTicks - currentDistanceTicks) < 40) {
                emMovimento = false;
            }

            telemetry.addData("Progresso Diagonal", "%.1f%%", progresso * 100);
            telemetry.addData("Potencia Atual", perfilPotencia);
            telemetry.update();
        }

        definirPotenciaHolonomica(0, 0, 0);
    }

    /**
     * Cinemática Holonômica para acionamento das 4 rodas Mecanum
     */
    private void definirPotenciaHolonomica(double x, double y, double rx) {
        double powerFrontLeft  = y + x + rx;
        double powerBackLeft   = y - x + rx;
        double powerFrontRight = y - x - rx;
        double powerBackRight  = y + x - rx;

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
