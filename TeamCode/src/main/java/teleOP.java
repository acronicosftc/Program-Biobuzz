package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "TeleOp FTC - Dois Controles", group = "Linear Opmode")
class TeleOpDoisControles extends LinearOpMode {

    // Declaração dos 4 motores do robô
    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;

    @Override
    public void runOpMode() {

        // Mapeamento dos motores com os nomes definidos no Driver Station
        frontLeft  = hardwareMap.get(DcMotor.class, "esquerdo_f");
        frontRight = hardwareMap.get(DcMotor.class, "direito_f");
        backLeft   = hardwareMap.get(DcMotor.class, "esquerdo_t");
        backRight  = hardwareMap.get(DcMotor.class, "direito_t");

        // Inverte os motores do lado direito para que todos andem para frente com valores positivos
        frontLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        // Define os motores para frear quando a potência for zero
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Robô Inicializado");
        telemetry.update();

        // Aguarda o botão START ser pressionado no Driver Station
        waitForStart();

        while (opModeIsActive()) {

            // --- GAMEPAD 1: Translação (Mover para Frente/Trás e Lados) ---
            // Leitura e aplicação de velocidade máxima (1.0 ou -1.0) para os movimentos
            double drive = 0.0;
            if (Math.abs(gamepad1.left_stick_y) > 0.1) {
                drive = Math.signum(-gamepad1.left_stick_y); // -1.0 ou 1.0 (Invertido no Java)
            }

            double strafe = 0.0;
            if (Math.abs(gamepad1.left_stick_x) > 0.1) {
                strafe = Math.signum(gamepad1.left_stick_x); // -1.0 ou 1.0
            }

            // --- GAMEPAD 2: Rotação (Girar no próprio eixo) ---
            double turn = 0.0;
            if (Math.abs(gamepad2.right_stick_x) > 0.1) {
                turn = Math.signum(gamepad2.right_stick_x); // -1.0 ou 1.0
            }

            // Cálculo das potências dos motores para rodas Mecanum
            double frontLeftPower  = drive + strafe + turn;
            double frontRightPower = drive - strafe - turn;
            double backLeftPower   = drive - strafe + turn;
            double backRightPower  = drive + strafe - turn;

            // Normalização: garante que a potência limite seja mantida no intervalo de [-1.0, 1.0]
            double maxPower = Math.max(
                    Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                    Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))
            );

            if (maxPower > 1.0) {
                frontLeftPower  /= maxPower;
                frontRightPower /= maxPower;
                backLeftPower   /= maxPower;
                backRightPower  /= maxPower;
            }

            // Aplica a potência calibrada em cada motor
            frontLeft.setPower(frontLeftPower);
            frontRight.setPower(frontRightPower);
            backLeft.setPower(backLeftPower);
            backRight.setPower(backRightPower);

            // Exibe informações em tempo real na tela da Driver Station
            telemetry.addData("Gamepad 1 - Frente/Trás", drive);
            telemetry.addData("Gamepad 1 - Lateral", strafe);
            telemetry.addData("Gamepad 2 - Giro", turn);
            telemetry.update();
        }
    }
}
