package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Mecanum Drive - FTC", group = "Linear OpMode")
public class MecanumTeleOp extends LinearOpMode {

    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;

    @Override
    public void runOpMode() {
        // Inicialização dos motores registrados na Driver Station
        frontLeft  = hardwareMap.get(DcMotor.class, "esquerdo_f");
        frontRight = hardwareMap.get(DcMotor.class, "direto_f");
        backLeft   = hardwareMap.get(DcMotor.class, "esquerdo_t");
        backRight  = hardwareMap.get(DcMotor.class, "direto_t");

        // Inverter o lado esquerdo (padrão FTC)
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {
            // Entradas do Gamepad (Analógico Esquerdo: Movimento / Analógico Direito: Rotação)
            double y = -gamepad1.left_stick_y; // Invertido porque o Y do controle é negativo para cima
            double x = gamepad1.left_stick_x * 1.1; // Multiplicador para corrigir a atrito do strafe lateral
            double rx = gamepad1.right_stick_x; // Rotação sobre o próprio eixo

            // Vetorização Mecanum
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1.0);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            // Envio de potência para os motores
            frontLeft.setPower(frontLeftPower);
            backLeft.setPower(backLeftPower);
            frontRight.setPower(frontRightPower);
            backRight.setPower(backRightPower);
        }
    }
}
