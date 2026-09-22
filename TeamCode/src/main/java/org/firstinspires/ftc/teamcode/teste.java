package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "teste java")
 public class teste extends LinearOpMode {

    // Declaração dos motores
    private DcMotor motorEsquerdo;
    private DcMotor motorDireito;

    @Override
    public void runOpMode() {
        // Mapeamento dos motores conforme configurado no Robot Controller
        motorEsquerdo = hardwareMap.get(DcMotor.class, "esquerdo_f");
        motorDireito = hardwareMap.get(DcMotor.class, "direito_f");

        // Inverte a direção de um dos motores se eles estiverem montados em direções opostas
        motorEsquerdo.setDirection(DcMotor.Direction.REVERSE);
        motorDireito.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("Status", "Aguardando início...");
        telemetry.update();

        // Aguarda o botão PLAY ser pressionado no aplicativo
        waitForStart();

        if (opModeIsActive()) {
            telemetry.addData("Status", "Andando para a frente");
            telemetry.update();

            // Configura a potência dos dois motores para mover para a frente (potência de 50%)
            motorEsquerdo.setPower(0.5);
            motorDireito.setPower(0.5);

            // Mantém os motores ligados por 2 segundos (2000 milissegundos)
            sleep(2000);

            // Para os motores
            motorEsquerdo.setPower(0.0);
            motorDireito.setPower(0.0);

            telemetry.addData("Status", "Parado");
            telemetry.update();
        }
    }
}
