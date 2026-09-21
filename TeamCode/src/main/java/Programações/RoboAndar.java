package Programações;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Andar Para Frente e Parar", group = "Exemplo")
public class RoboAndar extends LinearOpMode {

    // Declaração dos motores
    private DcMotor motorFrenteEsquerda = null;
    private DcMotor motorFrenteDireita = null;
    private DcMotor motorTrasEsquerda = null;
    private DcMotor motorTrasDireita = null;

    @Override
    public void runOpMode() {
        // 1. Inicialização dos motores (os nomes devem ser iguais aos configurados no Driver Station)
        motorFrenteEsquerda = hardwareMap.get(DcMotor.class, "left_front");
        motorFrenteDireita  = hardwareMap.get(DcMotor.class, "right_front");
        motorTrasEsquerda   = hardwareMap.get(DcMotor.class, "left_back");
        motorTrasDireita    = hardwareMap.get(DcMotor.class, "right_back");

        // 2. Configuração da direção dos motores (ajuste conforme a montagem do seu chassi)
        motorFrenteEsquerda.setDirection(DcMotor.Direction.REVERSE);
        motorTrasEsquerda.setDirection(DcMotor.Direction.REVERSE);
        motorFrenteDireita.setDirection(DcMotor.Direction.FORWARD);
        motorTrasDireita.setDirection(DcMotor.Direction.FORWARD);

        // Mensagem para a tela do Driver Station informando que o robô está pronto
        telemetry.addData("Status", "Pronto para iniciar");
        telemetry.update();

        // Espera o motorista pressionar o botão "PLAY" no Driver Station
        waitForStart();

        // ---------------------------------------------------------
        // AÇÕES DO ROBÔ APÓS O PLAY
        // ---------------------------------------------------------

        if (opModeIsActive()) {

            // Passo 1: Ligar os motores para andar para frente (potência de 50%)
            telemetry.addData("Status", "Andando para frente...");
            telemetry.update();

            motorFrenteEsquerda.setPower(0.5);
            motorFrenteDireita.setPower(0.5);
            motorTrasEsquerda.setPower(0.5);
            motorTrasDireita.setPower(0.5);

            // Passo 2: Manter o robô andando por 2000 milissegundos (2 segundos)
            sleep(2000);

            // Passo 3: Parar todos os motores
            telemetry.addData("Status", "Parando...");
            telemetry.update();

            motorFrenteEsquerda.setPower(0.0);
            motorFrenteDireita.setPower(0.0);
            motorTrasEsquerda.setPower(0.0);
            motorTrasDireita.setPower(0.0);

            // Pequena pausa para garantir a parada antes de encerrar o OpMode
            sleep(1000);
        }
    }
}