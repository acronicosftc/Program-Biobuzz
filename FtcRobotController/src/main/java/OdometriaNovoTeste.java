@Autonomous(name = "Auto_Basico_Park", group = "Autonomous")
public class AutoBasicoPark extends LinearOpMode {

    // Declaração dos motores do chassi (exemplo para chassi Mecanum ou Omni)
    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;

    @Override
    public void runOpMode() {
        // Inicialização dos motores no HardwareMap
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        // Ajuste de direção dos motores (inverta os da direita se o robô girar em vez de andar reto)
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addData("Status", "Pronto para o Autônomo!");
        telemetry.update();

        // Espera o sinal de início da partida (Start)
        waitForStart();

        if (isStopRequested()) return;

        // --- PASSO 1: Andar para frente ---
        // Define a potência para todos os motores (valor de 0.0 a 1.0)
        setDrivePower(0.5, 0.5, 0.5, 0.5);
        sleep(1500); // Mantém andando por 1.5 segundos (ajuste conforme a distância necessária)

        // --- PASSO 2: Parar o robô (Estacionamento) ---
        setDrivePower(0, 0, 0, 0);

        telemetry.addData("Status", "Autônomo Finalizado - Park Concluído!");
        telemetry.update();
    }

    // Método auxiliar para facilitar o controle dos motores
    public void setDrivePower(double fl, double fr, double bl, double br) {
        frontLeft.setPower(fl);
        frontRight.setPower(fr);
        backLeft.setPower(bl);
        backRight.setPower(br);
    }
}
