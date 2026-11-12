@Autonomous(name = "Auto_Push_And_Strafe", group = "Autonomous")
public class AutoPushAndStrafe extends LinearOpMode {

    DcMotor frontLeft;
    DcMotor frontRight;
    DcMotor backLeft;
    DcMotor backRight;

    @Override
    public void runOpMode() {
        // Mapeamento dos motores
        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft   = hardwareMap.get(DcMotor.class, "backLeft");
        backRight  = hardwareMap.get(DcMotor.class, "backRight");

        // Ajuste das direções para chassi Mecanum padrão
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);

        telemetry.addData("Status", "Pronto para iniciar!");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        // --- PASSO 1: Empurrar o elemento até a zona de pontuação ---
        setDrivePower(0.6, 0.6, 0.6, 0.6); // Anda para frente a 60% de velocidade
        sleep(2000);                      // Mantém por 2 segundos empurrando

        // --- PASSO 2: Recuar ligeiramente ---
        setDrivePower(-0.4, -0.4, -0.4, -0.4); // Anda para trás
        sleep(500);                            // Recua por 0.5 segundos para desvencilhar

        // --- PASSO 3: Movimento Lateral (Strafe para a Direita) ---
        // Em rodas Mecanum: Esquerda frente/trás vão para direções opostas
        setDrivePower(0.5, -0.5, -0.5, 0.5); // Move lateralmente para a direita
        sleep(1800);                         // Mantém até entrar na área de park

        // --- PASSO 4: Parar totalmente ---
        setDrivePower(0, 0, 0, 0);

        telemetry.addData("Status", "Pontuação e Park concluídos!");
        telemetry.update();
    }

    // Método auxiliar para os 4 motores
    public void setDrivePower(double fl, double fr, double bl, double br) {
        frontLeft.setPower(fl);
        frontRight.setPower(fr);
        backLeft.setPower(bl);
        backRight.setPower(br);
    }
}
