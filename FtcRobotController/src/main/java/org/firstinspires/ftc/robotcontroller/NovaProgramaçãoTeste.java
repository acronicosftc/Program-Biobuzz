import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "Auto_Push_And_Strafe", group = "Autonomous")
public class AutoPushAndStrafe extends LinearOpMode {

    DcMotor esquerdo_f;
    DcMotor direito_f;
    DcMotor esquerdo_t;
    DcMotor direito_t;

    @Override
    public void runOpMode() {
        // Mapeamento dos motores
        esquerdo_f  = hardwareMap.get(DcMotor.class, "frontLeft");
        direito_f = hardwareMap.get(DcMotor.class, "frontRight");
        esquerdo_t   = hardwareMap.get(DcMotor.class, "backLeft");
        direito_t  = hardwareMap.get(DcMotor.class, "backRight");

        // Ajuste das direções para chassi Mecanum padrão
        esquerdo_f.setDirection(DcMotorSimple.Direction.REVERSE);
        esquerdo_t.setDirection(DcMotorSimple.Direction.REVERSE);
        direito_f.setDirection(DcMotorSimple.Direction.FORWARD);
        direito_t.setDirection(DcMotorSimple.Direction.FORWARD);

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

    /**
     * @return
     */
    @Override
    public boolean isStopRequested() {
        boolean b = false;
        return b;
    }

    // Método auxiliar para os 4 motores
    public void setDrivePower(double fl, double fr, double bl, double br) {
        esquerdo_f.setPower(fl);
        direito_f.setPower(fr);
        esquerdo_t.setPower(bl);
        direito_t.setPower(br);
    }
}
