package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="AutonomoEstruturadoExemplo", group="LinearOpMode")
class AutonomoEstruturadoExemplo extends LinearOpMode {

    // Declaração dos motores do chassi (Exemplo de tração traseira/dianteira ou Mecanum básico)
    private DcMotor esquerdo_f = null;
    private DcMotor direito_f = null;
    private DcMotor esquerdo_t = null;
    private DcMotor direito_t = null;

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {

        // 1. Mapeamento dos motores no HardwareMap
        esquerdo_f = hardwareMap.get(DcMotor.class, "frente_esq");
        direito_f  = hardwareMap.get(DcMotor.class, "frente_dir");
        esquerdo_t   = hardwareMap.get(DcMotor.class, "tras_esq");
        direito_t    = hardwareMap.get(DcMotor.class, "tras_dir");

        // 2. Correção de direção (geralmente um lado precisa ser invertido dependendo da montagem)
        esquerdo_f.setDirection(DcMotor.Direction.REVERSE);
        esquerdo_t.setDirection(DcMotor.Direction.REVERSE);
        direito_f.setDirection(DcMotor.Direction.FORWARD);
        direito_t.setDirection(DcMotor.Direction.FORWARD);

        // 3. Parada no Zero Power (Evita que o robô continue deslizando por inércia)
        esquerdo_f.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        direito_f.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        esquerdo_t.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        direito_t.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Pronto para iniciar");
        telemetry.update();

        // Aguarda o comando de início no Driver Station
        waitForStart();
        runtime.reset();

        if (opModeIsActive()) {
            // --- SEQUÊNCIA DO AUTÔNOMO ---

            // Passo 1: Ir para frente por 1.5 segundos a 60% de potência
            moverFrente(0.6, 1500);

            // Pausa breve de 0.5 segundos para estabilizar
            pararMotores();
            sleep(500);

            // Passo 2: Virar para a direita por 800 milissegundos
            virarDireita(0.5, 800);

            pararMotores();
            sleep(500);

            // Passo 3: Ir para trás por 1 segundo
            moverTras(0.5, 1000);

            // Fim do autônomo
            pararMotores();
            telemetry.addData("Status", "Autônomo Concluído com Sucesso!");
            telemetry.update();
        }
    }

    // ==========================================
    // FUNÇÕES AUXILIARES DE MOVIMENTO
    // ==========================================

    public void moverFrente(double potencia, long tempoMs) {
        esquerdo_f.setPower(potencia);
        direito_f.setPower(potencia);
        esquerdo_t.setPower(potencia);
        direito_t.setPower(potencia);
        sleep(tempoMs);
    }

    public void moverTras(double potencia, long tempoMs) {
        esquerdo_f.setPower(-potencia);
        direito_f.setPower(-potencia);
        esquerdo_t.setPower(-potencia);
        direito_t.setPower(-potencia);
        sleep(tempoMs);
    }

    public void virarDireita(double potencia, long tempoMs) {
        // Lado esquerdo vai para frente, lado direito vai para trás
        esquerdo_f.setPower(potencia);
        esquerdo_t.setPower(potencia);
        direito_f.setPower(-potencia);
        direito_t.setPower(-potencia);
        sleep(tempoMs);
    }

    public void pararMotores() {
        esquerdo_f.setPower(0);
        direito_f.setPower(0);
        esquerdo_t.setPower(0);
        direito_t.setPower(0);
    }
}
