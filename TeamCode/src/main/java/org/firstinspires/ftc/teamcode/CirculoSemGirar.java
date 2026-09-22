package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "CirculoSemGirar")
public class CirculoSemGirar extends LinearOpMode {

    // Declaração dos 4 motores para tração Mecanum
    private DcMotor motorFrenteEsquerda;
    private DcMotor motorFrenteDireita;
    private DcMotor motorTrasEsquerda;
    private DcMotor motorTrasDireita;

    // Cronômetro para controlar o movimento suave em função do tempo
    private ElapsedTime cronometro = new ElapsedTime();

    @Override
    public void runOpMode() {
        // 1. Mapeamento dos motores conforme a configuração do Robot Controller
        motorFrenteEsquerda = hardwareMap.get(DcMotor.class, "esquerdo_f");
        motorFrenteDireita  = hardwareMap.get(DcMotor.class, "direito_f");
        motorTrasEsquerda   = hardwareMap.get(DcMotor.class, "esquerdo_t");
        motorTrasDireita    = hardwareMap.get(DcMotor.class, "direito_t");

        // 2. Inversão dos motores do lado esquerdo (necessário em chassi Mecanum)
        motorFrenteEsquerda.setDirection(DcMotor.Direction.REVERSE);
        motorTrasEsquerda.setDirection(DcMotor.Direction.REVERSE);
        motorFrenteDireita.setDirection(DcMotor.Direction.FORWARD);
        motorTrasDireita.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("Status", "Aguardando início...");
        telemetry.update();

        waitForStart();
        cronometro.reset();

        // Configuração do círculo
        double tempoParaVoltaCompleta = 6.0; // Tempo em segundos para dar 1 volta circular
        double potenciaMaxima = 0.5;          // Velocidade do robô (0.0 a 1.0)

        while (opModeIsActive() && cronometro.seconds() < tempoParaVoltaCompleta) {

            // Calcula o progresso do círculo em radianos (de 0 a 2*PI)
            double angulo = (cronometro.seconds() / tempoParaVoltaCompleta) * (2 * Math.PI);

            // Decomposição vetorial do movimento em Círculo:
            // drive  = movimento frente/trás (Eixo Y)
            // strafe = movimento lateral (Eixo X)
            double drive  = Math.sin(angulo) * potenciaMaxima;
            double strafe = Math.cos(angulo) * potenciaMaxima;

            // Cinemática Mecanum para translação pura (sem rotação/turn = 0)
            double potenciaFE = drive + strafe;
            double potenciaFD = drive - strafe;
            double potenciaTE = drive - strafe;
            double potenciaTD = drive + strafe;

            // Normalização de potências caso ultrapassem 1.0
            double max = Math.max(Math.abs(potenciaFE),
                    Math.max(Math.abs(potenciaFD),
                            Math.max(Math.abs(potenciaTE), Math.abs(potenciaTD))));

            if (max > 1.0) {
                potenciaFE /= max;
                potenciaFD /= max;
                potenciaTE /= max;
                potenciaTD /= max;
            }

            // Aplicação da potência em cada motor
            motorFrenteEsquerda.setPower(potenciaFE);
            motorFrenteDireita.setPower(potenciaFD);
            motorTrasEsquerda.setPower(potenciaTE);
            motorTrasDireita.setPower(potenciaTD);

            // Telemetria para acompanhamento no Driver Station
            telemetry.addData("Status", "Desenhando círculo...");
            telemetry.addData("Tempo (s)", "%.2f", cronometro.seconds());
            telemetry.addData("Drive (Y)", "%.2f", drive);
            telemetry.addData("Strafe (X)", "%.2f", strafe);
            telemetry.update();
        }

        // Parar todos os motores ao concluir a volta
        motorFrenteEsquerda.setPower(0);
        motorFrenteDireita.setPower(0);
        motorTrasEsquerda.setPower(0);
        motorTrasDireita.setPower(0);

        telemetry.addData("Status", "Círculo concluído e robô parado!");
        telemetry.update();
    }
}