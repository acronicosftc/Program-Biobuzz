package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Autonomous(name = "CirculoPerfeitoEncoders")
public class CirculoPerfeitoEncoders extends LinearOpMode {

    private DcMotor motorFE, motorFD, motorTE, motorTD;
    private IMU imu;
    private ElapsedTime cronometro = new ElapsedTime();

    @Override
    public void runOpMode() {
        // Mapeamento dos motores
        motorFE = hardwareMap.get(DcMotor.class, "esquerdo_f");
        motorFD = hardwareMap.get(DcMotor.class, "direito_f");
        motorTE = hardwareMap.get(DcMotor.class, "esquerdo_t");
        motorTD = hardwareMap.get(DcMotor.class, "direito_t");

        // Direção
        motorFE.setDirection(DcMotor.Direction.REVERSE);
        motorTE.setDirection(DcMotor.Direction.REVERSE);
        motorFD.setDirection(DcMotor.Direction.FORWARD);
        motorTD.setDirection(DcMotor.Direction.FORWARD);

        // Resetar e preparar encoders
        motorFE.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFD.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorTE.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorTD.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFE.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorTE.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorTD.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Configuração do Giroscópio
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parametrosIMU = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );
        imu.initialize(parametrosIMU);
        imu.resetYaw();

        telemetry.addData("Status", "Encoders e Giroscópio prontos!");
        telemetry.update();

        waitForStart();
        cronometro.reset();

        // CONFIGURAÇÕES DO CÍRCULO PERFEITO
        double raioCulturaTicks = 1500;  // Raio do círculo em Ticks (Aumente/diminua para ajustar o tamanho da BOLA)
        double tempoVoltaSegundos = 10.0; // 10 segundos garante movimento suave sem derrapagem
        double fatorAtritoMecanum = 1.414; // Constante geométrica de 45° para rodas Mecanum (evita o formato de ovo)
        double kP_IMU = 0.04;            // Ganho de correção do ângulo para não virar o chassi

        while (opModeIsActive() && cronometro.seconds() < tempoVoltaSegundos) {

            // Ângulo theta do círculo em radianos (0 a 2*PI)
            double theta = (cronometro.seconds() / tempoVoltaSegundos) * (2 * Math.PI);

            // Velocidade escalar nos eixos teóricos (Derivada da posição circular)
            // Drive (Y) = cos(theta), Strafe (X) = sin(theta)
            double drive  = Math.cos(theta) * 0.4;
            double strafe = Math.sin(theta) * 0.4 * fatorAtritoMecanum;

            // Leitura da orientação do robô para correção
            double anguloAtual = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            double turn = -anguloAtual * kP_IMU; // Mantém em 0 graus

            // Cálculo exato de potência das Mecanum
            double pFE = drive + strafe + turn;
            double pFD = drive - strafe - turn;
            double pTE = drive - strafe + turn;
            double pTD = drive + strafe - turn;

            // Normalização de potência
            double max = Math.max(Math.abs(pFE), Math.max(Math.abs(pFD), Math.max(Math.abs(pTE), Math.abs(pTD))));
            if (max > 1.0) {
                pFE /= max; pFD /= max; pTE /= max; pTD /= max;
            }

            // Aplicar potência nos motores
            motorFE.setPower(pFE);
            motorFD.setPower(pFD);
            motorTE.setPower(pTE);
            motorTD.setPower(pTD);

            telemetry.addData("Progresso Círculo", "%.0f%%", (cronometro.seconds()/tempoVoltaSegundos)*100);
            telemetry.addData("Ângulo Trava", "%.1f°", anguloAtual);
            telemetry.update();
        }

        // Parar totalmente o robô ao completar o círculo
        motorFE.setPower(0);
        motorFD.setPower(0);
        motorTE.setPower(0);
        motorTD.setPower(0);
    }
}