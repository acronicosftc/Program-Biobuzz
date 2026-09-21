package Programações;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Config
@Autonomous(name = "Frente Azul - 2 Linhas", group = "Linear OpMode")
public class frenteAzul_2Linhas_antiga extends LinearOpMode {
    private ElapsedTime cicloTimer = new ElapsedTime();

    /// Arthur

    /// Magola
    public static double Kp_frente = -0.14, Ki_frente = -0.002, Kd_frente = -0.02;
    public static double Kp_lateral = 0.14, Ki_lateral = 0.002, Kd_lateral = 0.02;
    public static double Kp_giro = -0.04, Ki_giro = -0.0005, Kd_giro = -0.005;

    public static double testeX = 0;
    public static double testeY = 0;
    public static double testeA = 0;


    public static double TEMPO_AQUECIMENTO = 2.5;
    public static double TEMPO_TOTAL_LANCAR = 7.5;

    Hardware_do_robo robot;

    static final int HEADING_BUFFER = 5;
    double[] headingBuffer = new double[HEADING_BUFFER];
    int headingIndex = 0;
    boolean bufferCheio = false;

    @Override
    public void runOpMode() {

        telemetry = new MultipleTelemetry(
                telemetry,
                FtcDashboard.getInstance().getTelemetry()
        );

        robot = new Hardware_do_robo(hardwareMap);
        robot.pinpoint.resetPosAndIMU();

        for (int i = 0; i < 15; i++) {
            robot.pinpoint.update();
            addHeading(robot.getHeading());
            sleep(20);
        }

        robot.pinpoint.setPosition(
                new Pose2D(
                        DistanceUnit.CM,
                        -326.26,
                        67.34,
                        AngleUnit.DEGREES,
                        48.0
                )
        );

        waitForStart();
        if (!opModeIsActive()) return;

        headingIndex = 0;
        bufferCheio = false;
        for (int i = 0; i < HEADING_BUFFER; i++) {
            addHeading(48.0);
        }

        /// Vai até o ponto de lançamento e lança as bolas, se afastando do gol
        goToPoint(-234.52, 148, 48.78);
        sistemaLancamento();

        /// vai até a primeira linha do gol da frente e coleta
        goToPoint(-239.54, 130.57, 90);
        ligarcoleta(true);
        goToPointDevagar(-239.54, 52.69, 90,2.3);
        ligarcoleta(false);

        /// corrige a bola para o lançador não ficar preso
        ligarcoleta_correção(400);

        /// vai até a zona de lançamento e lança as bolas
        goToPoint(-234.52, 148, 50);
        sistemaLancamento();

        /// vai até a segunda linha e coleta as bolas para deixar 3 bolas para o Teleop
        goToPoint(-180.76, 130.57, 90);
        parar();
        ligarcoleta(true);
        goToPointDevagar(-180.76, 52.69, 90,2.3);

        ligarcoleta(false);
        ligarcoleta_correção(400);
        goToPoint(-234.52, 148, 50);



        parar();

        robot.pinpoint.update();
        PosiçãoGlobal.xFinal = robot.getX();
        PosiçãoGlobal.yFinal = robot.getY();
        PosiçãoGlobal.anguloFinal = robot.getHeading();
    }

    public void goToPoint(double xAlvo, double yAlvo, double anguloAlvoGraus) {
        // Variáveis PID + Integral
        double erroFrenteAnterior = 0, erroLateralAnterior = 0, erroAngAnterior = 0;
        double somaErroFrente = 0, somaErroLateral = 0, somaErroAng = 0;

        boolean primeiroCiclo = true;
        cicloTimer.reset();

        while (opModeIsActive()) {
            double dt = cicloTimer.seconds();
            if (dt < 0.001) dt = 0.001; // Evita divisão por zero
            cicloTimer.reset();

            robot.pinpoint.update();

            double xAtual = robot.getX();
            double yAtual = robot.getY();
            double angAtual = robot.getHeading(); // Pega direto do pinpoint
            double angRad = Math.toRadians(angAtual);

            double erroX = xAlvo - xAtual;
            double erroY = yAlvo - yAtual;
            double erroAng = wrapDeg(anguloAlvoGraus - angAtual);
            double distancia = Math.hypot(erroX, erroY);


            if (distancia < 5.0 && Math.abs(erroAng) < 3.5) {
                parar();
                break;
            }

            double erroFrente = erroX * Math.cos(angRad) + erroY * Math.sin(angRad);
            double erroLateral = -erroX * Math.sin(angRad) + erroY * Math.cos(angRad);

            // Limitamos a soma (anti-windup) para o robô não ficar louco
            somaErroFrente = com.qualcomm.robotcore.util.Range.clip(somaErroFrente + (erroFrente * dt), -15, 15);
            somaErroLateral = com.qualcomm.robotcore.util.Range.clip(somaErroLateral + (erroLateral * dt), -15, 15);
            somaErroAng = com.qualcomm.robotcore.util.Range.clip(somaErroAng + (erroAng * dt), -10, 10);

            double dFrente = 0, dLateral = 0, dAng = 0;
            if (!primeiroCiclo) {
                dFrente = (erroFrente - erroFrenteAnterior) / dt;
                dLateral = (erroLateral - erroLateralAnterior) / dt;
                dAng = (erroAng - erroAngAnterior) / dt;
            }

            double frente = (Kp_frente * erroFrente) + (Ki_frente * somaErroFrente) + (Kd_frente * dFrente);
            double lateral = (Kp_lateral * erroLateral) + (Ki_lateral * somaErroLateral) + (Kd_lateral * dLateral);
            double giro = (Kp_giro * erroAng) + (Ki_giro * somaErroAng) + (Kd_giro * dAng);

            erroFrenteAnterior = erroFrente;
            erroLateralAnterior = erroLateral;
            erroAngAnterior = erroAng;
            primeiroCiclo = false;

            double escalaDist = Math.min(1.0, distancia / 25.0);
            frente *= escalaDist;
            lateral *= escalaDist;


            if (Math.abs(erroAng) > 20) {
                frente *= 0.4;
                lateral *= 0.4;
            }

            frente  = com.qualcomm.robotcore.util.Range.clip(frente, -0.45, 0.45);
            lateral = com.qualcomm.robotcore.util.Range.clip(lateral, -0.45, 0.45);
            giro    = com.qualcomm.robotcore.util.Range.clip(giro, -0.35, 0.35);

            mover(lateral, frente, giro);

            telemetry.addData("Distancia", distancia);
            telemetry.addData("Erro Ang", erroAng);
            telemetry.update();

        }
    }

    public void goToPointDevagar(double xAlvo, double yAlvo, double anguloAlvoGraus, double timeout) {
        ElapsedTime tempoMovimento = new ElapsedTime();
        tempoMovimento.reset();

        double erroFrenteAnterior = 0, erroLateralAnterior = 0, erroAngAnterior = 0;
        double somaErroFrente = 0, somaErroLateral = 0, somaErroAng = 0;
        boolean primeiroCiclo = true;
        cicloTimer.reset();

        while (opModeIsActive()) {

            if (tempoMovimento.seconds() > timeout) {
                parar();
                break;
            }

            double dt = cicloTimer.seconds();
            if (dt < 0.001) dt = 0.001;
            cicloTimer.reset();

            robot.pinpoint.update();
            double xAtual = robot.getX();
            double yAtual = robot.getY();
            double angAtual = robot.getHeading();
            double angRad = Math.toRadians(angAtual);

            double erroX = xAlvo - xAtual;
            double erroY = yAlvo - yAtual;
            double erroAng = wrapDeg(anguloAlvoGraus - angAtual);
            double distancia = Math.hypot(erroX, erroY);

            if (distancia < 6.5 && Math.abs(erroAng) < 4.5) {
                parar();
                break;
            }

            double erroFrente = erroX * Math.cos(angRad) + erroY * Math.sin(angRad);
            double erroLateral = -erroX * Math.sin(angRad) + erroY * Math.cos(angRad);

            somaErroFrente = com.qualcomm.robotcore.util.Range.clip(somaErroFrente + (erroFrente * dt), -15, 15);
            somaErroLateral = com.qualcomm.robotcore.util.Range.clip(somaErroLateral + (erroLateral * dt), -15, 15);
            somaErroAng = com.qualcomm.robotcore.util.Range.clip(somaErroAng + (erroAng * dt), -10, 10);

            double dFrente = 0, dLateral = 0, dAng = 0;
            if (!primeiroCiclo) {
                dFrente = (erroFrente - erroFrenteAnterior) / dt;
                dLateral = (erroLateral - erroLateralAnterior) / dt;
                dAng = (erroAng - erroAngAnterior) / dt;
            }

            double frente = (Kp_frente * erroFrente) + (Ki_frente * somaErroFrente) + (Kd_frente * dFrente);
            double lateral = (Kp_lateral * erroLateral) + (Ki_lateral * somaErroLateral) + (Kd_lateral * dLateral);
            double giro = (Kp_giro * erroAng) + (Ki_giro * somaErroAng) + (Kd_giro * dAng);

            erroFrenteAnterior = erroFrente;
            erroLateralAnterior = erroLateral;
            erroAngAnterior = erroAng;
            primeiroCiclo = false;

            double escalaDist = com.qualcomm.robotcore.util.Range.clip(distancia / 20.0, 0.15, 1.0);
            frente *= escalaDist;
            lateral *= escalaDist;

            frente  = com.qualcomm.robotcore.util.Range.clip(frente, -0.45, 0.45);
            lateral = com.qualcomm.robotcore.util.Range.clip(lateral, -0.45, 0.45);
            giro    = com.qualcomm.robotcore.util.Range.clip(giro, -0.35, 0.35);

            mover(lateral * 0.6, frente * 0.6, giro * 0.6);

            telemetry.addData("Distancia", distancia);
            telemetry.addData("Tempo Restante", timeout - tempoMovimento.seconds());
            telemetry.update();
        }
    }

    public void girarPara(double anguloAlvo) {

        final double Kd = -0.004;
        final double MAX_GIRO = 0.35;

        double erroAnterior = 0;
        long start = System.currentTimeMillis();

        while (opModeIsActive()) {

            robot.pinpoint.update();
            addHeading(robot.getHeading());

            double bruto = robot.getHeading();
            double atual = getHeadingSuavizado();
            double erro = wrapDeg(anguloAlvo - atual);

            double derivada = erro - erroAnterior;
            erroAnterior = erro;

            double giro = erro * Kp_giro + derivada * Kd;
            giro = clip(giro, MAX_GIRO);

            if (Math.abs(erro) < 1.0 && Math.abs(derivada) < 0.2) {
                parar();
                break;
            }

            if (System.currentTimeMillis() - start > 4000) {
                parar();
                break;
            }

            mover(0, 0, giro);

            telemetry.addLine("GIRO");
            telemetry.addData("Alvo", anguloAlvo);
            telemetry.addData("Heading bruto", bruto);
            telemetry.addData("Heading suavizado", atual);
            telemetry.addData("Erro", erro);
            telemetry.addData("Derivada", derivada);
            telemetry.addData("Giro", giro);
            telemetry.update();

            sleep(20);
        }
    }

    public void sistemaLancamento() {

        ElapsedTime timer = new ElapsedTime();
        timer.reset();

        robot.lancador.setPower(0.77);
        robot.transportador.setPower(0);
        robot.pegador.setPower(0);

        while (opModeIsActive() && timer.seconds() < TEMPO_TOTAL_LANCAR) {

            parar();

            double t = timer.seconds();

            if (t >= TEMPO_AQUECIMENTO) {
                robot.transportador.setPower(0.34);
            }

            if (t >= TEMPO_AQUECIMENTO + 0.3) {
                robot.pegador.setPower(1.0);
            }

            telemetry.addLine("LANÇAMENTO ATIVO");
            telemetry.addData("Tempo", t);
            telemetry.addData("Lançador Power", robot.lancador.getPower());
            telemetry.addData("Transportador Power", robot.transportador.getPower());
            telemetry.addData("Pegador Power", robot.pegador.getPower());
            telemetry.update();

            sleep(20);
        }

        PararDeLançar();
        parar();
    }
    public void ligarcoleta(boolean ligar_desligar){

        if(ligar_desligar){

            robot.pegador.setPower(1);
            robot.transportador.setPower(0.4);

        } else {

            robot.pegador.setPower(0);
            robot.transportador.setPower(0);
        }


    }
    public void ligarcoleta_correção(int temp){


        robot.pegador.setPower(-0.5);
        robot.transportador.setPower(-0.4);
        sleep(temp);
        robot.pegador.setPower(0);
        robot.transportador.setPower(0);

    }



    public void PararDeLançar() {
        robot.lancador.setPower(0);
        robot.transportador.setPower(0);
        robot.pegador.setPower(0);
    }

    public void addHeading(double ang) {
        headingBuffer[headingIndex++] = ang;
        if (headingIndex >= HEADING_BUFFER) {
            headingIndex = 0;
            bufferCheio = true;
        }
    }

    public double getHeadingSuavizado() {
        int count = bufferCheio ? HEADING_BUFFER : headingIndex;
        double sin = 0, cos = 0;
        for (int i = 0; i < count; i++) {
            double r = Math.toRadians(headingBuffer[i]);
            sin += Math.sin(r);
            cos += Math.cos(r);
        }
        return Math.toDegrees(Math.atan2(sin, cos));
    }

    public void mover(double lat, double frente, double giro) {
        double df = frente - lat - giro;
        double dt = frente + lat - giro;
        double ef = frente + lat + giro;
        double et = frente - lat + giro;
        double max = Math.max(1, Math.max(Math.abs(df),
                Math.max(Math.abs(dt), Math.max(Math.abs(ef), Math.abs(et)))));
        robot.motor_direito_frente.setPower(df / max);
        robot.motor_direito_tras.setPower(dt / max);
        robot.motor_esquerdo_frente.setPower(ef / max);
        robot.motor_esquerdo_tras.setPower(et / max);
    }

    public void parar() {
        mover(0, 0, 0);
    }

    public double wrapDeg(double ang) {
        while (ang > 180) ang -= 360;
        while (ang < -180) ang += 360;
        return ang;
    }

    private double clip(double v, double max) {
        return Math.max(-max, Math.min(max, v));
    }

}