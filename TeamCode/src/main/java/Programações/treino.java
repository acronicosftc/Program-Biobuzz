package Programações;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;


@TeleOp(name="TeleOp de Treino", group="Linear OpMode")
public class treino extends LinearOpMode {

    // Adicione esta variável no topo da sua classe
    private ElapsedTime cicloTimer = new ElapsedTime();
    private ElapsedTime tempoLancador = new ElapsedTime();


    private Telemetry dashboardTelemetry;


    //limelight?
    //Modos
    private boolean Modo_automático = false;
    private boolean Time_respectivo = true;
    public double calculoDinamico = 0;

    //Coeficientes de PID
    //Kp
    public static double Kp_frente = -0.009;
    public static double Kp_lateral = 0.013;
    public static double Kp_giro = -2;
    double dt = 0.01;

    public static double h1 = 35.56; // Altura da câmera no robô (em cm)
    public static double h2 = 73; // Altura da AprilTag no campo (em cm) - Verifique o manual do Decode
    public static double angulo_montagem = 10; // Ângulo que a câmera está inclinada para cima (em graus)

    //Ki
    public static double Ki_frente = 0;
    public static double Ki_lateral = 0;
    public static double Ki_giro = 0;


    //Kd
    public static double Kd_frente = 0;
    public static double Kd_lateral = 0;
    public static double Kd_giro = 0;


    //Variáveis do calculo de PID
    public double Erro_acumulado_lateral = 0;
    private double Erro_ultimo_lateral = 0;
//

    public double Erro_acumulado_frente = 0;
    private double Erro_ultimo_frente = 0;


    public double Erro_acumulado_giro = 0;
    private double Erro_ultimo_giro = 0;


    public boolean EstadoAtualDoGamepad = false;
    public boolean EstadoAnteriorDoGamepad = false;
    public boolean Estado = false;
    public boolean Estado_pegador = false;
    public boolean Estado_transportador = false;
    public boolean Estado_lançador_limelight = true;

    public boolean Estado_gamepad2_a = false;
    public boolean Estado_gamepad2_a_anterior = false;
    public boolean Estado_gamepad2_x = false;
    public boolean Estado_gamepad2_x_anterior = false;
    public boolean Estado_gamepad2_y = false;
    public boolean Estado_gamepad2_y_anterior = false;
    public boolean Estado_gamepad1_x = false;
    public boolean Estado_gamepad1_x_anterior = false;
    public boolean Estado_gamepad1_b = false;
    public boolean Estado_gamepad1_b_anterior = false;


    public boolean Estado_padUp = false;
    public boolean Estado_padDown = false;
    public boolean Estado_padUp1 = false;
    public boolean Estado_padDown1 = false;
    public boolean Estado_padLeft = false;
    public boolean Estado_padRight = false;
    public boolean Estado_Bumper = false;
    public boolean Estado_padLeft_anterior = false;
    public boolean Estado_padRight_anterior = false;
    public boolean Estado_padDown_anterior = false;
    public boolean Estado_padUp_anterior = false;
    public boolean Estado_padDown_anterior1 = false;
    public boolean Estado_padUp_anterior1 = false;
    public boolean Estado_Bumper_anterior = false;


    public double Pot_max = 1;
    public double Pot_min = 0;
    public double Potencia_parado = 0;
    public static double Potencia_ligado_lancador = 1;
    public static double Potencia_ligado_pegador = 0.6;
    public static double Potencia_ligado_transportador = 0.5;
    public double Potencia = 0;
    public double Potencia_pegador = 0;
    public double Potencia_transportador = 0;


    public static double X_alvo = 0;
    public static double Y_alvo = 0;
    public static double Angulo_alvo = 0;
    public double Angulo_alvo_em_rad = 0;

    public static double Margem_de_erro = 2;
    public static double Margem_do_angulo = 1;
    public double Margem_de_erro_angulo_em_rad = Margem_do_angulo * Math.PI / 180;


    double X_inicial = 0.0;
    double Y_inicial = 0.0;

    double Y_atual = 0;
    double X_atual = 0;
    double angulo_atual_pinpoint = 0;
    double angulo_atual_imu = 0;


    Hardware_do_robo robot;

    @Override
    public void runOpMode() {

        robot = new Hardware_do_robo(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        FtcDashboard dashboard = FtcDashboard.getInstance();
        dashboardTelemetry = dashboard.getTelemetry();

        robot.pinpoint.resetPosAndIMU();
        robot.imu.resetYaw();
        sleep(200); // Tempo para o sensor estabilizar

// 2. Só agora define a posição vinda do Autônomo
        robot.pinpoint.setPosition(
                new Pose2D(
                        DistanceUnit.CM,
                        PosiçãoGlobal.xFinal,
                        PosiçãoGlobal.yFinal,
                        AngleUnit.DEGREES,
                        PosiçãoGlobal.anguloFinal)
        );

        telemetry.addData("Status", "Iniciado");
        telemetry.update();

        robot.limelight.start();
        robot.limelight.pipelineSwitch(1);

        waitForStart();

        cicloTimer.reset();
        robot.runtime.reset();


        while (opModeIsActive()) {

            dt = cicloTimer.seconds();
            if (dt < 0.001) dt = 0.001;
            cicloTimer.reset();

            robot.pinpoint.update();
//                robot.getX();
//                robot.getY();
//                robot.getHeading();

            Y_atual = robot.getY();
            X_atual = robot.getX();
            angulo_atual_pinpoint = robot.pinpoint.getHeading(AngleUnit.RADIANS);
            angulo_atual_imu = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);


            alterar_time();
            Atualização_de_modo();
            if (Modo_automático) {

                Andar_automático();

            } else {

                Orientação_robo_imu_tatame(gamepad1.left_stick_x * 1.2, gamepad1.right_stick_x, -gamepad1.left_stick_y);

            }


            telemetria_limelight();
//                telemetria_dasboard();
            telemetria();


//                Pegador();
            lançador();
            transporte();

            telemetry.update();
//                dashboardTelemetry.update();

        }


    }

//        public void Pegador(){
//
//
//            Estado_gamepad2_a = gamepad2.a;
//
//        if(Estado_gamepad2_a && !Estado_gamepad2_a_anterior){
//
//            Estado_pegador = !Estado_pegador;
//
//
//
//        }
//
//        Potencia_ligado_pegador = (Range.clip(Potencia_ligado_pegador, Pot_min, Pot_max));
//
//        Estado_gamepad2_a_anterior = Estado_gamepad2_a;
//
//            if (Estado_pegador) {
//
//                robot.pegador.setPower(Potencia_ligado_pegador);
//            }
//            else {
//
//                robot.pegador.setPower(0);
//
//            }
//
////        Potencia_pegador = Estado_pegador ? Potencia_ligado_pegador : Potencia_parado;
//

    /// /            robot.pegador.setPower(Potencia_pegador);
//
//        }
    public void lançador() {

        EstadoAtualDoGamepad = gamepad2.b;
        if (EstadoAtualDoGamepad && !EstadoAnteriorDoGamepad) {

            Estado = !Estado;

        }

        Estado_padDown = gamepad2.dpad_down;

        if (Estado_padDown && !Estado_padDown_anterior) {

            Estado_lançador_limelight = !Estado_lançador_limelight;

        }

//            if(Estado_lançador_limelight){
//                LLResult result = robot.limelight.getLatestResult();
//
//                if (result != null && result.isValid()) {
//
//                    double distAtual = ditancia_limelight(result.getTy());
//
//                    double distMinimaReferencia = 117;
//
//                    double variacaoDistancia = Math.max(0, distAtual - distMinimaReferencia);
//
//                    if(distAtual < 150)
//                    {
//                        calculoDinamico = 0.8 + (variacaoDistancia * 0.0005);
//                    } else {
//
//                        calculoDinamico = 1;
//                    }
//                }
//
//                    Potencia_ligado_lancador = Range.clip(calculoDinamico, 0.8, 1.0);
//
//
//            }
//                else {
//                    if (gamepad2.dpad_up) {
//                        Potencia_ligado_lancador = 0.9;
//                    }
////                    if (gamepad2.dpad_down) {
////                        Potencia_ligado_lancador = 0.7;
////                    }
//                    if (gamepad2.dpad_left) {
//                        Potencia_ligado_lancador = 0.8;
//                    }
//                    if (gamepad2.dpad_right) {
//                        Potencia_ligado_lancador = 0.7;
//
//                    }
//                    if(gamepad2.dpad_down){
//
//                        Potencia_ligado_lancador = 0.5;
//
//                    }
//                }
        if (Estado_lançador_limelight) {
            LLResult result = robot.limelight.getLatestResult();

            if (result != null && result.isValid()) {
                double distAtual = ditancia_limelight(result.getTy());
                double distMinimaReferencia = 79;
                double variacaoDistancia = Math.max(0, distAtual - distMinimaReferencia);

                if (distAtual < 300) {
                    calculoDinamico = 0.7 + (variacaoDistancia * 0.0015);
                } else {
                    calculoDinamico = 1.0;
                }
                Potencia_ligado_lancador = Range.clip(calculoDinamico, 0.7, 1.0);
            }
            // Caso a Limelight perca o alvo, mantém a última potência ou define uma padrão (ex: 0.8)
        } else {

            if (gamepad2.dpad_up) {
                Potencia_ligado_lancador = 1.0;
            } else if (gamepad2.dpad_left) {
                Potencia_ligado_lancador = 0.9;
            } else if (gamepad2.dpad_right) {
                Potencia_ligado_lancador = 0.8;

            }
            Potencia_ligado_lancador = (Range.clip(Potencia_ligado_lancador, Pot_min, Pot_max));

        }

        EstadoAnteriorDoGamepad = EstadoAtualDoGamepad;
        Estado_padDown_anterior = Estado_padDown;


        Potencia = Estado ? Potencia_ligado_lancador : Potencia_parado;

        robot.lancador.setPower(Potencia);

    }

    public void transporte() {

        //            double cicloEmSegundos2 = tempoLancador.seconds();
///
//            if(gamepad2.y)
//            {
//                robot.transportador.setPower(Potencia_ligado_transportador);
//                tempoLancador.reset();
//                Estado_transportador = true;
//            } else
//            {
//                robot.transportador.setPower(0);
//            }


//            Estado_gamepad2_y = gamepad2.y;
//
//            if(Estado_gamepad2_y && !Estado_gamepad2_y_anterior){
//
//                Estado_transportador = !Estado_transportador;
//
//                if (!Estado_transportador) {
//
//                    tempoLancador.reset();
//
//                }
//            }
//            Estado_gamepad2_y_anterior = Estado_gamepad2_y;
///

//            if (!Estado_transportador) {
//
//                if (cicloEmSegundos2 < 0.5) {
//
//                    robot.transportador.setPower(0);
//                }
//                else if(cicloEmSegundos2 < 1.0){
//
//                    robot.transportador.setPower(-0.3);
//                }
//
//                else {
//
//                    robot.transportador.setPower(0);
//                    Estado_transportador = false;
//
//                }
//            }

        robot.transportador.setPower(0);
        robot.pegador.setPower(0);

        if (gamepad2.right_bumper) {

            robot.transportador.setPower(-0.5);
            robot.pegador.setPower(-0.5);
        } else if (gamepad2.left_bumper) {
            robot.transportador.setPower(0.5);
            robot.pegador.setPower(1);
        } else if (gamepad2.x) {

            robot.transportador.setPower(-0.5);
            robot.pegador.setPower(0);
        } else {
            robot.transportador.setPower(0);
            robot.pegador.setPower(0);
        }
    }

    public void Orientação_robo_imu_tatame(double lateral, double giro, double frente) {


        double yaw = verificação_angulo();

        double lateral_novo = lateral * Math.cos(-yaw) - frente * Math.sin(-yaw);
        double frente_novo = lateral * Math.sin(-yaw) + frente * Math.cos(-yaw);
//            double lateral_novo = lateral * Math.cos(-yaw) + frente * Math.sin(-yaw);
//            double frente_novo = -lateral * Math.sin(-yaw) + frente * Math.cos(-yaw);


        if (gamepad1.y) {

            robot.imu.resetYaw();

        }

        Movimento_Mecanum(lateral_novo, frente_novo, giro);

    }

    //        private double wrapAngle(double angle)
//        {
//            angle = ((angle % 360.0) + 360.0) % 360.0;
//
//            if (angle > 180.0) angle -= 360.0;
//
//            return angle;
//        }
    private double wrapAngle(double angle) {

        while (angle > Math.PI) {
            angle -= 2 * Math.PI;
        }

        while (angle < -Math.PI) {
            angle += 2 * Math.PI;
        }

        return angle;
    }

    public void Movimento_Mecanum(double lateral, double frente, double giro) {

        double multiplicador = 1.0;

        // Define o multiplicador ANTES
        if (gamepad1.right_bumper) multiplicador = 0.5;
        else if (gamepad1.left_bumper) multiplicador = 0.25;

        double pot_d_f = (frente - lateral - giro) * multiplicador;
        double pot_d_t = (frente + lateral - giro) * multiplicador;
        double pot_e_f = (frente + lateral + giro) * multiplicador;
        double pot_e_t = (frente - lateral + giro) * multiplicador;

//            double pot_d_f = frente - lateral - giro;
//            double pot_d_t = frente + lateral - giro;
//            double pot_e_f = frente + lateral + giro;
//            double pot_e_t = frente - lateral + giro;


        double correção = Math.max(Math.abs(pot_d_f), Math.abs(pot_d_t));
        double correção2 = Math.max(Math.abs(pot_e_f), Math.abs(pot_e_t));
        double denominador = Math.max(correção, correção2);


        if (denominador > 1.0) {

            pot_d_f /= denominador;
            pot_d_t /= denominador;
            pot_e_f /= denominador;
            pot_e_t /= denominador;

        }


        robot.motor_direito_frente.setPower(pot_d_f);
        robot.motor_direito_tras.setPower(pot_d_t);

        robot.motor_esquerdo_frente.setPower(pot_e_f);
        robot.motor_esquerdo_tras.setPower(pot_e_t);

//            if(gamepad1.right_bumper){
//
//                robot.motor_direito_frente.setPower(pot_d_f * 0.5);
//                robot.motor_direito_tras.setPower(pot_d_t * 0.5);
//
//                robot.motor_esquerdo_frente.setPower(pot_e_f * 0.5);
//                robot.motor_esquerdo_tras.setPower(pot_e_t * 0.5);
//            }
//            if(gamepad1.left_bumper){
//
//                robot.motor_direito_frente.setPower(pot_d_f * 0.25);
//                robot.motor_direito_tras.setPower(pot_d_t * 0.25);
//
//                robot.motor_esquerdo_frente.setPower(pot_e_f * 0.25);
//                robot.motor_esquerdo_tras.setPower(pot_e_t * 0.25);
//            }


    }

    public void Andar_automático() {

        double yaw2 = robot.getHeading();
//            double error2_X = X_alvo - X_atual;
//            double error2_Y = Y_alvo - Y_atual;
//            double error2_angulo = wrapAngle(Angulo_alvo_em_rad - yaw2);

        double potencia_frente_A = PID_Control_frente(dt);//-Kp_frente * error_X;
        double potencia_lateral_A = PID_Control_Lateral(dt); //Kp_lateral * error_Y;
        double potencia_final_giro = PID_Control_Giro(dt);//-Kp_giro * error_angulo;

//            if (Math.abs(error2_X) < Margem_de_erro && Math.abs(error2_Y) < Margem_de_erro && Math.abs(error2_angulo) < Margem_de_erro_angulo_em_rad)
//            {
//
//                Modo_automático = false;
//
//                Movimento_Mecanum(0,0,0);
//
//            } else {
//                double potencia_final_lateral = potencia_lateral_A * Math.cos(-yaw2) - potencia_frente_A * Math.sin(-yaw2);
//                double potencia_final_frente = -potencia_lateral_A * Math.sin(-yaw2) + potencia_frente_A * Math.cos(-yaw2);
        double potencia_final_lateral = potencia_lateral_A - potencia_frente_A;
        double potencia_final_frente = potencia_lateral_A + potencia_frente_A;

        Movimento_Mecanum(potencia_final_lateral, potencia_final_frente, potencia_final_giro);

        if (gamepad1.left_stick_x != 0 || gamepad1.right_stick_x != 0 || gamepad2.left_stick_y != 0) {

            Modo_automático = false;

        }


//            }


    }

    private double verificação_angulo() {

        YawPitchRollAngles orientation = robot.imu.getRobotYawPitchRollAngles();


        return (orientation.getYaw(AngleUnit.RADIANS));
    }

    private void Atualização_de_modo() {

        Estado_padUp1 = gamepad1.dpad_up;
        Estado_padDown1 = gamepad1.dpad_down;

        if (Time_respectivo) {

            if (Estado_padUp1 && !Estado_padUp_anterior1) {


                X_alvo = -234.52;
                Y_alvo = 148;
                Angulo_alvo = 50;
                Angulo_alvo_em_rad = Angulo_alvo * Math.PI / 180;

                Erro_acumulado_lateral = 0;
                Erro_ultimo_lateral = 0;

                Erro_acumulado_frente = 0;
                Erro_ultimo_frente = 0;

                Erro_acumulado_giro = 0;
                Erro_ultimo_giro = 0;


                Modo_automático = !Modo_automático;


            }
            if (Estado_padDown1 && !Estado_padDown_anterior1) {


                X_alvo = -8.38;
                Y_alvo = 110.09;
                Angulo_alvo = 17;
                Angulo_alvo_em_rad = Angulo_alvo * Math.PI / 180;


                Erro_acumulado_lateral = 0;
                Erro_ultimo_lateral = 0;

                Erro_acumulado_frente = 0;
                Erro_ultimo_frente = 0;

                Erro_acumulado_giro = 0;
                Erro_ultimo_giro = 0;

                Modo_automático = !Modo_automático;

            }


        } else {

            if (Estado_padUp1 && !Estado_padUp_anterior1) {


                X_alvo = 0;
                Y_alvo = 0;
                Angulo_alvo = 0;
                Angulo_alvo_em_rad = Angulo_alvo * Math.PI / 180;

                Erro_acumulado_lateral = 0;
                Erro_ultimo_lateral = 0;

                Erro_acumulado_frente = 0;
                Erro_ultimo_frente = 0;

                Erro_acumulado_giro = 0;
                Erro_ultimo_giro = 0;


                Modo_automático = !Modo_automático;


            }
            if (Estado_padDown1 && !Estado_padDown_anterior1) {


                X_alvo = 0;
                Y_alvo = 0;
                Angulo_alvo = 0;
                Angulo_alvo_em_rad = Angulo_alvo * Math.PI / 180;


                Erro_acumulado_lateral = 0;
                Erro_ultimo_lateral = 0;

                Erro_acumulado_frente = 0;
                Erro_ultimo_frente = 0;

                Erro_acumulado_giro = 0;
                Erro_ultimo_giro = 0;

                Modo_automático = !Modo_automático;

            }


        }
//            if(Estado_padLeft && !Estado_padLeft_anterior){
//
//
//                X_alvo = -252.63;
//                Y_alvo = -88.47;
//                Angulo_alvo = 180;
//                Angulo_alvo_em_rad = Angulo_alvo * Math.PI / 180;
//
//
//                Erro_acumulado_lateral = 0;
//                Erro_ultimo_lateral = 0;
//
//                Erro_acumulado_frente = 0;
//                Erro_ultimo_frente = 0;
//
//                Erro_acumulado_giro = 0;
//                Erro_ultimo_giro = 0;
//
//                Modo_automático = !Modo_automático;
//
//            }
        Estado_padUp_anterior1 = Estado_padUp1;
        Estado_padDown_anterior1 = Estado_padDown1;


    }

    public double PID_Control_Lateral(double timer) {

//        Ki_lateral = 0;
//        Kd_lateral = 0.0005;

//        double cicloEmSegundos = cicloTimer.seconds();
//        if (cicloEmSegundos <= 0) {
//            cicloEmSegundos = 0.0001;
//        }
        double error_Y = Y_alvo - Y_atual;


        Erro_acumulado_lateral += error_Y * timer;

        double derivação = (error_Y - Erro_ultimo_lateral) / timer;

        Erro_ultimo_lateral = error_Y;

        double Saida_pid_lateral = (Kp_lateral * error_Y) + (Ki_lateral * Erro_acumulado_lateral) + (Kd_lateral * derivação);


        return Saida_pid_lateral;
    }

    ;

    public double PID_Control_frente(double timer) {

//        Ki_frente = 0;
//        Kd_frente = -0.0005;

//        double cicloEmSegundos = cicloTimer.seconds();
//
//        if (cicloEmSegundos <= 0) {
//            cicloEmSegundos = 0.0001;
//        }
        double error_X = X_alvo - X_atual;

        Erro_acumulado_frente += error_X * timer;

        double derivação = (error_X - Erro_ultimo_frente) / timer;

        Erro_ultimo_frente = error_X;

        double Saida_pid_frente = (Kp_frente * error_X) + (Ki_frente * Erro_acumulado_frente) + (Kd_frente * derivação);


        return Saida_pid_frente;
    }

    ;

    public double PID_Control_Giro(double timer) {

//            Ki_giro = 0;
//            Kd_giro = 0.004;

//            double cicloEmSegundos = cicloTimer.seconds();
//
//            if (cicloEmSegundos <= 0) {
//                cicloEmSegundos = 0.0001;
//            }

        double yaw = verificação_angulo();

        double error_angulo = wrapAngle(Angulo_alvo_em_rad - yaw);


        Erro_acumulado_giro += error_angulo * timer;

        double derivação = (error_angulo - Erro_ultimo_giro) / timer;

        Erro_ultimo_giro = error_angulo;

        double Saida_pid_giro = (Kp_giro * error_angulo) + (Ki_giro * Erro_acumulado_giro) + (Kd_giro * derivação);


        return Saida_pid_giro;
    }

    ;

    //        public void telemetria_dasboard() {
//
//            dashboardTelemetry.addData("Status", "Run Time: " + robot.runtime.toString());
//            dashboardTelemetry.addData("Potência pegador:", "%.2f", robot.pegador.getPower());
//            dashboardTelemetry.addData("Potência lançador:", "%.2f", robot.lancador.getPower());
//            dashboardTelemetry.addData("Potência Média:", "%.2f", robot.motor_direito_frente.getPower() * robot.motor_direito_tras.getPower() * robot.motor_esquerdo_frente.getPower() * robot.motor_esquerdo_tras.getPower() / 4);
//            dashboardTelemetry.addData("Lançador", Estado ? "Ligado" : "Desligado");
//            dashboardTelemetry.addData("Yaw:", "%.2f", robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
//            dashboardTelemetry.addData("X:", "%.2f", robot.getX());
//            dashboardTelemetry.addData("Y:", "%.2f", robot.getY());
//            dashboardTelemetry.addData("Heading", "%.2f", robot.getHeading());
//            dashboardTelemetry.addData("Modo Atual:", Modo_automático ? "Automático" : "Manual");
//            dashboardTelemetry.addData("Y_Alvo:", "%.2f", Y_alvo);
//            dashboardTelemetry.addData("X_Alvo:", "%.2f", X_alvo);
//            dashboardTelemetry.addData("Angulo_alvo:", "%.2f", Angulo_alvo);
//            dashboardTelemetry.addData("Angulo_alvo_em_rad:", "%.2f", Angulo_alvo_em_rad);
//
//        }
    public void telemetria() {

        telemetry.addData("Modo Atual:", Modo_automático ? "Automático" : "Manual");
        telemetry.addData("Potência pegador:", "%.2f", robot.pegador.getPower());
        telemetry.addData("Potência lançador:", "%.2f", robot.lancador.getPower());
        telemetry.addData("Potência Média:", "%.2f", robot.motor_direito_frente.getPower() * robot.motor_direito_tras.getPower() * robot.motor_esquerdo_frente.getPower() * robot.motor_esquerdo_tras.getPower() / 4);
        telemetry.addData("Lançador", Estado ? "Ligado" : "Desligado");
        telemetry.addData("Yaw:", "%.2f", robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES));
        telemetry.addData("X:", "%.2f", robot.getX());
        telemetry.addData("Y:", "%.2f", robot.getY());
        telemetry.addData("Heading", "%.2f", robot.getHeading());
        telemetry.addData("Y_Alvo:", "%.2f", Y_alvo);
        telemetry.addData("X_Alvo:", "%.2f", X_alvo);
        telemetry.addData("Angulo_alvo:", "%.2f", Angulo_alvo);
        telemetry.addData("Limalight", Estado_lançador_limelight);


    }

    public double ditancia_limelight(double ty) {
//
//            double escala = 26846.47;
//            double distancia = (escala / ta);
//
//            return distancia;
//             h1 = 36; // Altura da câmera no robô (em cm)
//             h2 = 73; // Altura da AprilTag no campo (em cm) - Verifique o manual do Decode
//             angulo_montagem = 30; // Ângulo que a câmera está inclinada para cima (em graus)

        double angulo_total_rad = Math.toRadians(angulo_montagem + ty);

        if (angulo_total_rad == 0) return 0;

        return (h2 - h1) / Math.tan(angulo_total_rad);
    }

    ;

    public void telemetria_limelight() {

        LLResult result = robot.limelight.getLatestResult();

        if (result != null && result.isValid()) {

            Pose3D botpose = result.getBotpose();

            telemetry.addData("tx", result.getTx());
            telemetry.addData("ty", result.getTy());
            telemetry.addData("ta", result.getTa());
            telemetry.addData("Distancia da limelight", ditancia_limelight(result.getTy()));


        } else {

            telemetry.addData("limelight", "Apriltag não detectada");


        }

    }

    public void alterar_time() {

        Estado_gamepad1_x = gamepad1.x;
        Estado_gamepad1_b = gamepad1.b;

        if (Estado_gamepad1_x && !Estado_gamepad1_x_anterior) {

            Time_respectivo = !Time_respectivo;

        }

        Estado_gamepad1_x_anterior = Estado_gamepad1_x;
        Estado_gamepad1_b_anterior = Estado_gamepad1_b;

    }
}