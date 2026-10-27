@Autonomous(name = "Auto_Odometria_Pro", group = "Autonomous")
public class AutoOdometriaPro extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // Inicialização do hardware (Motores, Servos, Odometria)
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        // Definir a pose inicial (X, Y, Heading em graus convertidos para radianos)
        Pose2D startPose = new Pose2D(36, -60, Math.toRadians(90));
        drive.setPoseEstimate(startPose);

        // Construção das Trajetórias
        TrajectorySequence trajSequence = drive.trajectorySequenceBuilder(startPose)
                // Ir para a área de pontuação inicial
                .lineToLinearHeading(new Pose2D(36, -36, Math.toRadians(0)))
                .addTemporalMarker(0.5, () -> {
                    // Abrir garra ou levantar braço no meio do caminho
                    claw.open();
                })
                .waitSeconds(0.5)
                // Ir até a pilha de elementos
                .splineTo(new Pose2D(48, -12, Math.toRadians(0)), Math.toRadians(0))
                // Retornar para estacionar
                .lineToLinearHeading(new Pose2D(60, -60, Math.toRadians(90)))
                .build();

        waitForStart();

        if (isStopRequested()) return;

        // Executar a sequência gravada
        drive.followTrajectorySequence(trajSequence);
    }
}
