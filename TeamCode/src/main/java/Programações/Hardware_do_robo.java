package Programações;


import android.util.Size;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.ArrayList;
import java.util.List;

public class Hardware_do_robo {


    private static final Logger log = LoggerFactory.getLogger(Hardware_do_robo.class);
    public AprilTagProcessor aprilTagProcessor;
    public VisionPortal VisionPortal;
    public List<AprilTagDetection> aprilTagDetected = new ArrayList<>();
    public Telemetry telemetry;







    public GoBildaPinpointDriver pinpoint;
    public DcMotor motor_esquerdo_tras , motor_direito_tras, motor_direito_frente, motor_esquerdo_frente, lancador, pegador, transportador;
    public IMU imu;
    public Limelight3A limelight;
    //oi
    public ElapsedTime runtime = new ElapsedTime();

    HardwareMap hwMap = null;

    public  Hardware_do_robo(HardwareMap ahwMap) {


        hwMap = ahwMap;



        pinpoint = hwMap.get(GoBildaPinpointDriver.class, "pinpoint");

        motor_esquerdo_tras = hwMap.get(DcMotor.class, "esquerdo_t");
        motor_esquerdo_frente = hwMap.get(DcMotor.class, "esquerdo_f");
        motor_direito_tras = hwMap.get(DcMotor.class, "direito_t");
        motor_direito_frente = hwMap.get(DcMotor.class, "direito_f");
        imu = hwMap.get(IMU.class, "imu");
        lancador = hwMap.get(DcMotor.class, "lancador");
        pegador = hwMap.get(DcMotor.class, "pegador");
        transportador = hwMap.get(DcMotor.class, "transportador");
        limelight = hwMap.get(Limelight3A.class, "limelight");



        motor_esquerdo_tras.setDirection(DcMotor.Direction.REVERSE);
        motor_direito_tras.setDirection(DcMotor.Direction.FORWARD);
        motor_direito_frente.setDirection(DcMotor.Direction.FORWARD);
        motor_esquerdo_frente.setDirection(DcMotor.Direction.REVERSE);
        lancador.setDirection(DcMotor.Direction.FORWARD);
        transportador.setDirection(DcMotor.Direction.FORWARD);
        pegador.setDirection(DcMotor.Direction.REVERSE);

        motor_esquerdo_frente.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor_direito_frente.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor_esquerdo_tras.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor_direito_tras.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lancador.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pegador.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        transportador.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motor_esquerdo_frente.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor_direito_frente.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor_esquerdo_tras.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor_direito_tras.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lancador.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pegador.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        transportador.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));


        pinpoint.setOffsets(-84, -168, DistanceUnit.MM);
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);
        pinpoint.resetPosAndIMU();

        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw());




    }


    public double getHeading() {

        return pinpoint.getHeading(AngleUnit.DEGREES);
    }

    public double getX() {

        return pinpoint.getPosX(DistanceUnit.CM);
    }

    public double getY() {

        return pinpoint.getPosY(DistanceUnit.CM);
    }


//    public void init(HardwareMap hwMap2, Telemetry telemetry) {
//        this.telemetry = telemetry;
//        aprilTagProcessor = new AprilTagProcessor.Builder()
//                .setDrawTagID(true)
//                .setDrawAxes(true)
//                .setDrawTagOutline(true)
//                .setDrawCubeProjection(true)
//                .setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES)
//                .build();
//
//        VisionPortal.Builder builder = new VisionPortal.Builder();
//        builder.setCamera(hwMap2.get(WebcamName.class, "Webcam 1"));
//        builder.setCameraResolution(new Size(640, 480));
//        builder.addProcessor(aprilTagProcessor);
//
//        VisionPortal = builder.build();
//
//
//    }
//
//
//    public void update(){
//        aprilTagDetected = aprilTagProcessor.getDetections();
//    }
//
//    public List<AprilTagDetection> Detections(){
//
//        return aprilTagDetected;
//    }
//    public void diplayDetectionTelemetry(AprilTagDetection detectedId){
//        if(detectedId == null){return;}
//        if (detectedId.metadata != null) {
//            telemetry.addLine(String.format("\n==== (ID %d) %s", detectedId.id, detectedId.metadata.name));
//            telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detectedId.ftcPose.x, detectedId.ftcPose.y, detectedId.ftcPose.z));
//            telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detectedId.ftcPose.pitch, detectedId.ftcPose.roll, detectedId.ftcPose.yaw));
//            telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detectedId.ftcPose.range, detectedId.ftcPose.bearing, detectedId.ftcPose.elevation));
//        } else {
//            telemetry.addLine(String.format("\n==== (ID %d) Unknown", detectedId.id));
//            telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detectedId.center.x, detectedId.center.y));
//        }
//    }
//    public AprilTagDetection Specific_Id(int Id){
//
//        for (AprilTagDetection detection : aprilTagDetected){
//
//            if(detection.id == Id){
//
//                return detection;
//            }
//        }
//        return null;
//    }
//
//    public void stop_visual(){
//
//        if(VisionPortal != null){
//            VisionPortal.close();
//
//        }
//    }


}