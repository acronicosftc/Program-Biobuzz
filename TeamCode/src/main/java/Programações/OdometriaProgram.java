package Programações;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.Range;

public class OdometriaProgram {

    private Hardware_do_robo robot;
    private LinearOpMode opMode;

    public OdometriaProgram(Hardware_do_robo robot, LinearOpMode opMode) {
        this.robot = robot;
        this.opMode = opMode;
    }

    private void mecanumDrive(double strafe, double forward, double rotate, double powerLimit) {

        double fl = forward + strafe + rotate;
        double fr = forward - strafe - rotate;
        double bl = forward - strafe + rotate;
        double br = forward + strafe - rotate;

        robot.motor_esquerdo_frente.setPower(Range.clip(fl, -powerLimit, powerLimit));
        robot.motor_direito_frente.setPower(Range.clip(fr, -powerLimit, powerLimit));
        robot.motor_esquerdo_tras.setPower(Range.clip(bl, -powerLimit, powerLimit));
        robot.motor_direito_tras.setPower(Range.clip(br, -powerLimit, powerLimit));
    }

    public void goToPoint(double targetX_cm, double targetY_cm, double targetAngle_deg,
                          double maxPower, double stopThresholdCm, double angleThresholdDeg) {

        final double kDrive = 0.05;     // ganho drive
        final double kStrafe = 0.06;    // ganho strafe
        final double kTurn = 0.02;      // ganho giro
        final long TIMEOUT = 9000;

        long start = System.currentTimeMillis();

        while (opMode.opModeIsActive() && System.currentTimeMillis() - start < TIMEOUT) {

            robot.pinpoint.update();

            double curX = robot.getX();
            double curY = robot.getY();
            double curAngle = robot.getHeading();
            double headingRad = Math.toRadians(curAngle);

            double dx = targetX_cm - curX;
            double dy = targetY_cm - curY;

            double distanceError = Math.hypot(dx, dy);
            double angleError = wrapAngle(targetAngle_deg - curAngle);

            if (distanceError < stopThresholdCm && Math.abs(angleError) < angleThresholdDeg)
                break;

            double x_robot =  dx * Math.cos(headingRad) + dy * Math.sin(headingRad);
            double y_robot = -dx * Math.sin(headingRad) + dy * Math.cos(headingRad);

            double forward = Range.clip(y_robot * kDrive, -maxPower, maxPower);
            double strafe  = Range.clip(x_robot * kStrafe, -maxPower, maxPower);
            double rotate  = Range.clip(angleError * kTurn, -0.4, 0.4);

            mecanumDrive(strafe, forward, rotate, maxPower);

            opMode.telemetry.addData("Target", "X %.1f | Y %.1f | A %.1f", targetX_cm, targetY_cm, targetAngle_deg);
            opMode.telemetry.addData("Current", "X %.1f | Y %.1f | A %.1f", curX, curY, curAngle);
            opMode.telemetry.addData("Error", "Dist %.1f | Ang %.1f", distanceError, angleError);
            opMode.telemetry.update();

            opMode.sleep(10);
        }

        mecanumDrive(0,0,0,0);
        opMode.sleep(50);
    }

    private double wrapAngle(double angle) {
        while (angle > 180) angle -= 360;
        while (angle < -180) angle += 360;
        return angle;
    }
}