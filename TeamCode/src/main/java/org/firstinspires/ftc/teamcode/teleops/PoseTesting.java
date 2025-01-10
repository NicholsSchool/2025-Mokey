package org.firstinspires.ftc.teamcode.teleops;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.controller.Controller;
import org.firstinspires.ftc.teamcode.math_utils.PoseEstimator;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import java.util.Locale;

@TeleOp(name = "Pose Testing", group = "Dev")
public class PoseTesting extends OpMode {

    private PoseEstimator poseEstimator;
    FtcDashboard dashboard;
    private Drivetrain drivetrain;
    Controller driverOI;

    @Override
    public void init() {

        Pose2D initialPose = new Pose2D(DistanceUnit.INCH, 0, 40, AngleUnit.DEGREES, 270);
        drivetrain = new Drivetrain(hardwareMap, initialPose, 270, false);
        poseEstimator = drivetrain.poseEstimator;
        driverOI = new Controller(gamepad1);
        dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        telemetry.setMsTransmissionInterval(50);
    }

    @Override
    public void loop() {

        drivetrain.update();
        driverOI.update();

        drivetrain.drive(driverOI.leftStick.toVector(), driverOI.rightStick.x.value(), !driverOI.rightBumper.isPressed());

        //drivetrain.driveToPose(new Pose2D(DistanceUnit.INCH, 40, 40, AngleUnit.DEGREES, 0), true);

        telemetry.addData("OTOS Heading", poseEstimator.otos.getHeading());
        telemetry.addData("OTOS Position", poseEstimator.otos.getPosition().toString());

        telemetry.addData("Initial Pose", String.format(Locale.US, "(%.3f, %.3f)", poseEstimator.initialPose.getX(DistanceUnit.INCH), poseEstimator.initialPose.getY(DistanceUnit.INCH)));
        telemetry.addData("Robot Pose", String.format(Locale.US, "(%.3f, %.3f)", poseEstimator.getPose().getX(DistanceUnit.INCH), poseEstimator.getPose().getY(DistanceUnit.INCH)));
        telemetry.addData("Using LL", poseEstimator.isUsingLL());

        TelemetryPacket packet = new TelemetryPacket();
        packet.fieldOverlay()
                .setAlpha(0.7)
                .setStrokeWidth(1)
                .setStroke("Red")
                .strokeCircle(
                        poseEstimator.getPose().getX(DistanceUnit.INCH),
                        poseEstimator.getPose().getY(DistanceUnit.INCH),
                        9
                )
                .setStroke("Green")
                .strokeLine(
                        poseEstimator.getPose().getX(DistanceUnit.INCH),
                        poseEstimator.getPose().getY(DistanceUnit.INCH),
                        poseEstimator.getPose().getX(DistanceUnit.INCH) + (9 * Math.cos(poseEstimator.getPose().getHeading(AngleUnit.RADIANS))),
                        poseEstimator.getPose().getY(DistanceUnit.INCH) + (9 * Math.sin(poseEstimator.getPose().getHeading(AngleUnit.RADIANS)))
                );
        dashboard.sendTelemetryPacket(packet);
        telemetry.update();

    }
}
