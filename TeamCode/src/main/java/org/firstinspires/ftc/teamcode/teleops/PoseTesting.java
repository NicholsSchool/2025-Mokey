package org.firstinspires.ftc.teamcode.teleops;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.math_utils.PoseEstimator;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import java.util.Locale;

@TeleOp(name = "Pose Testing", group = "Dev")
public class PoseTesting extends OpMode {

    private PoseEstimator pose;
    FtcDashboard dashboard;

    @Override
    public void init() {

        pose = new PoseEstimator(hardwareMap, new Pose2D(DistanceUnit.INCH, 0, 20, AngleUnit.DEGREES, 270), true);

        dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        telemetry.setMsTransmissionInterval(50);
    }

    @Override
    public void loop() {

        pose.update();

        telemetry.addData("OTOS Heading", pose.otos.getHeading());
        telemetry.addData("OTOS Position", pose.otos.getPosition().toString());

        telemetry.addData("Initial Pose", String.format(Locale.US, "(%.3f, %.3f)", pose.initialPose.getX(DistanceUnit.INCH), pose.initialPose.getY(DistanceUnit.INCH)));
        telemetry.addData("Robot Pose", String.format(Locale.US, "(%.3f, %.3f)", pose.getPose().getX(DistanceUnit.INCH), pose.getPose().getY(DistanceUnit.INCH)));
        telemetry.addData("Using LL", pose.isUsingLL());

        TelemetryPacket packet = new TelemetryPacket();
        packet.fieldOverlay()
                .setAlpha(0.7)
                .setStrokeWidth(1)
                .setStroke("Red")
                .strokeCircle(
                        pose.getPose().getX(DistanceUnit.INCH),
                        pose.getPose().getY(DistanceUnit.INCH),
                        9
                )
                .setStroke("Green")
                .strokeLine(
                        pose.getPose().getX(DistanceUnit.INCH),
                        pose.getPose().getY(DistanceUnit.INCH),
                        pose.getPose().getX(DistanceUnit.INCH) + (9 * Math.cos(pose.getPose().getHeading(AngleUnit.RADIANS))),
                        pose.getPose().getY(DistanceUnit.INCH) + (9 * Math.sin(pose.getPose().getHeading(AngleUnit.RADIANS)))
                );
        dashboard.sendTelemetryPacket(packet);
        telemetry.update();

    }
}
