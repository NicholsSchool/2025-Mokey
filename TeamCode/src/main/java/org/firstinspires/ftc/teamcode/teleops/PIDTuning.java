package org.firstinspires.ftc.teamcode.teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.constants.ElevatorConstants;
import org.firstinspires.ftc.teamcode.constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.controller.Controller;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

@TeleOp(name = "PIDTuning", group = "Dev")
public class PIDTuning extends OpMode {

    private Drivetrain drivetrain;
    private Intake intake;
    private Elevator elevator;
    private Controller controller1;

    FtcDashboard dashboard;

    public void init() {

        drivetrain = new Drivetrain(hardwareMap, new Pose2D(DistanceUnit.METER, 0, 0, AngleUnit.DEGREES, 0), 270, false, false);
        intake = new Intake(hardwareMap, false);
        elevator = new Elevator(hardwareMap, false);
        controller1 = new Controller( gamepad1 );



        dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        telemetry.setMsTransmissionInterval(50);
    }

    public void loop() {

        controller1.update();

        drivetrain.driveToPose(new Pose2D(DistanceUnit.METER, 0, 0, AngleUnit.DEGREES, (controller1.square.isPressed() ? 90 : 0)), false);

        intake.setIntakeSetpoint(controller1.triangle.isPressed() ? IntakeConstants.WAYPOINT_EXTEND : IntakeConstants.WAYPOINT_RETRACT);

        elevator.setElevatorSetpoint( controller1.x.isPressed() ? -1000 : -200);

        telemetry.addData("intake desired", controller1.triangle.isPressed() ? IntakeConstants.WAYPOINT_EXTEND : IntakeConstants.WAYPOINT_RETRACT);
        telemetry.addData("intake real", -intake.getIntakeSlidePos());
        telemetry.addData("elevator desired", controller1.x.isPressed() ? -1000 : -200);
        telemetry.addData("elevator real", elevator.getEncoderPosition());
        telemetry.addData("drive desired", controller1.square.isPressed() ? 90 : 0);
        telemetry.addData("drive real", drivetrain.getPose().getHeading(AngleUnit.DEGREES));

        //intake.periodic();
        drivetrain.update();
        elevator.periodic();

    }

}
