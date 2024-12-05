package org.firstinspires.ftc.teamcode.teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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

    public double intake_p = IntakeConstants.SLIDE_P;
    public double elevator_p = ElevatorConstants.SLIDE_P;

    public void init() {

        drivetrain = new Drivetrain(hardwareMap, 0, 0, Math.PI, false);
        intake = new Intake(hardwareMap);
        elevator = new Elevator(hardwareMap, drivetrain::getElevatorPosition);
        controller1 = new Controller( gamepad1 );

        dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        telemetry.setMsTransmissionInterval(50);
    }

    public void loop() {

        controller1.update();

        intake.setPIDCoefficients(intake_p, 0.0, 0.0);
        elevator.setPIDCoefficients(elevator_p, 0.0, 0.0);

        intake.setIntakeSetpoint( controller1.triangle.isPressed() ? 27000 : 1000 );

        elevator.setSetpoint( controller1.x.isPressed() ? 50000 : 10000 );

        telemetry.addData("intake desired", controller1.triangle.isPressed() ? 30000 : 10000);
        telemetry.addData("intake real", intake.getEncoderPosition());
        telemetry.addData("elevator desired", controller1.x.isPressed() ? 50000 : 10000 );
        telemetry.addData("elevator real", elevator.getEncoderPosition());
        telemetry.addData("wrist desired",  controller1.circle.isPressed() ? 0 : 100 );
        telemetry.addData("wrist real", intake.getWristServoPositions()[0]);

        intake.periodic();
        elevator.periodic();

    }

}
