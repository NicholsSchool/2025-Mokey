package org.firstinspires.ftc.teamcode.teleops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.constants.ElevatorConstants;
import org.firstinspires.ftc.teamcode.constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.controller.Controller;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.Intake;

@TeleOp(name = "PIDTuning", group = "Dev")
public class PIDTuning extends OpMode {

    private Intake intake;
    private Elevator elevator;
    private Controller controller1;

    FtcDashboard dashboard;

    public double intake_p = IntakeConstants.INTAKE_P;
    public double intake_d = IntakeConstants.INTAKE_D;

    public double elevator_p = ElevatorConstants.SLIDE_P;

    public void init() {

        intake = new Intake(hardwareMap);
        elevator = new Elevator(hardwareMap);
        controller1 = new Controller( gamepad1 );

        dashboard = FtcDashboard.getInstance();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        telemetry.setMsTransmissionInterval(50);
    }

    public void loop() {

        controller1.update();

        intake.setPIDCoefficients(intake_p, 0.0, intake_d);
        intake.setPIDCoefficients(elevator_p, 0.0, 0.0);

        intake.setSetpoint( controller1.triangle.isPressed() ? 30000 : 10000 );

        elevator.setSetpoint( controller1.x.isPressed() ? 30000 : 10000 );

        telemetry.addData("intake desired", controller1.triangle.isPressed() ? 30000 : 10000);
        telemetry.addData("intake real", intake.getEncoderPosition());
        telemetry.addData("elevator desired", controller1.x.isPressed() ? 30000 : 10000 );
        telemetry.addData("elevator real", elevator.getEncoderPosition());

        intake.periodic();
        elevator.periodic();

    }

}
