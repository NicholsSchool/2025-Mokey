package org.firstinspires.ftc.teamcode.autos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.constants.ElevatorConstants;
import org.firstinspires.ftc.teamcode.math_utils.Vector;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Elevator;

import java.util.concurrent.TimeUnit;

/**
 * Testing Auto for Lerp
 */
@Autonomous(name="[EMERGENCY] strafe sample DEAD AUTO", group="EmergencyAutos")
public class DeadReckoningStrafeSample extends LinearOpMode {

    @Override
    public void runOpMode() {
        Drivetrain drivetrain = new Drivetrain(hardwareMap, new Pose2D(DistanceUnit.INCH,0, 0, AngleUnit.DEGREES, 0), 270, false, false);
        ElapsedTime time = new ElapsedTime();
        Elevator elevator = new Elevator(hardwareMap, false);

        waitForStart();

        drivetrain.setTargetHeading(drivetrain.getPose().getHeading(AngleUnit.DEGREES));
        elevator.setElevatorSetpoint(ElevatorConstants.WAYPOINT_ZERO);
        while(time.time(TimeUnit.SECONDS) < 5.0){
            drivetrain.drive(new Vector(-0.7,0.0), 0, true);
            elevator.periodic();
            drivetrain.update();
        }
    }
}
