package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by akanksha.joshi on 2017-11-19.
 */
@Autonomous(name="RotateServo", group="Linear Opmode")  // Or @Autonomous
//@Disabled
public class RotateServo extends LinearOpMode
{
    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    // DcMotor leftMotor = null;
    // DcMotor rightMotor = null;

    Servo servo;
    double servoPosition = 0.0;

    @Override
    public void runOpMode() throws InterruptedException
    {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        servo = hardwareMap.servo.get("servo");
        servo.setPosition(servoPosition);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        servoPosition = 0.5;
        servo.setPosition(servoPosition);
        sleep(2000);

        servoPosition = 1.0;
        servo.setPosition(servoPosition);
    }
}
