package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * Created by Akanksha.Joshi on 2017-11-12.
 */

@TeleOp(name="Simple Drive", group="Training")
public class SimpleDriveOpMode extends OpMode
{

    DcMotor leftWheel;
    DcMotor rightWheel;
    double drivePower = 0.5;

    @Override
    public void init() {
        leftWheel = hardwareMap.dcMotor.get("left_wheel");
        rightWheel = hardwareMap.dcMotor.get("right_wheel");

        rightWheel.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void loop() {
        leftWheel.setPower(drivePower);
        rightWheel.setPower(drivePower);
    }
}
