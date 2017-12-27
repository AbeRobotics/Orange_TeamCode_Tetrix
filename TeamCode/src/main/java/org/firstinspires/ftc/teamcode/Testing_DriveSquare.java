package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Created by akanksha.joshi on 2017-11-19.
 */
@Autonomous(name="DriveSquare", group="Linear Opmode") // Or @Autonomous
//@Disabled
public class Testing_DriveSquare extends LinearOpMode
{
    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
    // DcMotor leftMotor = null;
    // DcMotor rightMotor = null;

    DcMotor leftMotor;
    DcMotor rightMotor;

    double power = 0.5;

    @Override
    public void runOpMode() throws InterruptedException
    {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        leftMotor = hardwareMap.dcMotor.get("Left_Motor");
        rightMotor = hardwareMap.dcMotor.get("Right_Motor");
        leftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        runtime.reset();

        for (int i = 0; i<4; i++)
        {
            leftMotor.setPower(power);
            rightMotor.setPower(power);
            sleep(2000);

            leftMotor.setPower(-power);
            rightMotor.setPower(-power);
            sleep(1000);
        }
        leftMotor.setPower(0.0);
        rightMotor.setPower(0.0);
    }
}
