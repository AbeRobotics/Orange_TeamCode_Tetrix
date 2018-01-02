package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by 500260501 on 13/11/2017.
 */
@TeleOp(name="Gamepad Drive", group="Teleop")
//@Disabled
public class Abe_Teleop_Main extends OpMode
{
    public final void sleep(long milliseconds) {
        try {
            Thread.sleep(milliseconds);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    DcMotor leftWheel;
    DcMotor rightWheel;
    DcMotor liftMotor;

    Servo leftClaw;
    Servo rightClaw;
    Servo leftArm;

    double leftWheelPower;
    double rightWheelPower;

    double leftArmDefaultPosition = 0.00;

    double leftArmPosition = 0.70;

    @Override
    public void init()
    {
        leftWheel = hardwareMap.dcMotor.get("left_wheel");
        rightWheel = hardwareMap.dcMotor.get("right_wheel");
        liftMotor = hardwareMap.dcMotor.get("lift_motor");

        leftClaw = hardwareMap.servo.get("left_claw");
        rightClaw = hardwareMap.servo.get("right_claw");
        leftArm = hardwareMap.servo.get("left_arm");


        leftClaw.setPosition(0.5);
        rightClaw.setPosition(0.5);

        leftArm.setPosition(leftArmDefaultPosition);

        rightWheel.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    @Override
    public void loop()
    {
        leftWheelPower = gamepad1.left_stick_y; //-1 to 0
        rightWheelPower = gamepad1.right_stick_y;

        leftWheel.setPower(leftWheelPower);
        rightWheel.setPower(rightWheelPower);

        leftArm.setPosition(leftArmDefaultPosition);

        //Controlling the claw with the triggers
        if(gamepad1.left_trigger > 0.01)
        {
            leftClaw.setPosition(0.35);
            rightClaw.setPosition(0.65);
        }
        if(gamepad1.right_trigger > 0.01)
        {
            leftClaw.setPosition(0.65);
            rightClaw.setPosition(0.35);
        }

        //Controlling the lift mechanism using y and a buttons
        if(gamepad1.y)
        {
            liftMotor.setPower(1.0);
        }
        else if(gamepad1.a)
        {
            liftMotor.setPower(-1.0);
        }
        else
        {
            liftMotor.setPower(0);
        }

        //Controlling the arms (will not be used in teleop mode so comment out)
        if(gamepad1.x)
        {
            leftArm.setPosition(leftArmPosition);
            sleep(150000);
        }
    }
}