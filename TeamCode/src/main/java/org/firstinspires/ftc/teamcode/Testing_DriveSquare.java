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
/*
public void gyroDrive ( double speed,
                            double distance,
                            double angle) {

        int     newLeftTarget;
        int     newRightTarget;
        int     moveCounts;
        double  max;
        double  error;
        double  steer;
        double  leftSpeed;
        double  rightSpeed;

        // Ensure that the opmode is still active
        if (true) {

            // Determine new target position, and pass to motor controller
            moveCounts = (int)(distance * OPModeConstants.ticksPerInch);
            newLeftTarget = leftWheel.getCurrentPosition() + moveCounts;
            newRightTarget = rightWheel.getCurrentPosition() + moveCounts;

            // Set Target and Turn On RUN_TO_POSITION
            leftWheel.setTargetPosition(newLeftTarget);
            rightWheel.setTargetPosition(newRightTarget);

            leftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // start motion.
            speed = Range.clip(Math.abs(speed), 0.0, 1.0);
            leftWheel.setPower(speed);
            rightWheel.setPower(speed);

            // keep looping while we are still active, and BOTH motors are running.
            while ((leftWheel.isBusy() && rightWheel.isBusy())) {

                // adjust relative speed based on heading error.
                error = getError(angle);
                steer = getSteer(error, 0.15);

                // if driving in reverse, the motor correction also needs to be reversed
                if (distance < 0)
                    steer *= -1.0;

                leftSpeed = speed - steer;
                rightSpeed = speed + steer;

                // Normalize speeds if either one exceeds +/- 1.0;
                max = Math.max(Math.abs(leftSpeed), Math.abs(rightSpeed));
                if (max > 1.0)
                {
                    leftSpeed /= max;
                    rightSpeed /= max;
                }

                leftWheel.setPower(leftSpeed);
                rightWheel.setPower(rightSpeed);

                // Display drive status for the driver.
                telemetry.addData("Err/St",  "%5.1f/%5.1f",  error, steer);
                telemetry.addData("Target",  "%7d:%7d",      newLeftTarget,  newRightTarget);
                telemetry.addData("Actual",  "%7d:%7d",      leftWheel.getCurrentPosition(),
                        rightWheel.getCurrentPosition());
                telemetry.addData("Speed",   "%5.2f:%5.2f",  leftSpeed, rightSpeed);
                telemetry.update();
            }

            // Stop all motion;
            leftWheel.setPower(0);
            rightWheel.setPower(0);

            // Turn off RUN_TO_POSITION
            leftWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }



*/
