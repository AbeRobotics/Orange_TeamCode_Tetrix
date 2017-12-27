package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.configuration.MotorConfigurationType;
import com.qualcomm.robotcore.hardware.configuration.MotorType;

import org.firstinspires.ftc.robotcore.external.navigation.Rotation;

/**
 * Created by harsh.joshi on 24-Dec-2017.
 */
@Autonomous(name="Encoder Test", group="Autonomous")
public class Testing_DCMotorEncoder extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor leftWheel;
        DcMotor rightWheel;
        leftWheel = hardwareMap.dcMotor.get("left_wheel");
        rightWheel = hardwareMap.dcMotor.get("right_wheel");
        rightWheel.setDirection(DcMotorSimple.Direction.REVERSE);
        leftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightWheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        int leftWheelPosition = leftWheel.getCurrentPosition();
        leftWheel.setPower(1);
        rightWheel.setPower(1.0);
        leftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftWheel.setTargetPosition(-14400);
        MotorConfigurationType motorType = leftWheel.getMotorType();

        Rotation rotation =  motorType.getOrientation();
        int rotationOrdinal = rotation.ordinal();
        while(opModeIsActive()) {
            sleep(2000);
            rightWheel.setPower(0.0);
            rightWheel.setDirection(DcMotorSimple.Direction.FORWARD);
            sleep(500);
            rightWheel.setPower(1.0);
            if(leftWheel.isBusy())
            {
                telemetry.addData("LeftWheelPosition ",leftWheel.getCurrentPosition());
                telemetry.update();
                leftWheel.setPower(0.3);
            }
            else
            {
                leftWheel.setPower(0.0);
            }
            rightWheel.setPower(0.3);
        }
        leftWheel.setPower(0);
        rightWheel.setPower(0);
    }
}
