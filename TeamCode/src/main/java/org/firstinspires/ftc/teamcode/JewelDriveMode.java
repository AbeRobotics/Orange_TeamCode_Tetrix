package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;


/**
 * Created by Akanksha.Joshi on 23-Dec-2017.
 */

public class JewelDriveMode {

    private static JewelDriveMode jewelDriveInstance;
    OPModeConstants.FireSequence fireSequence;
    private JewelDriveMode()
    {}
    public static JewelDriveMode jewelDriveModeInstance()
    {
        if(jewelDriveInstance == null)
        {
            jewelDriveInstance = new JewelDriveMode();
        }
        return jewelDriveInstance;
    }
    public boolean performJewelRemovalTask(OPModeConstants.FireSequence fireSequence, HardwareMap hardwareMap)
    {

        DcMotor leftWheel;
        DcMotor rightWheel;
        leftWheel = hardwareMap.dcMotor.get("left_wheel");
        rightWheel = hardwareMap.dcMotor.get("right_wheel");


        leftWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        int rightWheelPosition = rightWheel.getCurrentPosition();
        int leftWheelPosition = leftWheel.getCurrentPosition();
        leftWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftWheel.setPower(0.75);
        rightWheel.setPower(0.75);


        //FWD - AKA Right wheel negative, left wheel positive
        if(fireSequence == OPModeConstants.FireSequence.FORWARD) {
            //one rotation
            leftWheel.setTargetPosition(1440);
            rightWheel.setTargetPosition(-1440);
        }
        //Back - AKA Right wheel positive, left wheel negative
        if(fireSequence == OPModeConstants.FireSequence.BACKWARD) {
            leftWheel.setTargetPosition(-1440);
            rightWheel.setTargetPosition(1440);
        }

        while(leftWheel.isBusy())
        {

        }
        leftWheel.setPower(0);
        rightWheel.setPower(0);
        leftWheel.setTargetPosition(leftWheelPosition);
        rightWheel.setTargetPosition(rightWheelPosition);
        leftWheel.setPower(0.75);
        rightWheel.setPower(0.75);
        while(leftWheel.isBusy())
        {

        }
        leftWheel.setPower(0);
        rightWheel.setPower(0);
        return true;
    }
    public void Reset()
    {
        jewelDriveInstance = null;
    }
}
