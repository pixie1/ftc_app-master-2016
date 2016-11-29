package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * A simple test of a pair of motors
 */
@Autonomous
public class MasterThroneAutoRampBlue extends LinearOpMode {
    DcMotor motorFrontRight;
    DcMotor motorFrontLeft;
    DcMotor motorBackRight;
    DcMotor motorBackLeft;

    ModernRoboticsI2cGyro sensorGyro;
    EncoderMoveUtil encoderMoveUtil;

    @Override
    public void runOpMode() throws InterruptedException {
        final int RBL = 26; //robot length, to be used when against wall.
        motorFrontRight = hardwareMap.dcMotor.get("motor_1");
        motorBackRight = hardwareMap.dcMotor.get("motor_2");
        motorFrontLeft = hardwareMap.dcMotor.get("motor_3");
        motorBackLeft = hardwareMap.dcMotor.get("motor_4");
        sensorGyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");

        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sensorGyro.calibrate();
        while (sensorGyro.isCalibrating()){
            telemetry.addData("gyro sensor is calibrating","0");
            telemetry.update();
        }
        telemetry.addData("Initialization done","0");
        telemetry.update();

        encoderMoveUtil= new EncoderMoveUtil(motorFrontRight, motorBackLeft, motorBackRight, motorFrontLeft,
                telemetry, sensorGyro);

        //Robot length: 26cm
        encoderMoveUtil.forward(40-RBL,0.25); //position robot
        encoderMoveUtil.turnGyro(40,0.25); //turn parrallel to ramp
        encoderMoveUtil.forward(70,0.25); //position near center of ramp
        encoderMoveUtil.turnGyro(80,0.25); //face ramp
        encoderMoveUtil.forward(30,0.3); //get on ramp

    }
}

