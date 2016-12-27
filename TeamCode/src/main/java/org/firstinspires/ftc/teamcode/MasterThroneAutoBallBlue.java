package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * A simple test of a pair of motors
 */
@Autonomous(name="MasterThroneAutoBallBlue", group="Master")
public class MasterThroneAutoBallBlue extends LinearOpMode {

    DcMotor motorFrontRight;
    DcMotor motorFrontLeft;
    DcMotor motorBackRight;
    DcMotor motorBackLeft;

    Servo catcher;
    Servo buttonbash;

    ModernRoboticsI2cGyro sensorGyro;
    EncoderMoveUtil encoderMoveUtil;

    @Override
    public void runOpMode() throws InterruptedException {

        final int RBL = 26; //robot length, to be used when against wall

        motorFrontRight = hardwareMap.dcMotor.get("motor_1");
        motorBackRight = hardwareMap.dcMotor.get("motor_2");
        motorFrontLeft = hardwareMap.dcMotor.get("motor_3");
        motorBackLeft = hardwareMap.dcMotor.get("motor_4");
        sensorGyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");
        AutonomousUtil AutonomousUtil;
        catcher = hardwareMap.servo.get("servo_1");
        buttonbash = hardwareMap.servo.get("servo_2");

        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        sensorGyro.calibrate();
        while (sensorGyro.isCalibrating()) {
            telemetry.addData("gyro sensor is calibrating", "0");
            telemetry.update();
        }
        telemetry.addData("Initialization done", "0");
        telemetry.update();
        waitForStart();

        //encoderMoveUtil = new EncoderMoveUtil(motorFrontRight, motorBackLeft, motorBackRight, motorFrontLeft,
        //        telemetry, sensorGyro);

        //Robot length: 26cm
        AutonomousUtil = new AutonomousUtil(catcher, buttonbash, motorFrontRight, motorBackLeft, motorBackRight, motorFrontLeft, telemetry, sensorGyro);
        AutonomousUtil.hitBall(false);
    }
}

