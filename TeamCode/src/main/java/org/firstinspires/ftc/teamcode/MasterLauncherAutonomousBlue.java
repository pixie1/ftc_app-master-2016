/* Copyright (c) 2015 Qualcomm Technologies Inc

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Qualcomm Technologies Inc nor the names of its contributors
may be used to endorse or promote products derived from this software without
specific prior written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 *
 * This is an example LinearOpMode that shows how to use
 * a Modern Robotics Color Sensor.
 *
 * The op mode assumes that the color sensor
 * is configured with a name of "sensor_color".
 *
 * You can use the X button on gamepad1 to toggle the LED on and off.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@Autonomous
public class MasterLauncherAutonomousBlue extends LinearOpMode {
    DcMotor motorFrontRight;
    DcMotor motorFrontLeft;
    DcMotor motorBackRight;
    DcMotor motorBackLeft;
    DcMotor launchR;
    DcMotor launchL;
    ModernRoboticsI2cGyro sensorGyro;
    EncoderMoveUtil encoderMoveUtil;
    AutonomousUtil AutonomousUtil;
    Servo buttonbashL;
    Servo buttonbashR;
    Servo catcherL;
    Servo catcherR;
    ColorSensor colorSensor;    // Hardware Device Object
    ColorSensor lineSensor;
    ModernRoboticsI2cRangeSensor rangeSensor;

    @Override
    public void runOpMode() {
        final int LoopATime = 1000;
        int LoopARep = 0;
        final int RBL = 26;
        ElapsedTime lineLookTime = new ElapsedTime();

        initMotors();

        buttonbashL = hardwareMap.servo.get("servo_3");
        buttonbashR = hardwareMap.servo.get("servo_4");
        catcherL = hardwareMap.servo.get("servo_1");
        catcherR = hardwareMap.servo.get("servo_2");
        sensorGyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");
        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "sensor_range");
        colorSensor = hardwareMap.colorSensor.get("sensor_color");
        lineSensor = hardwareMap.colorSensor.get("sensor_line");

        lineSensor.setI2cAddress(new I2cAddr(0x3a));


        buttonbashL.setPosition(0.1);
        buttonbashR.setPosition(0.8);
        catcherL.setPosition(0.5);
        catcherR.setPosition(0.5);
        sensorGyro.calibrate();
        while (sensorGyro.isCalibrating()) {
            telemetry.addData("gyro sensor is calibrating", "0");
            telemetry.update();
        }
        telemetry.addData("Initialization done", "0");
        telemetry.update();

        encoderMoveUtil = new EncoderMoveUtil(motorFrontRight, motorBackLeft, motorBackRight, motorFrontLeft,
                telemetry, sensorGyro);
        AutonomousUtil = new AutonomousUtil(catcherL, buttonbashL, motorFrontRight, motorBackLeft, motorBackRight, motorFrontLeft, telemetry, sensorGyro, launchR, launchL, buttonbashR, catcherR, null, null, null);
        colorSensor.enableLed(false);
        lineSensor.enableLed(true);
        waitForStart();

        AutonomousUtil.launch(false);
        /*
        encoderMoveUtil.backward(70, 0.5);

        lineLookTime.reset();

        while (lineLookTime.seconds() < 6) {
            launchL.setPower(1);
            launchR.setPower(-1);
            while (lineLookTime.seconds() < 2) {
            }
            catcherL.setPosition(1);
        }
        catcherL.setPosition(0.5);
        buttonbashL.setPosition(1);
        buttonbashR.setPosition(0);
        encoderMoveUtil.turnGyroPrecise(-30, 0.3);
        encoderMoveUtil.backward(30, 0.5);
        encoderMoveUtil.turnGyroPrecise(45, 0.3);
        encoderMoveUtil.backward(70, 0.5);
        */
    }

    private void initMotors() {
        motorFrontRight = hardwareMap.dcMotor.get("motor_1");
        motorBackRight = hardwareMap.dcMotor.get("motor_2");
        motorFrontLeft = hardwareMap.dcMotor.get("motor_3");
        motorBackLeft = hardwareMap.dcMotor.get("motor_4");
        launchR = hardwareMap.dcMotor.get("motor_5");
        launchL = hardwareMap.dcMotor.get("motor_6");

        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        motorBackLeft.setMaxSpeed(2500);
        motorFrontLeft.setMaxSpeed(2500);
        motorBackRight.setMaxSpeed(2500);
        motorFrontRight.setMaxSpeed(2500);
    }
}
