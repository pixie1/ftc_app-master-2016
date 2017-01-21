/*
Copyright (c) 2016 Robert Atkinson

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted (subject to the limitations in the disclaimer below) provided that
the following conditions are met:

Redistributions of source code must retain the above copyright notice, this list
of conditions and the following disclaimer.

Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

Neither the name of Robert Atkinson nor the names of his contributors may be used to
endorse or promote products derived from this software without specific prior
written permission.

NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESSFOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.TelemetryImpl;

/**
 * A simple test of a pair  of motors
 */
@Autonomous
public class autonomousTest extends LinearOpMode {

    DcMotor motorFrontRight;
    DcMotor motorFrontLeft;
    DcMotor motorBackRight;
    DcMotor motorBackLeft;

    ModernRoboticsI2cGyro sensorGyro;
    ModernRoboticsI2cRangeSensor rangeSensor;
    EncoderMoveUtil encoderMoveUtil;
    ElapsedTime eTime;

    @Override
    public void runOpMode() throws InterruptedException {

        int y = 666;
        telemetry.addData("DAN IS A SATANIST", y);
        telemetry.update();
        final int RBL = 26; //robot length, to be used when against wall.
        motorFrontRight = hardwareMap.dcMotor.get("motor_1");
        motorBackRight = hardwareMap.dcMotor.get("motor_2");
        motorFrontLeft = hardwareMap.dcMotor.get("motor_3");
        motorBackLeft = hardwareMap.dcMotor.get("motor_4");
        sensorGyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");
        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "sensor_range");
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        eTime = new ElapsedTime();
        eTime.reset();


    //    forward(100-RBL, 0.25);
        ColorSensor colorSensor;    // Hardware Device Object
        ColorSensor lineSensor= lineSensor = hardwareMap.colorSensor.get("sensor_line");
        colorSensor = hardwareMap.colorSensor.get("sensor_color");
        OpticalDistanceSensor lightSensor= hardwareMap.opticalDistanceSensor.get("light_sensor");
        lineSensor.setI2cAddress(new I2cAddr(0x3a));
        lightSensor.enableLed(true);
      //  colorSensor.setI2cAddress(new I2cAddr(0x3c));



        while (eTime.seconds() < 30) {
            telemetry.addData("distance OpticalCM", rangeSensor.cmOptical());
            telemetry.addData("distance UltrasoundCM", rangeSensor.cmUltrasonic());
            telemetry.update();
        }
//        double speed=0.25;
//
//        motorBackRight.setPower(-speed);
//        eTime.reset();
//        while(eTime.time()<5){
//            //telemetry.addData("Front Left currentEncoderValue", motorFrontLeft.getCurrentPosition());
//            telemetry.addData("Back Right currentEncoderValue", motorBackRight.getCurrentPosition());
//            telemetry.update();
//        }
//        motorBackRight.setPower(0);
//        while(eTime.time()<5){
//            telemetry.addData("Back Right currentEncoderValue", motorBackRight.getCurrentPosition());
//            telemetry.update();
//        }
//        motorBackRight.setPower(speed);
//        eTime.reset();
//        while(eTime.time()<5){
//            //telemetry.addData("Front Left currentEncoderValue", motorFrontLeft.getCurrentPosition());
//            telemetry.addData("Back Right currentEncoderValue", motorBackRight.getCurrentPosition());
//            telemetry.update();
//        }
//        motorBackRight.setPower(0);
//        while(eTime.time()<5){
//            telemetry.addData("Back Right currentEncoderValue", motorBackRight.getCurrentPosition());
//            telemetry.update();
//        }
//        telemetry.addData("start back left", "0");
//        telemetry.update();
//        motorBackLeft.setPower(-speed);
//        eTime.reset();
//        while(eTime.time()<5){
//            //telemetry.addData("Front Left currentEncoderValue", motorFrontLeft.getCurrentPosition());
//            telemetry.addData("Back Left currentEncoderValue", motorBackLeft.getCurrentPosition());
//            telemetry.update();
//        }
//        motorBackLeft.setPower(speed);
//        eTime.reset();
//        while(eTime.time()<5){
//            //telemetry.addData("Front Left currentEncoderValue", motorFrontLeft.getCurrentPosition());
//            telemetry.addData("Back Left currentEncoderValue", motorBackLeft.getCurrentPosition());
//            telemetry.update();
//        }
//        motorBackLeft.setPower(0);
//        while(eTime.time()<5){
//            telemetry.addData("Back Left currentEncoderValue", motorBackLeft.getCurrentPosition());
//            telemetry.update();
//        }
//        telemetry.addData("start Front Left", 0);
//        telemetry.update();
//        motorFrontLeft.getCurrentPosition();
//
//        telemetry.addData("Front Left currentEncoderValue", motorFrontLeft.getCurrentPosition());
//        telemetry.update();
//
//        motorFrontLeft.setPower(-speed);
//        eTime.reset();
//        while(eTime.time()<5){
//            telemetry.addData("Front Left currentEncoderValue", motorFrontLeft.getCurrentPosition());
//            telemetry.update();
//        }
//        motorFrontLeft.setPower(0);
//        motorFrontLeft.setPower(speed);
//
//        eTime.reset();
//        while(eTime.time()<5){
//            telemetry.addData("Front Left currentEncoderValue", motorFrontLeft.getCurrentPosition());
//            telemetry.update();
//        }
//        motorFrontLeft.setPower(0);
//        telemetry.addData("Front Left currentEncoderValue", motorFrontLeft.getCurrentPosition());
//        telemetry.update();
//
//      //  current = motorFrontRight.getCurrentPosition();
//
//        while(eTime.time()<5){
//            telemetry.addData("Front Left currentEncoderValue", motorFrontLeft.getCurrentPosition());
//            telemetry.addData("Front Right currentEncoderValue", motorFrontRight.getCurrentPosition());
//            telemetry.update();
//        }
//
//        motorFrontRight.setPower(-speed);
//        eTime.reset();
//        while(eTime.time()<5){
//            telemetry.addData("Front Left currentEncoderValue", motorFrontLeft.getCurrentPosition());
//            telemetry.addData("Front Right currentEncoderValue", motorFrontRight.getCurrentPosition());
//            telemetry.update();
//        }
//        motorFrontRight.setPower(0);
//        while(eTime.time()<5){
//            telemetry.addData("Front Right currentEncoderValue", motorFrontRight.getCurrentPosition());
//            telemetry.update();
//        }



    }
}

