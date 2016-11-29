
package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
        import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
        import com.qualcomm.robotcore.hardware.DcMotor;
        import com.qualcomm.robotcore.hardware.Servo;
        import com.qualcomm.robotcore.robocol.TelemetryMessage;
        import com.qualcomm.robotcore.util.Range;
/**
 * Created by Karine on 10/27/2015.
 */
@TeleOp
public class MasterThroneTeleop extends OpMode {

    DcMotor motorFrontRight;
    DcMotor motorFrontLeft;
    DcMotor motorBackRight;
    DcMotor motorBackLeft;
    DcMotor launchR;
    DcMotor launchL;

    Servo art;
    Servo buttonBash;
    Servo pushl;
    Servo pushr;

    /**
     * Constructor
     */
    public MasterThroneTeleop() {
    }

    @Override
    public void init() {
        //this is where we define all of the motors on the robot.
        motorFrontRight = hardwareMap.dcMotor.get("motor_1");
        motorBackRight = hardwareMap.dcMotor.get("motor_2");
        motorFrontLeft = hardwareMap.dcMotor.get("motor_3");
        motorBackLeft = hardwareMap.dcMotor.get("motor_4");
        launchR = hardwareMap.dcMotor.get("motor_5");
        launchL = hardwareMap.dcMotor.get("motor_6");
        art = hardwareMap.servo.get("servo_1");
        buttonBash = hardwareMap.servo.get("servo_2");
        pushl = hardwareMap.servo.get("servo_3");
        pushr = hardwareMap.servo.get("servo_4");

        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        art.setPosition(.5);
        pushr.setPosition(.5);
        pushl.setPosition(.5);
    }

    @Override
    public void loop() {
        //Control method #2 Joysticks
        double n = -1 * (gamepad1.left_stick_x + gamepad1.left_stick_y);
        double m = gamepad1.left_stick_y - gamepad1.left_stick_x;
        motorFrontRight.setPower(n);
        motorBackRight.setPower(n);
        motorFrontLeft.setPower(m);
        motorBackLeft.setPower(m);

        if (gamepad2.x) {
            art.setPosition(.9);
        } else if (gamepad2.b) {
            art.setPosition(.1);
        } else {
            art.setPosition(.5);
        }

        if (gamepad2.dpad_up) { //Button Basher
            buttonBash.setPosition(0.75);
        } if (gamepad2.dpad_down) {
            buttonBash.setPosition(0);
        }

//        if (gamepad2.a) {
//            launchL.setPower(1);
//            launchR.setPower(-1);
//            //pushl.setPosition(0.47);
//            //pushr.setPosition(0.63);
//        } if (gamepad2.y) {
//            launchL.setPower(0);
//            launchR.setPower(0);
//            //pushl.setPosition(0.9); //DEPLOY CHANGES DANGIT
//            //pushr.setPosition(0.2);
//        }

//        if (gamepad2.a) {
//            launchL.setPower(1);
//            launchR.setPower(-1);
//            //pushl.setPosition(0.47);
//            //pushr.setPosition(0.63);
//        } else if (gamepad2.y) {
//            launchL.setPower(0);
//            launchR.setPower(0);
//            //pushl.setPosition(0.9); //DEPLOY CHANGES DANGIT
//            //pushr.setPosition(0.2);
//        }

        if (gamepad2.x) {
            art.setPosition(.9);
        } else if (gamepad2.b) {
            art.setPosition(.1);
        } else {
            art.setPosition(.5);
        }

        if (gamepad2.dpad_up) { //Button Basher
            buttonBash.setPosition(0.75);
        } if (gamepad2.dpad_down) {
            buttonBash.setPosition(0);
        }

        if (gamepad2.a) {
            pushl.setPosition(0.05);//0.47
            pushr.setPosition(0.95);
        } if (gamepad2.y) {
            pushl.setPosition(0.95); //DEPLOY CHANGES DANGIT
            pushr.setPosition(0.05);
        }
    }
}

