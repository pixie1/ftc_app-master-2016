 package org.firstinspires.ftc.teamcode;

 import com.qualcomm.robotcore.eventloop.opmode.OpMode;
 import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
 import com.qualcomm.robotcore.hardware.DcMotor;
 import com.qualcomm.robotcore.hardware.Servo;
 import com.qualcomm.robotcore.util.ElapsedTime;

 /**
 * Created by Karine on 10/27/2015.
 */
@TeleOp
public class MasterThroneTeleopNew extends OpMode {

    DcMotor motorFrontRight;
    DcMotor motorFrontLeft;
    DcMotor motorBackRight;
    DcMotor motorBackLeft;
    DcMotor launchR;
    DcMotor launchL;

    Servo catcher;
    Servo buttonBasher;

    /**
     * Constructor
     */
    public MasterThroneTeleopNew() {
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
        catcher = hardwareMap.servo.get("servo_1");
        buttonBasher = hardwareMap.servo.get("servo_2");
       // pushl = hardwareMap.servo.get("servo_3");
        //pushr = hardwareMap.servo.get("servo_4");

        motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        launchL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        launchR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        catcher.setPosition(.9);
        buttonBasher.setPosition(1);
       // pushr.setPosition(.5);
       // pushl.setPosition(.5);
    }
     final double ARTT = 0.3;
    @Override
    public void loop() {
        //Control method #2 Joysticks
        double n = ((gamepad1.left_stick_x + gamepad1.left_stick_y))/.8;
        double m = (-(gamepad1.left_stick_y - gamepad1.left_stick_x))/.8;
        motorFrontRight.setPower(n);
        motorBackRight.setPower(n);
        motorFrontLeft.setPower(m);
        motorBackLeft.setPower(m);

        if (gamepad2.x) {
            catcher.setPosition(0.9);
        } else if (gamepad2.b) {
            catcher.setPosition(0);
        }
        if (gamepad2.dpad_up) { //Button Basher
            buttonBasher.setPosition(1);
        } if (gamepad2.dpad_down) {
            buttonBasher.setPosition(0);
        }

        if (gamepad2.a) {
            ElapsedTime flingerCounter = new ElapsedTime();
            launchR.setPower(1); // prep
            launchL.setPower(-1);
            flingerCounter.reset();
            while (flingerCounter.time() < ARTT/2){}
            launchL.setPower(1); //hit ball
            launchR.setPower(-1);
            flingerCounter.reset();
            while (flingerCounter.time() < ARTT){}
            launchR.setPower(1); // return
            launchL.setPower(-1);
            flingerCounter.reset();
            while (flingerCounter.time() < ARTT/2){}
            launchL.setPower(0);
            launchR.setPower(0);
        }
        //} if (gamepad2.y) {
        //    pushl.setPosition(0); //DEPLOY CHANGES DANGIT
        //    pushr.setPosition(1);

        //}

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

    }
}
