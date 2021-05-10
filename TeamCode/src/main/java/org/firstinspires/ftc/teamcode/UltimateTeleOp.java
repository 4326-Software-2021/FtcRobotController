package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;


import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="ult-op", group="Iterative Opmode")
public class UltimateTeleOp extends OpMode {
    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftBack;
    DcMotor rightBack;

    DcMotor railMotor;

    Servo wobbleExtender;
    Servo wobbleGrabber;
    Servo ringExtender;
    Servo ringGrabber;

    float rail;

//    DcMotor lowArm;
//    Servo clawServo;
//
//    DcMotor wobbleArm;

    double x;
    double y;

    boolean grip = false;

    @Override
    public void init() {
        //setting up the motors and servos
        rightFront = hardwareMap.dcMotor.get("front_right");
        leftFront = hardwareMap.dcMotor.get("front_left");
        rightBack = hardwareMap.dcMotor.get("back_right");
        leftBack = hardwareMap.dcMotor.get("back_left");

        railMotor = hardwareMap.dcMotor.get("rail_motor");

        wobbleExtender = hardwareMap.servo.get("wobble_extender");
        wobbleGrabber = hardwareMap.servo.get("wobble_grabber");
        ringExtender = hardwareMap.servo.get("ring_extender");
        ringGrabber = hardwareMap.servo.get("ring_grabber");

//        lowArm = hardwareMap.dcMotor.get("low_arm"); //motor on bottom of arm for lower goal
//        clawServo = hardwareMap.servo.get("claw_servo"); //claw on top of arm for lower goal
//
//        wobbleArm = hardwareMap.dcMotor.get("wobble_arm"); //motor for wobble goal arm

        x=0;
        y=1;

    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {

        //GAMEPAD 1
        //Drive train

        float drive = gamepad1.left_stick_y;
        float strafe = gamepad1.right_stick_x;
        float turn = gamepad1.left_stick_x;

        float fl = strafe - turn + drive;
        float fr = strafe + turn - drive;
        float bl = strafe + turn + drive;
        float br = strafe - turn - drive;

        leftFront.setPower(fl);
        rightFront.setPower(fr);
        leftBack.setPower(bl);
        rightBack.setPower(br);

////    //GAMEPAD 2

        //XRail
        Boolean controllable = true;
        if (gamepad2.dpad_up){
            controllable = !controllable;
        }
        if (controllable){
            rail = gamepad2.right_stick_y;
        }
        railMotor.setPower(rail/3);


        //Wobble Mech
//        wobbleExtender.setPosition(gamepad2.left_stick_x);
        if (gamepad2.y){
            wobbleExtender.setPosition(0.65);
        }
        if (gamepad2.x){
            wobbleExtender.setPosition(0.35);
        }
        if (gamepad2.a) {
            wobbleGrabber.setPosition(0.65);
        }
        else if(gamepad2.b){
            wobbleGrabber.setPosition(0);
        }

        //Ring Mech
        if (gamepad2.left_trigger>0.1){
            ringGrabber.setPosition(0.65);
        }
        if (gamepad2.right_trigger>0.1){
            ringGrabber.setPosition(0.35);
        }
        if (gamepad2.right_bumper){
            ringExtender.setPosition(0.65);
        }
        if (gamepad2.left_bumper){
            ringExtender.setPosition(0.35);
        }
      }
}
