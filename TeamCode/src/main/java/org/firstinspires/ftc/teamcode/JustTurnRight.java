package org.firstinspires.ftc.teamcode;
//This is a test for state machines, iterative opmode

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;


import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.ColorSenseStopState;
import org.firstinspires.ftc.teamcode.GyroTurnCWByPID;
import org.firstinspires.ftc.teamcode.OnlyClaspState;
import org.firstinspires.ftc.teamcode.StateMachine; //necessary
import org.firstinspires.ftc.teamcode.StateMachine.State; //necessary
import java.util.ArrayList;
import java.util.Locale;
import com.qualcomm.robotcore.hardware.Servo;

import android.hardware.camera2.CameraDevice;
import android.hardware.camera2.*;

import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.adjustPulleyState;
import org.firstinspires.ftc.teamcode.teamcode.GyroTurnCWByPID;
import org.firstinspires.ftc.teamcode.teamcode.OnlyClaspState;
import org.firstinspires.ftc.teamcode.teamcode.StateMachine;
import org.firstinspires.ftc.teamcode.teamcode.driveState;
import org.firstinspires.ftc.teamcode.teamcode.timeState;


@Autonomous(name="Move A Little", group="Iterative Opmode")
public class JustTurnRight extends OpMode{

    Servo leftHand;
    Servo rightHand;
    Servo clasp;
    BNO055IMU imu;

    adjustPulleyState adjustPulley;
    org.firstinspires.ftc.teamcode.teamcode.driveState driveToFoundation;
    org.firstinspires.ftc.teamcode.teamcode.driveState straighten;
    org.firstinspires.ftc.teamcode.teamcode.driveState getOffWall;
    timeState driveBack;
    OnlyClaspState foundationClasp;
    OnlyClaspState releaseClasp;
    org.firstinspires.ftc.teamcode.teamcode.driveState dragFoundationIn;
    ColorSenseStopState parkUnderBridge2;
    GyroTurnCWByPID turnRightAngle;

    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftBack;
    DcMotor rightBack;

    ColorSensor colorSensor;

    //Setting up the order
    DcMotor pulley;


    public void init(){


        colorSensor = hardwareMap.colorSensor.get("colorSensor");
        rightHand = hardwareMap.servo.get("right");
        leftHand = hardwareMap.servo.get("left");
        rightFront = hardwareMap.dcMotor.get("right front");
        leftFront = hardwareMap.dcMotor.get("left front");
        rightBack = hardwareMap.dcMotor.get("right back");
        leftBack = hardwareMap.dcMotor.get("left back");
        pulley = hardwareMap.dcMotor.get("pulley");
        clasp = hardwareMap.servo.get("clasp");
        ArrayList<DcMotor> motors = new ArrayList<>();
        motors.add(rightFront);
        motors.add(leftFront);
        motors.add(rightBack);
        motors.add(leftBack);
        motors.add(pulley);

        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        driveToFoundation = new org.firstinspires.ftc.teamcode.teamcode.driveState(2, .4, motors, "forward");

        adjustPulley = new adjustPulleyState(.25,-.5 ,motors, rightHand, leftHand );

        foundationClasp = new OnlyClaspState(clasp, 2,  1.2);


        parkUnderBridge2 = new ColorSenseStopState(motors, colorSensor, "red", .225, "backward");

        turnRightAngle = new GyroTurnCWByPID(90, .5, motors, imu);
        driveBack = new timeState(5, .5, motors, "backward");

        releaseClasp = new OnlyClaspState(clasp, 2, 0 );
        straighten = new org.firstinspires.ftc.teamcode.teamcode.driveState(5,.5, motors, "left");//right
        getOffWall = new org.firstinspires.ftc.teamcode.teamcode.driveState(2, .5 , motors, "right");//left
        dragFoundationIn = new driveState(5, .5 , motors, "left");//right



        //Sequence

        driveToFoundation.setNextState(null);
    }

    @Override
    public void start(){
        machine = new StateMachine(driveToFoundation);

    }
    private StateMachine machine;
    public void loop()  {


        telemetry.addData("state", machine.currentState());
        telemetry.addData("state", machine.currentState());

        telemetry.update();
        machine.update();

    }
}
