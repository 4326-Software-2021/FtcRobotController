package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.ColorSenseStopState;
import org.firstinspires.ftc.teamcode.StateMachine;
import org.firstinspires.ftc.teamcode.adjustPulleyState;
import org.firstinspires.ftc.teamcode.teamcode.driveState;

import java.util.ArrayList;

@Autonomous(name="RingMachine", group="Iterative Opmode")
public class RingMachine extends OpMode {
    DcMotor leftFront;
    DcMotor rightFront;

    DcMotor leftBack;
    DcMotor rightBack;
    DcMotor pulley;
    
    Servo ringExtender;
    Servo ringGrabber;
    Servo wobbleExtender;
    Servo wobbleGrabber;

    BNO055IMU imu;
    ColorSensor colorSensor;

    driveState moveForward;
    adjustPulleyState dropRings;
    ColorSenseStopState parkReverse;

    StateMachine machine;
    @Override
    public void init() {
        rightFront = hardwareMap.dcMotor.get("front_right");
        leftFront = hardwareMap.dcMotor.get("front_left");
        rightBack = hardwareMap.dcMotor.get("back_right");
        leftBack = hardwareMap.dcMotor.get("back_left");
        pulley = hardwareMap.dcMotor.get("rail_motor");
        ringExtender = hardwareMap.servo.get("ring_extender");
        ringGrabber = hardwareMap.servo.get("ring_grabber");
        wobbleExtender = hardwareMap.servo.get("wobble_extender");
        wobbleGrabber = hardwareMap.servo.get("wobble_grabber");
        colorSensor = hardwareMap.get(ColorSensor.class, "color");

        ArrayList<DcMotor> motors = new ArrayList<DcMotor>();
        motors.add(rightFront);
        motors.add(leftFront);
        motors.add(rightBack);
        motors.add(leftBack);
        motors.add(pulley);

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE); //leftFront
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;


        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);
        
        
        //state setup
        //moveForward = new driveState(.5, .5, motors, "forward");
        //dropRings = new adjustPulleyState(10, .5, motors, ringExtender, ringGrabber);
        parkReverse = new ColorSenseStopState(motors, colorSensor, "yellow", 5.0, "backward");
        
        //moveForward.setNextState(dropRings);
        //dropRings.setNextState(parkReverse);
    }

    @Override
    public void start(){
        machine = new StateMachine(parkReverse);
        machine.update();
    }

    @Override
    public void loop(){
        machine.update();
    }       
}