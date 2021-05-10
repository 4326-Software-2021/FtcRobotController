package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.GyroTurnCCWByPID;
import org.firstinspires.ftc.teamcode.GyroTurnCWByPID;
import org.firstinspires.ftc.teamcode.StateMachine; //necessary
import org.firstinspires.ftc.teamcode.StateMachine.State; //necessary
import org.firstinspires.ftc.teamcode.teamcode.GyroTurnCCWByPID;
import org.firstinspires.ftc.teamcode.teamcode.GyroTurnCWByPID;
import org.firstinspires.ftc.teamcode.teamcode.driveState;

import java.util.ArrayList;
import java.util.Locale;
import com.qualcomm.hardware.*;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name="UltimatePark", group="Iterative Opmode")
public class UltimatePark extends OpMode {
    private StateMachine machine;
    org.firstinspires.ftc.teamcode.teamcode.driveState moveForward;
    org.firstinspires.ftc.teamcode.teamcode.driveState park;
    GyroTurnCCWByPID leftTurn;
    GyroTurnCWByPID rightTurn;

    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftBack;
    DcMotor rightBack;

    BNO055IMU imu;

    @Override
    public void init() {
        rightFront = hardwareMap.dcMotor.get("front_right");
        leftFront = hardwareMap.dcMotor.get("front_left");
        rightBack = hardwareMap.dcMotor.get("back_right");
        leftBack = hardwareMap.dcMotor.get("back_left");

        ArrayList<DcMotor> motors = new ArrayList<DcMotor>();
        motors.add(rightFront);
        motors.add(leftFront);
        motors.add(rightBack);
        motors.add(leftBack);

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE); //leftFront
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;

        imu = hardwareMap.get(BNO055IMU.class, "imu"); // lmao hardware what a joke

        imu.initialize(parameters);

        //State set up



        park = new driveState(((double)10/64), 0.05, motors, "forward");





        park.setNextState(null);

        //yay!

    }

    @Override
    public void start(){
        machine = new StateMachine(park);
        machine.update();
    }

    @Override
    public void loop(){
        machine.update();
    }
}