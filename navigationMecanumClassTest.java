package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


@Autonomous(name = "Mecanum IMU")
//@Disabled
public class navigationMecanumClassTest extends OpMode {
    navigationMecanumPID testNavigator;
    DcMotor rightFront;
    DcMotor rightBack;
    DcMotor leftFront;
    DcMotor leftBack;

    //1: 1 = moveForward, 2 = moveBackward, 3 = rotateCW, 4 = rotateCCW, 5 = moveLeft, 6 = moveRight, 10 = end navigation
    //2: Tp: the speed at which the robot goes when moving in the correct direction.
    //3: Either the wanted angle, or the goal distance in inches.
    double[][] movementArray = new double[][]{
           //_,______,______}
            {1,   0.5,    24},
            {2,  -0.5,   -24},
            {3,   0.5,    90},
            {4,  -0.5,     0},
            {5,   0.5,    12},
            {6,  -0.5,   -12},
            {10,    0,     0} //Stop all movements
    };

    @Override
    public void init() {
        rightFront = hardwareMap.dcMotor.get("RF");
        rightBack = hardwareMap.dcMotor.get("RB");
        leftFront = hardwareMap.dcMotor.get("LF");
        leftBack = hardwareMap.dcMotor.get("LB");

        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);

        testNavigator = new navigationMecanumPID(movementArray, this, "AG", leftFront, rightFront, leftBack, rightBack, 1120, 3);
        testNavigator.tuneGains(0.01, 0, 0.02);

        telemetry.addData("Status", "Tuning done");

        //Reset the motor encoders
        resetEncoders();
        runWithoutEncoders();
    }

    @Override
    public void start() {
        testNavigator.initialize();
    }

    @Override
    public void loop() {
        testNavigator.loopNavigation();

        telemetry.addData("Current navigation step", testNavigator.navigationStep());
        telemetry.addData("Current navigation type", testNavigator.navigationType());
        telemetry.addData("Current Adafruit Heading", testNavigator.currentHeading());
        telemetry.addData("Left front", leftFront.getCurrentPosition());
        telemetry.addData("Left back", leftBack.getCurrentPosition());
        telemetry.addData("Right front", rightFront.getCurrentPosition());
        telemetry.addData("Right back", rightBack.getCurrentPosition());
    }

    public void resetEncoders() {
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void runWithoutEncoders() {
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}