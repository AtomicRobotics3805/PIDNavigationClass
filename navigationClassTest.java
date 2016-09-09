package com.qualcomm.ftcrobotcontroller.opmodes.FTC3805;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;

public class navigationClassTest extends OpMode {
    navigationPID testNavigator;
    DcMotor leftMotor;
    DcMotor rightMotor;

    //1: 1 = moveForward, 2 = moveBackward, 3 = rotateCW, 4 = rotateCCW, 5 = end navigation
    //2: Tp: the speed at which the robot goes when moving in the correct direction.
    //3: Either the wanted angle, or the goal distance in inches.
    double[][] movementArray = new double[][]{
            //_,______,______}
            {1,   0.15,    12}, //Move forward 12 inches at 15% power
            {2,  -0.15,   -12}, //Move backward 12 inches at 15% power
            {3,   0.15,    90}, //Rotate CW to 90 degrees at 15% power
            {4,   0.15,   -90}, //Rotate CCW to -90 degrees at 15% power

            {5,      0,     0} //Stop all movements
    };

    @Override
    public void init() {
        leftMotor = hardwareMap.dcMotor.get("LM");
        rightMotor = hardwareMap.dcMotor.get("RM");
        rightMotor.setDirection(DcMotor.Direction.REVERSE);

        testNavigator = new navigationPID(movementArray, hardwareMap, "AG", leftMotor, rightMotor);
        testNavigator.tuneGains(0.05, 0, 0.1);

        telemetry.addData("Status", "Tuning done");

        //Reset the motor encoders
        leftMotor.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        rightMotor.setMode(DcMotorController.RunMode.RESET_ENCODERS);

        leftMotor.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        rightMotor.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
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
        telemetry.addData("Left", leftMotor.getCurrentPosition());
        telemetry.addData("Right", rightMotor.getCurrentPosition());
    }
}
