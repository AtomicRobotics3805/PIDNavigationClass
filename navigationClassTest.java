package com.qualcomm.ftcrobotcontroller.opmodes.FTC3805;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;

public class navigationClassTest extends OpMode {
    navigationPID testPID;
    DcMotor leftMotor;
    DcMotor rightMotor;

    int moveStep = 0;

    //1: 1 = moveForward, 2 = moveBackward, 3 = rotateCW, 4 = rotateCCW, 5 = end navigation
    //2: Tp: the speed at which the robot goes when moving in the correct direction.
    //3: Either the wanted angle, or the goal distance in inches.
    double[][] movementArray = new double[][]{
           //_,______,______}
            {1,  0.25,    12},
            {2, -0.25,   -12},
            {3,  0.25,    90},
            {4,  0.25,   -90},
            {5,     0,     0}
    };

    @Override
    public void init() {
        leftMotor = hardwareMap.dcMotor.get("LM");
        rightMotor = hardwareMap.dcMotor.get("RM");
        rightMotor.setDirection(DcMotor.Direction.REVERSE);

        testPID = new navigationPID(hardwareMap, "AG", leftMotor, rightMotor);
        testPID.tuneGains(0.015, 0, 0);

        telemetry.addData("Status", "Tuning done");

        //Reset the motor encoders
        leftMotor.setMode(DcMotorController.RunMode.RESET_ENCODERS);
        rightMotor.setMode(DcMotorController.RunMode.RESET_ENCODERS);

        leftMotor.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
        rightMotor.setMode(DcMotorController.RunMode.RUN_WITHOUT_ENCODERS);
    }

    @Override
    public void start() {
        testPID.initialize();
        telemetry.addData("Status", "Initialization done");
    }

    @Override
    public void loop() {
        telemetry.addData("moveStep", moveStep);

        boolean sendValue = testPID.readyForNewCommand();

        telemetry.addData("readyForNewCommand", sendValue);

        if (sendValue) {
            testPID.move((int) movementArray[moveStep][0], movementArray[moveStep][1], movementArray[moveStep][2]);
            telemetry.addData("Array position", movementArray[moveStep][0] + ", " + movementArray[moveStep][1] + ", " + movementArray[moveStep][2]);

            moveStep+=1;
        }
    }
}