package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


@Autonomous(name = "Navigation IMU")
//@Disabled                            // Uncomment this to add to the opmode list
public class navigationClassTest extends OpMode {
    navigationPID testNavigator;
    DcMotor leftMotor;
    DcMotor rightMotor;

    //1: 1 = moveForward, 2 = moveBackward, 3 = rotateCW, 4 = rotateCCW, 5 = end navigation
    //2: Tp: the speed at which the robot goes when moving in the correct direction.
    //3: Either the wanted angle, or the goal distance in inches.
    double[][] movementArray = new double[][]{
           //_,______,______}
            {1,  0.15,    36},
            {3,  0.15,   -45},
            {1,  0.15,    12},
            {3,  0.15,   -90},
            {1,   0.1,    12},
            {5,     0,     0} //Stop all movements
    };

    @Override
    public void init() {
        leftMotor = hardwareMap.dcMotor.get("LM");
        rightMotor = hardwareMap.dcMotor.get("RM");
        rightMotor.setDirection(DcMotor.Direction.REVERSE);

        testNavigator = new navigationPID(movementArray, this, "AG", leftMotor, rightMotor);
        testNavigator.tuneGains(0.01, 0, 0.02);

        telemetry.addData("Status", "Tuning done");

        //Reset the motor encoders
        leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
