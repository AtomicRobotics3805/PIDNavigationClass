package com.qualcomm.ftcrobotcontroller.opmodes.FTC3805;

import android.util.Log;

import com.qualcomm.ftcrobotcontroller.opmodes.AdafruitIMU;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;


public final class navigationPID {
    private AdafruitIMU AdafruitGyro; //Instance of AdafruitIMU
    private volatile double[] rollAngle = new double[2], pitchAngle = new double[2], yawAngle = new double[2];  //Array to store IMU's output

    private DcMotor leftMotor; //Instance of DcMotor
    private DcMotor rightMotor; //Instance of DcMotor

    private double[][] movementArray;
    private int movementArrayStep = 0;

    //Variables for PID loop
    private double setPoint = 0;
    private double Kp;
    private double Ki;
    private double Kd;
    private double integral = 0;
    private double preError;


    private int encoderTicksPerRev = 1440; //Change this to match the specs of the motors being used
    private double encoderTicksPerInch = encoderTicksPerRev / (Math.PI * 3); //Variable to convert encoder ticks to inches
    private int encoderPositionReference; //Variable to use as reference for moveForward and moveBackward

    //Constructor
    public navigationPID(double [][] movementCommandArray, HardwareMap currentIMUHWmap, String configuredIMUname, DcMotor leftMotor, DcMotor rightMotor) {
        //Map the IMU instance pointer to the actual hardware
        try {
            AdafruitGyro = new AdafruitIMU(currentIMUHWmap, configuredIMUname, (byte) (AdafruitIMU.BNO055_ADDRESS_A * 2),
                    (byte) AdafruitIMU.OPERATION_MODE_IMU);
        } catch (RobotCoreException e) {
            Log.i("FtcRobotController", "Exception: " + e.getMessage());
        }

        //Assign the DcMotor instances to those of the main class
        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;

        movementArray = movementCommandArray;
    }

    //**********Public methods***********

    public void initialize() {
        AdafruitGyro.startIMU(); //Prepare the IMU for I2C communication

        encoderPositionReference = (leftMotor.getCurrentPosition() + rightMotor.getCurrentPosition()) / 2; //Set encoderPositionReference to the mean value of the current motor positions
    }

    public void tuneGains(double newKp, double newKi, double newKd) {
        //Set the gains for the PID loop
        Kp = newKp;
        Ki = newKi;
        Kd = newKd;
    }

    public void loopNavigation() {
        AdafruitGyro.getIMUGyroAngles(rollAngle, pitchAngle, yawAngle);

        move((int) movementArray[movementArrayStep][0], movementArray[movementArrayStep][1], movementArray[movementArrayStep][2]);
    }

    public int navigationStep() {
        return movementArrayStep;
    }

    public int navigationType() {
        return (int) movementArray[movementArrayStep][0];
    }

    public double currentHeading() {
        return yawAngle[0];
    }

    public void forceNextMovement() {
        movementArrayStep++;
    }

    //**********Private methods**********

    private void move(int movementType, double Tp, double goalValue) {
        switch (movementType) {
            case 1:
                moveForward(goalValue, Tp);
                break;
            case 2:
                moveBackward(goalValue, Tp);
                break;
            case 3:
                rotateCW(goalValue, Tp);
                break;
            case 4:
                rotateCCW(goalValue, Tp);
                break;
            case 5:
                leftMotor.setPower(0);
                rightMotor.setPower(0);
                break;
        }
    }

    private void moveForward(double goalDistanceInch, double Tp) { //Movement val == 1
        int distanceTraveled = (leftMotor.getCurrentPosition() + rightMotor.getCurrentPosition()) / 2;

        if (distanceTraveled < convertInchesToEncoderTicks(goalDistanceInch) + encoderPositionReference) {
            loopPID(Tp);
        } else {
            nextMovement();
        }
    }

    private void moveBackward(double goalDistanceInch, double Tp) { //Movement val == 1
        int distanceTraveled = (leftMotor.getCurrentPosition() + rightMotor.getCurrentPosition()) / 2;

        if (distanceTraveled > convertInchesToEncoderTicks(goalDistanceInch) + encoderPositionReference) {
            loopPID(Tp);
        } else {
            nextMovement();
        }
    }

    private void rotateCW(double goalAngle, double Tp) { //Movement val == 3
        setPoint = goalAngle;
        if (yawAngle[0] < setPoint) {
            loopPID(Tp);
        } else {
            nextMovement();
        }
    }

    private void rotateCCW(double goalAngle, double Tp) { //Movement val == 4
        setPoint = goalAngle;
        if (yawAngle[0] > setPoint) {
            loopPID(Tp);
        } else {
            nextMovement();
        }
    }

    private double convertInchesToEncoderTicks(double inches) {
        return encoderTicksPerInch * inches;
    }

    private void nextMovement() {
        leftMotor.setPower(0);
        rightMotor.setPower(0);
        encoderPositionReference = (leftMotor.getCurrentPosition() + rightMotor.getCurrentPosition()) / 2;
        movementArrayStep++;
    }

    private void loopPID(double Tp) {
        double error = yawAngle[0] - setPoint;
        integral += error;
        double derivative = error - preError;
        double output = (Kp * error) + (Ki * integral) + (Kd * derivative);

        double leftOut = Range.clip(Tp + output, -0.5, 0.5);
        double rightOut = Range.clip(Tp - output, -0.5, 0.5);

        leftMotor.setPower(leftOut);
        rightMotor.setPower(rightOut);
        preError = error;
    }
}
