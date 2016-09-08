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

    private int currentMovement = 0; //Variable to choose which methods to call in movementControl and readyForNewCommand. This is set to 0 so readyForNewCommand initially goes to the default case
    private double currentGoalValue; //Variable to store the parameters passed through move
    private double currentTpValue; //Variable to store the parameters passed through move

    //Constructor
    public navigationPID(HardwareMap currentIMUHWmap, String configuredIMUname, DcMotor leftMotor, DcMotor rightMotor) {
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

    public boolean readyForNewCommand() {
        boolean readyForNextValue; //Local variable to store the output of the switch statement
        switch (currentMovement) {
            case 1:
                readyForNextValue = moveForward(currentGoalValue, currentTpValue); //Set readyForNextValue to the output of moveForward with the same arguments from the last time move was called
                break;
            case 2:
                readyForNextValue = moveBackward(currentGoalValue, currentTpValue); //Set readyForNextValue to the output of moveBackward with the same arguments from the last time move was called
                break;
            case 3:
                readyForNextValue = rotateCW(currentGoalValue, currentTpValue); //Set readyForNextValue to the output of rotateCW with the same arguments from the last time move was called
                break;
            case 4:
                readyForNextValue = rotateCCW(currentGoalValue, currentTpValue); //Set readyForNextValue to the output of rotateCCW with the same arguments from the last time move was called
                break;
            case 5:
                readyForNextValue = false; //5 is the end navigation value, so set readyForNextValue to false
                break;
            default:
                readyForNextValue = true; //If there is any other value, set readyForNextValue to true
                break;
        }

        return readyForNextValue; //Send the output of the switch statement
    }

    public void move(int movementType, double Tp, double goalDistanceOrAngle) {
        //Store the current arguments so the user does not have to respecify these values until the next time move is called
        currentGoalValue = goalDistanceOrAngle;
        currentTpValue = Tp;
        currentMovement = movementType;

        movementControl(movementType, Tp, goalDistanceOrAngle); //Call movementControl
    }

    //**********Private methods**********

    private void movementControl(int movementType, double Tp, double goalValue) {
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
                //End of navigation
                break;
        }
    }

    private double loopPID(double Tp) {
        AdafruitGyro.getIMUGyroAngles(rollAngle, pitchAngle, yawAngle);

        double error = yawAngle[0] - setPoint;
        integral += error;
        double derivative = error - preError;
        double output = (Kp * error) + (Ki * integral) + (Kd * derivative);

        double leftOut = Range.clip(Tp - output, -0.5, 0.5);
        double rightOut = Range.clip(Tp + output, -0.5, 0.5);

        leftMotor.setPower(leftOut);
        rightMotor.setPower(rightOut);
        preError = error;

        return output;
    }

    private boolean moveForward(double goalDistanceInch, double Tp) { //Movement val == 1
        int distanceTraveled = (leftMotor.getCurrentPosition() + rightMotor.getCurrentPosition()) / 2;

        if (distanceTraveled > (encoderTicksPerInch * goalDistanceInch) + encoderPositionReference) {
            loopPID(Tp);
            return false;
        } else {
            encoderPositionReference = (leftMotor.getCurrentPosition() + rightMotor.getCurrentPosition()) / 2;
            return true;
        }
    }

    private boolean moveBackward(double goalDistanceInch, double Tp) { //Movement val == 1
        int distanceTraveled = (leftMotor.getCurrentPosition() + rightMotor.getCurrentPosition()) / 2;

        if (distanceTraveled < (encoderTicksPerInch * goalDistanceInch) + encoderPositionReference) {
            loopPID(Tp);
            return false;
        } else {
            encoderPositionReference = (leftMotor.getCurrentPosition() + rightMotor.getCurrentPosition()) / 2;
            return true;
        }
    }

    private boolean rotateCW(double goalAngle, double Tp) { //Movement val == 3
        setPoint = goalAngle;
        if (yawAngle[0] > setPoint) {
            loopPID(Tp);
            return false;
        } else {
            encoderPositionReference = (leftMotor.getCurrentPosition() + rightMotor.getCurrentPosition()) / 2;
            return true;
        }
    }

    private boolean rotateCCW(double goalAngle, double Tp) { //Movement val == 4
        setPoint = goalAngle;
        if (yawAngle[0] < setPoint) {
            loopPID(Tp);
            return false;
        } else {
            encoderPositionReference = (leftMotor.getCurrentPosition() + rightMotor.getCurrentPosition()) / 2;
            return true;
        }
    }
}