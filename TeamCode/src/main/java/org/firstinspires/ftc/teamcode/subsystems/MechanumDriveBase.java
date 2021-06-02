package org.firstinspires.ftc.teamcode.subsystems;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;

public class MechanumDriveBase {
    public static Wheels joystickToWheels(double joyX, double joyY, double joyTurn) {
        MechanumDriveBase.Wheels w = new MechanumDriveBase.Wheels(joyY - joyTurn - joyX, joyY - joyTurn + joyX, joyY + joyTurn + joyX, joyY + joyTurn - joyX);
        return w;
    }

    public static class Wheels {
        public double frontLeft;
        public double backLeft;
        public double frontRight;
        public double backRight;

        public Wheels (double fl, double bl, double fr, double br) {
            frontLeft = fl;
            backLeft = bl;
            frontRight = fr;
            backRight = br;
        }

        public void scaleAndMaintainRatio() {
            double[] powArr = {frontLeft, backLeft, frontRight, backRight};

            // this algorithm will find the maximum motor value, check if it is greater than one,
            // then divide all the motor values by that amount
            double maxPow = 0;
            for(double d : powArr) {
                if(Math.abs(d) > maxPow)
                    maxPow = Math.abs(d);
            }
            if(maxPow > 1) {
                frontLeft  /= maxPow;
                backLeft   /= maxPow;
                frontRight /= maxPow;
                backRight  /= maxPow;
            }
        }
    }
}
