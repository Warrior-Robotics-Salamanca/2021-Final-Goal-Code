/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.GameOpModes.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.constants.DriveBaseConstants;
import org.firstinspires.ftc.teamcode.network.TCPDriveStationClient;
import org.firstinspires.ftc.teamcode.util.CartesianVectorD;
import org.firstinspires.ftc.teamcode.util.PolarVectorD;

@Disabled
@TeleOp(name="network client test", group="Tests")
public class Client_Test extends LinearOpMode {
    private TCPDriveStationClient driveStation;

    DcMotor motor;

    @Override
    public void runOpMode() {
        driveStation = new TCPDriveStationClient(telemetry, "192.168.43.173", 12345);

        motor = hardwareMap.get(DcMotor.class, "motor");


        waitForStart();

        while (opModeIsActive()) {
            double motPos = motor.getCurrentPosition();
            double motAngle = motPos * Math.PI / DriveBaseConstants.TICKS_PER_MOTOR_REVOLUTION;

            CartesianVectorD circlePlot = new CartesianVectorD();
            circlePlot.fromPolarVector(motAngle, 100);

            String response = driveStation.exchangeData("x:" + circlePlot.x + "y:" + circlePlot.y + "inData:" + motAngle);
            if(response.contains("x:") && response.contains("y:")) {
                int xIndex = response.indexOf("x:");
                int yIndex = response.indexOf("y:");
                CartesianVectorD mouse = new CartesianVectorD(
                        Double.parseDouble(response.substring(xIndex + 2, yIndex)),
                        Double.parseDouble(response.substring(yIndex + 2))
                );
                PolarVectorD polarMouse = new PolarVectorD();
                polarMouse.fromCartesianVector(mouse);
                motor.setTargetPosition((int) (polarMouse.theta * DriveBaseConstants.TICKS_PER_MOTOR_REVOLUTION / Math.PI));
                motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motor.setPower(.9);
            } else {
                motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                motor.setPower(0);
            }
            telemetry.addLine("Server: " + response);
            telemetry.update();
        }
        driveStation.shutdownClient();
    }
}
