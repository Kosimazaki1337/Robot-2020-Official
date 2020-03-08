
/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.SPI;

/**
 * Add your docs here.
 */
public class PortMap {

    // DriveTrain
    public static final int kLMasterDrive = 0;
    public static final int kRMasterDrive = 1;
    public static final int kLSlaveMDrive = 2;
    public static final int kLSlaveBDrive = 3;
    public static final int kRSlaveMDrive = 0;
    public static final int kRSlaveBDrive = 1;

    public static final SPI.Port gyroPort = SPI.Port.kMXP;

    // Shooter
    public static int kLMasterShooter = 2;
    public static int kLSlaveShooter = 4;
    public static int kRMasterShooter = 3;
    public static int kRSlaveShooter = 6;

    // Intake
    public static int kIntakeSolenoidA = 0;
    public static int kIntakeSolenoidB = 1;
    public static int kIntakeMotor = 8;

    // Aiming
    public static int kAimMotor = 7;

    public static int kPotentiometer = 0; // Analog

    // Transporter
    public static int kTransportMotor = 5;

    public static int kPhotoSensor = 0; // DIO
    public static int kLimitSwitch = 1; // DIO
    public static int kUltrasonic = 1; // Analog
    private I2C.Port kColorSensor = I2C.Port.kOnboard;

    // LED
    public static int kLedPWM = 2;

    public I2C.Port getkColorSensor() {
        return kColorSensor;
    }
}

