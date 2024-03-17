package org.firstinspires.ftc.teamcode.worldsAuto;

import com.acmerobotics.dashboard.config.Config;

/**
 * The following class here is PID constants that I (Abdullah) have tuned myself using FTC Dashboard
 */
@Config
public class PIDConstants {
    public static double Kp = 2;  // Proportional gain
    public static double Ki = 0.0;   // Integral gain ( KEEP THIS 0 )
    public static double Kd = 0.0;   // Derivative gain
}
