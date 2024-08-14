/*
 * xbox.cpp
 */

#include "xbox.h"
#include "debug.h"
#include "defines.h"
#include "globals.h"
#include "MPU6050.h"
#include <XboxSeriesXControllerESP32_asukiaaa.hpp>

// only bind to my xbox controller
// XboxSeriesXControllerESP32_asukiaaa::Core xboxController("9c:aa:1b:f2:66:3d");

// bind to any xbox controller
XboxSeriesXControllerESP32_asukiaaa::Core xboxController;

void Xbox_setup()
{
#ifdef DEBUG_XBOX_CONTROLLER
    DB_PRINTLN("Starting NimBLE Client");
#endif
    xboxController.begin();
}

void Xbox_loop()
{
    // if not connected, reconnect (this is _very_ expensive!), else respond to controller input
    xboxController.onLoop();
    if (xboxController.isConnected())
    {
        if (xboxController.isWaitingForFirstNotification())
        {
#ifdef DEBUG_XBOX_CONTROLLER
            DB_PRINTLN("waiting for first notification");
#endif
        }
        else
        {
#ifdef DEBUG_XBOX_CONTROLLER
            static int first = 1;
            if (first)
            {
                first = 0;
                DB_PRINTLN("Address: " + xboxController.buildDeviceAddressStr());
                DB_PRINT(xboxController.xboxNotif.toString());
            }
#endif
            // normalize the controller input to the range of 0 to 1 then scale
            float car_speed_forward = ((float)xboxController.xboxNotif.trigRT / XboxControllerNotificationParser::maxTrig);
            float car_speed_reverse = ((float)xboxController.xboxNotif.trigLT / XboxControllerNotificationParser::maxTrig);

            // subtract the requested reverse speed from the requested forward speed in case both triggers are requesting different values
            throttle = (car_speed_forward - car_speed_reverse) * max_throttle;

            // the steering is based off the left horizontal joystick
            // convert the range from 0 <-> maxJoy to -1.0 <-> 1.0
            steering = (float)(xboxController.xboxNotif.joyLHori - (XboxControllerNotificationParser::maxJoy / 2)) / (XboxControllerNotificationParser::maxJoy / 2);

            // We add some exponential on steering to smooth the center band
            if (steering > 0)
                steering = (steering * steering + 0.5 * steering) * max_steering;
            else
                steering = (-steering * steering + 0.5 * steering) * max_steering;

            // The 'Start' button will tell us to start the calibration process
            if (xboxController.xboxNotif.btnStart)
            {
#ifndef OLD_MPU6050
                DB_PRINTLN("PID tuning - each dot = 100 readings");
                mpu.CalibrateAccel(6);
                mpu.CalibrateGyro(6);
                DB_PRINTLN(" Calibration complete and stored in MPU6050");
#else
                MPU6050_calibrate();
#endif // OLD_MPU6050
            }

#ifdef XBOX_SERIAL_PLOTTER
            DB_PRINT("throttle:");
            DB_PRINT(throttle);
            DB_PRINT(", steering:");
            DB_PRINTLN(steering);
#endif // XBOX_SERIAL_PLOTTER
        }
    }
    else
    {
#ifdef DEBUG_XBOX_CONTROLLER
        DB_PRINTLN("not connected");
#endif
        // To prevent rebooting when a controller turns on but doesn't connect we could
        // start with an empty address and reboot if failed connections. Once connected,
        // save the address in preferences (with a WebUI to clear it) and use that without
        // the reboot logic on future boots.
        if (xboxController.getCountFailedConnection() > 3)
        {
            ESP.restart();
        }
    }
}