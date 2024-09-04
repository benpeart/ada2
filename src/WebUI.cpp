#include "defines.h"
#ifdef WEBUI
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWiFiManager.h>
#include <ESPAsyncWebServer.h>
#include <ElegantOTA.h>
#include <SPIFFS.h>
#include "debug.h"
#include "webui.h"
#include "globals.h"

char robotName[63] = "ada";
AsyncWebServer httpServer(80);
AsyncWebSocket wsServer("/ws");
DNSServer dns;

bool settingsChanged = false;

// Plot settings
struct
{
    boolean enable = 0; // Enable sending data
    uint8_t prescaler = 4;
} plot;

void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len)
{
    switch (type)
    {
    // client connected
    case WS_EVT_CONNECT:
    {
        DB_PRINTF("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());

        // send config data to webSocket client
#ifdef DRIVING_MODE
        SendDriveMode(num);
#endif  // DRIVING_MODE
        // !!FIX THIS!! This causes a crash
//        wsServer.printfAll("kp%.4f", Kp);
//        wsServer.printfAll("kd%.4f", Kd);
        break;
    }

    // client disconnected
    case WS_EVT_DISCONNECT:
    {
        DB_PRINTF("WebSocket client #%u disconnected\n", client->id());
        if (settingsChanged)
        {
            preferences.putFloat("Angle_kp", Kp);
            preferences.putFloat("Angle_ki", Kd);
            settingsChanged = false;
        }
        break;
    }

    // error was received from the other end
    case WS_EVT_ERROR:
    {
        DB_PRINTF("WebSocket[%s][%u] error(%u): %s\n", server->url(), client->id(), *((uint16_t *)arg), (char *)data);
        break;
    }

    // data packet
    case WS_EVT_DATA:
    {
        AwsFrameInfo *info = (AwsFrameInfo *)arg;

        if (info->final && info->index == 0 && info->len == len)
        {
            switch (info->opcode)
            {
            case WS_TEXT:
                char *payload = (char *)data;
                DB_PRINTF("WebSocket client #%u get Text: %s\n", client->id(), payload);

                switch (payload[0])
                {
#ifdef DRIVING_MODE
                case 'm':
                    switch (payload[1])
                    {
                    case 'p':
                        BalanceDriveController_SetMode(MODE_PARKED);
                        break;
                    case 's':
                        BalanceDriveController_SetMode(MODE_STANDING_UP);
                        break;
                    case 'k':
                        BalanceDriveController_SetMode(MODE_PARKING);
                        break;
                    case 'd':
                        BalanceDriveController_SetMode(MODE_DRIVE);
                        break;
                    case 'f':
                        BalanceDriveController_SetMode(MODE_FALLEN);
                        break;
                    case 'c':
                        BalanceDriveController_SetMode(MODE_CALIBRATION);
                        break;
                    }
                    break;
#endif // DRIVING_MODE
                case 's':
                    switch (payload[1])
                    {
                    case 't':
                        plot.enable = true;
                        break;
                    case 'p':
                        plot.enable = false;
                        break;
                    case 'r':
                        plot.prescaler = atoi(payload + 2);
                        break;
                    }
                    break;

                case 'k':
                    float val = atof(payload + 2);
                    switch (payload[1])
                    {
                    case 'p':
                        Kp = val;
                        DB_PRINTF("Set Kp to [%f]\n", val);
                        break;
                    case 'd':
                        Kd = val;
                        DB_PRINTF("Set Kd to [%f]\n", val);
                        break;
                    }
                    settingsChanged = true;
                }
                break;
            }
        }
    }
    }
}

bool WebUI_setup()
{
    // SPIFFS setup
    if (!SPIFFS.begin(false))
    {
        DB_PRINTLN("SPIFFS mount failed");
        return false;
    }
    else
    {
        DB_PRINTLN("SPIFFS mount success");
    }

    // Read robot name
    preferences.getBytes("robot_name", robotName, sizeof(robotName));
    WiFi.setHostname(robotName);

    // WiFiManager local intialization. Once its business is done, there is no need to keep it around
    AsyncWiFiManager wifiManager(&httpServer, &dns);
    // wifiManager.resetSettings();

    // fetches WiFi ssid and password from eeprom and tries to connect
    // if it does not connect it starts an access point with the specified name
    // and goes into a blocking loop awaiting configuration
    wifiManager.autoConnect(robotName);
    if (!(WiFi.waitForConnectResult() != WL_CONNECTED))
    {
        DB_PRINT("Connected to WiFi with IP address: ");
        DB_PRINTLN(WiFi.localIP());
    }
    else
    {
        DB_PRINTLN("Could not connect to known WiFi network");
    }

    // setup the Async Web Server

    // Serve static files from SPIFFS
    httpServer.serveStatic("/", SPIFFS, "/").setDefaultFile("index.html");

    // Add AsyncWebSocket handler
    wsServer.onEvent(onEvent);
    httpServer.addHandler(&wsServer);

    // Add over the air (OTA) updates
    ElegantOTA.begin(&httpServer);

    // start the server
    httpServer.begin();

    return true;
}

void WebUI_loop()
{
    static uint8_t k = 0;

    if (k == plot.prescaler)
    {
        k = 0;

        if (plot.enable)
        {
            float plotData[15] = {0};

            plotData[0] = (speed_M1 + speed_M2) / 2;
            plotData[1] = steering;
#if 0            
            plotData[2] = pidBalance.Input;
            plotData[3] = pidBalance.Output;
            plotData[4] = ypr[0] * 180 / M_PI;
            plotData[5] = ypr[2] * 180 / M_PI; // pitch and roll are reversed due to how the MPU6050
            plotData[6] = ypr[1] * 180 / M_PI; // is mounted on the breadboard
            plotData[10] = encoder_count_left_a;
            plotData[11] = encoder_count_right_a;
            plotData[12] = pwm_left;
            plotData[13] = pwm_right;
            plotData[14] = dt;
#endif // 0
            wsServer.binaryAll((uint8_t *)plotData, sizeof(plotData));
        }
    }
    k++;

    // only clean up clients every second
    static unsigned long lastTime = 0;
    unsigned long currentTime = millis();
    if ((currentTime - lastTime) < 1000)
        return;

    wsServer.cleanupClients();

    // update our time tracking
    lastTime = currentTime;
}

#endif // WEBUI
