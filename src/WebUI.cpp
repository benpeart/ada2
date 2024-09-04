#include "defines.h"
#ifdef WEBUI
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWiFiManager.h>
#include <ESPAsyncWebServer.h>
#ifdef ELEGANTOTA
#include <ElegantOTA.h>
#endif // ELEGANTOTA
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
#endif // DRIVING_MODE
        AsyncWebSocketMessageBuffer *buffer;

        buffer = wsServer.makeBuffer(16);
        if (buffer)
        {
            snprintf((char *)buffer->get(), buffer->length(), "kp%.4f", Kp);
            wsServer.textAll(buffer);
        }
        buffer = wsServer.makeBuffer(16);
        if (buffer)
        {
            snprintf((char *)buffer->get(), buffer->length(), "kd%.4f", Kd);
            wsServer.textAll(buffer);
        }
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
                payload[len] = 0;
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
#ifdef ELEGANTOTA
    ElegantOTA.begin(&httpServer);
#endif // ELEGANTOTA

    // start the server
    httpServer.begin();

    return true;
}

void WebUI_loop(int dt)
{
    static uint8_t k = 0;

    if (k == plot.prescaler)
    {
        k = 0;

        if (plot.enable)
        {
            AsyncWebSocketMessageBuffer *buffer;

            buffer = wsServer.makeBuffer(sizeof(float) * 16);
            if (buffer)
            {
                float *plotData = (float *)buffer->get();

                plotData[0] = throttle;
                plotData[1] = steering;
                plotData[2] = 0;              // pidBalance.Input;
                plotData[3] = 0;              // pidBalance.Output;
                plotData[4] = 0;              // ypr[0] * 180 / M_PI;
                plotData[5] = angle_adjusted; // ypr[2] * 180 / M_PI; // pitch and roll are reversed due to how the MPU6050
                plotData[6] = 0;              // ypr[1] * 180 / M_PI; // is mounted on the breadboard
                plotData[10] = steps1;
                plotData[11] = steps2;
                plotData[12] = speed_M1;
                plotData[13] = speed_M2;
                plotData[14] = dt;
                wsServer.binaryAll(buffer);
            }
        }
    }
    k++;

    // only clean up clients every second
    static unsigned long lastTime = 0;
    unsigned long currentTime = millis();
    if ((currentTime - lastTime) < 1000)
        return;
    wsServer.cleanupClients();
    lastTime = currentTime;
}

#endif // WEBUI
