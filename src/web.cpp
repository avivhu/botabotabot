#include "web.hpp"
#include <Arduino.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <Streaming.h>
#include <SPIFFS.h>

const char index_html[] PROGMEM = R"rawliteral(
    This is a HTML placeholder.
)rawliteral";

static MessageCallbackType static_onMessageCb;

// See tutorial: https://randomnerdtutorials.com/esp32-websocket-server-arduino/
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

inline void handleWebSocketMessage(void *arg, uint8_t *data, size_t len)
{
    AwsFrameInfo *info = (AwsFrameInfo *)arg;
    if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT)
    {
        if (static_onMessageCb)
        {
            auto res = static_onMessageCb(data, len);
            ws.textAll(res.c_str());
        }
    }
}

inline void webSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type,
                           void *arg, uint8_t *data, size_t len)
{
    Serial << "webSocketEvent "
           << "Type " << (int)type << endl;
    switch (type)
    {
    case WS_EVT_CONNECT:
        Serial.printf("WebSocket client #%u connected from %s\n", client->id(), client->remoteIP().toString().c_str());
        break;
    case WS_EVT_DISCONNECT:
        Serial.printf("WebSocket client #%u disconnected\n", client->id());
        break;
    case WS_EVT_DATA:
        handleWebSocketMessage(arg, data, len);
        break;
    case WS_EVT_PONG:
    case WS_EVT_ERROR:
        break;
    }
}

void StartWebServer(MessageCallbackType onMessageCb)
{
    static_onMessageCb = onMessageCb;

    // Init Websockets
    {
        ws.onEvent(webSocketEvent);
        server.addHandler(&ws);
    }

    // Start web server (not in use)
    {
        // Route for root / web page
        server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
                  { request->send_P(200, "text/html", index_html); });

        server.serveStatic("/data", SPIFFS, "/data");
    }
    // Start server
    server.begin();
}
