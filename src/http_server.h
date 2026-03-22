#include <WebServer.h>
#include <WebSocketsServer.h>
#include "lidar_ctrl.h"   // points Lidar. IMU (icm_*, ax_ms2, gx...) déjà dispo via ugv_config.h inclus dans le .ino
extern double icm_yaw;   // cap IMU (degrés) pour estimateur / carte côté client
extern double icm_roll, icm_pitch;
extern double ax_ms2, ay_ms2, az_ms2;
extern double gx, gy, gz;

WebServer httpServer(80);
// WebSocket sur port 81
WebSocketsServer webSocket = WebSocketsServer(81);

String buildHttpJsonResponse() {
  if (jsonInfoHttp.isNull() || jsonInfoHttp.size() == 0) {
    StaticJsonDocument<96> okDoc;
    okDoc["ok"] = true;
    String okOut;
    serializeJson(okDoc, okOut);
    return okOut;
  }
  String output;
  serializeJson(jsonInfoHttp, output);
  return output;
}

void handleJsRequest() {
  if (!httpServer.hasArg("json")) {
    httpServer.send(400, "application/json", "{\"ok\":false,\"error\":\"missing json\"}");
    return;
  }

  const String rawJson = httpServer.arg("json");
  jsonCmdReceive.clear();
  jsonInfoHttp.clear();

  DeserializationError err = deserializeJson(jsonCmdReceive, rawJson);
  if (err != DeserializationError::Ok) {
    String errorOut = String("{\"ok\":false,\"error\":\"") + err.c_str() + "\"}";
    httpServer.send(400, "application/json", errorOut);
    jsonCmdReceive.clear();
    return;
  }

  const int cmdType = jsonCmdReceive["T"] | 0;

  switch (cmdType) {
    case CMD_BASE_FEEDBACK:
      inaDataUpdate();
      baseInfoFeedback();
      break;
    case CMD_WIFI_INFO:
      wifiStatusFeedback();
      break;
    case CMD_GET_IMU_DATA:
      getIMUData();
      break;
    case CMD_LIDAR_STATUS:
      getLidarStatus();
      break;
    default:
      jsonCmdReceiveHandler();
      break;
  }

  httpServer.send(200, "application/json", buildHttpJsonResponse());
  jsonCmdReceive.clear();
  jsonInfoHttp.clear();
}

void onWebSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length) {
  switch (type) {
    case WStype_CONNECTED:
      Serial.printf("WS client %u connected\n", num);
      break;
    case WStype_DISCONNECTED:
      Serial.printf("WS client %u disconnected\n", num);
      break;
    case WStype_TEXT: {
      // Commandes reçues en JSON via WebSocket (remplace /js HTTP)
      DeserializationError err = deserializeJson(jsonCmdReceive, payload, length);
      if (err == DeserializationError::Ok) {
        jsonCmdReceiveHandler();
      } else {
        Serial.printf("WS JSON parse error: %s\n", err.c_str());
      }
      jsonCmdReceive.clear();
      break;
    }
    default:
      break;
  }
}

// Tâche pour le Websocket (diffusion radar)
void websocketTask(void *parameter) {
  static unsigned long lastSend = 0;
  static DynamicJsonDocument doc(20000); 

  while (true) {
    webSocket.loop();
    
    unsigned long now = millis();
    
    // Envoi Client (20 FPS pour fluidité)
    if (now - lastSend > 50 && webSocket.connectedClients() > 0) {
      doc.clear(); 
      JsonArray pts = doc.createNestedArray("points");
      
      // On envoie l'état réel d'urgence + yaw pour estimateur/carte côté client (sans passer par VM)
      doc["emergency"] = emergencyStopActive;
      doc["r"] = (float)icm_roll;
      doc["p"] = (float)icm_pitch;
      doc["yaw"] = (float)icm_yaw;
      doc["y"] = (float)icm_yaw;  // alias pour compat bridge ROS
      doc["ax_ms2"] = (float)ax_ms2;
      doc["ay_ms2"] = (float)ay_ms2;
      doc["az_ms2"] = (float)az_ms2;
      doc["gx"] = (float)gx;
      doc["gy"] = (float)gy;
      doc["gz"] = (float)gz;

      if (xSemaphoreTake(pointsMutex, 50) == pdTRUE) {
        for (int i = 0; i < 360; i++) {
          if (lidarPoints[i].valid) {
            // Format ultra-compact: [Angle, Dist, Angle, Dist...]
            pts.add((int)lidarPoints[i].angle);
            pts.add(lidarPoints[i].distance);
          }
        }
        xSemaphoreGive(pointsMutex);
      }

      String output;
      serializeJson(doc, output);
      webSocket.broadcastTXT(output);
      lastSend = now;
    }
    vTaskDelay(5); 
  }
}

void webCtrlServer(){
  httpServer.on("/", HTTP_GET, []() {
    httpServer.send(200, "text/plain", "PT Rover HTTP server ready");
  });
  httpServer.on("/js", HTTP_GET, handleJsRequest);
  httpServer.begin();
  Serial.println("HTTP Server Starts.");

  // Start WebSocket
  webSocket.onEvent(onWebSocketEvent);
  webSocket.begin();
  Serial.println("WebSocket Server Starts.");
  
  // Lancer la tâche WebSocket sur le Core 1 (même que Loop, mais en tâche de fond)
  // ou Core 0 si on veut équilibrer. Ici Core 1 est déjà chargé, mettons sur Core 0 avec le Lidar
  // ou gardons Core 1 pour ne pas bloquer le Lidar qui est prioritaire sur Core 0.
  // Mettons-le sur Core 0 avec priorité basse (1) pour ne pas gêner Lidar (3).
  xTaskCreatePinnedToCore(websocketTask, "WS_Task", 10000, NULL, 1, NULL, 0);
}

void initHttpWebServer(){
  webCtrlServer();
}

void httpServerLoop() {
  httpServer.handleClient();
}
