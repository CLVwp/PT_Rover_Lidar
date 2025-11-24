#include "web_page.h"
#include <WebSocketsServer.h>
#include "lidar_ctrl.h" // Pour accéder aux points Lidar

// WebServer sur port 80
WebServer server(80);

// WebSocket sur port 81
WebSocketsServer webSocket = WebSocketsServer(81);

void handleRoot(){
  server.send(200, "text/html", index_html); //Send web page
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
      
      // On simule l'état d'urgence pour l'instant (à connecter à la logique Rover si besoin)
      bool emergencyStopActive = false;
      doc["emergency"] = emergencyStopActive; 
      
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
  server.on("/", handleRoot);

  server.on("/js", [](){
    String jsonCmdWebString = server.arg(0);
    deserializeJson(jsonCmdReceive, jsonCmdWebString);
    jsonCmdReceiveHandler();
    serializeJson(jsonInfoHttp, jsonFeedbackWeb);
    server.send(200, "text/plane", jsonFeedbackWeb);
    jsonFeedbackWeb = "";
    jsonInfoHttp.clear();
    jsonCmdReceive.clear();
  });

  // Start server
  server.begin();
  Serial.println("HTTP Server Starts.");

  // Start WebSocket
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
