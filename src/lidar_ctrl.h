#ifndef LIDAR_CTRL_H
#define LIDAR_CTRL_H

#include <Arduino.h>
#include <HardwareSerial.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

// ============================================
// CONFIGURATION LIDAR
// ============================================
#define LIDAR_RX_PIN 5   // IO5 sur le Rover
#define LIDAR_TX_PIN -1  // Pas utilisé
#define LIDAR_BAUDRATE 230400

// ============================================
// STRUCTURES
// ============================================
struct LidarPoint {
  float angle;
  uint16_t distance;
  uint8_t quality;
  bool valid;
  unsigned long lastUpdate;
};

// Structure paquet LD06 (Standard)
struct __attribute__((packed)) LidarPacket {
  uint8_t header;         
  uint8_t ver_len;        
  uint16_t speed;         
  uint16_t start_angle;   
  uint8_t data[36];       
  uint16_t end_angle;     
  uint16_t timestamp;     
  uint8_t crc8;           
};

// ============================================
// VARIABLES GLOBALES LIDAR
// ============================================
// Utilisation de UART 2 pour éviter les conflits avec UART 1 (souvent lié à la Flash)
HardwareSerial lidarSerial(2); 
TaskHandle_t TaskLidarHandle;
SemaphoreHandle_t pointsMutex;

// Buffer circulaire naturel (0-359 degrés)
LidarPoint lidarPoints[360]; 

uint8_t serialBuffer[1024];
int bufferIndex = 0;
unsigned long totalPacketsProcessed = 0;

// Table CRC8
static const uint8_t CrcTable[256] = {
    0x00, 0x4d, 0x9a, 0xd7, 0x79, 0x34, 0xe3, 0xae, 0xf2, 0xbf, 0x68, 0x25, 0x8b, 0xc6, 0x11, 0x5c,
    0xa9, 0xe4, 0x33, 0x7e, 0xd0, 0x9d, 0x4a, 0x07, 0x5b, 0x16, 0xc1, 0x8c, 0x22, 0x6f, 0xb8, 0xf5,
    0x1f, 0x52, 0x85, 0xc8, 0x66, 0x2b, 0xfc, 0xb1, 0xed, 0xa0, 0x77, 0x3a, 0x94, 0xd9, 0x0e, 0x43,
    0xb6, 0xfb, 0x2c, 0x61, 0xcf, 0x82, 0x55, 0x18, 0x44, 0x09, 0xde, 0x93, 0x3d, 0x70, 0xa7, 0xea,
    0x3e, 0x73, 0xa4, 0xe9, 0x47, 0x0a, 0xdd, 0x90, 0xcc, 0x81, 0x56, 0x1b, 0xb5, 0xf8, 0x2f, 0x62,
    0x97, 0xda, 0x0d, 0x40, 0xee, 0xa3, 0x74, 0x39, 0x65, 0x28, 0xff, 0xb2, 0x1c, 0x51, 0x86, 0xcb,
    0x21, 0x6c, 0xbb, 0xf6, 0x58, 0x15, 0xc2, 0x8f, 0xd3, 0x9e, 0x49, 0x04, 0xaa, 0xe7, 0x30, 0x7d,
    0x88, 0xc5, 0x12, 0x5f, 0xf1, 0xbc, 0x6b, 0x26, 0x7a, 0x37, 0xe0, 0xad, 0x03, 0x4e, 0x99, 0xd4,
    0x7c, 0x31, 0xe6, 0xab, 0x05, 0x48, 0x9f, 0xd2, 0x8e, 0xc3, 0x14, 0x59, 0xf7, 0xba, 0x6d, 0x20,
    0xd5, 0x98, 0x4f, 0x02, 0xac, 0xe1, 0x36, 0x7b, 0x27, 0x6a, 0xbd, 0xf0, 0x5e, 0x13, 0xc4, 0x89,
    0x63, 0x2e, 0xf9, 0xb4, 0x1a, 0x57, 0x80, 0xcd, 0x91, 0xdc, 0x0b, 0x46, 0xe8, 0xa5, 0x72, 0x3f,
    0xca, 0x87, 0x50, 0x1d, 0xb3, 0xfe, 0x29, 0x64, 0x38, 0x75, 0xa2, 0xef, 0x41, 0x0c, 0xdb, 0x96,
    0x42, 0x0f, 0xd8, 0x95, 0x3b, 0x76, 0xa1, 0xec, 0xb0, 0xfd, 0x2a, 0x67, 0xc9, 0x84, 0x53, 0x1e,
    0xeb, 0xa6, 0x71, 0x3c, 0x92, 0xdf, 0x08, 0x45, 0x19, 0x54, 0x83, 0xce, 0x60, 0x2d, 0xfa, 0xb7,
    0x5d, 0x10, 0xc7, 0x8a, 0x24, 0x69, 0xbe, 0xf3, 0xaf, 0xe2, 0x35, 0x78, 0xd6, 0x9b, 0x4c, 0x01,
    0xf4, 0xb9, 0x6e, 0x23, 0x8d, 0xc0, 0x17, 0x5a, 0x06, 0x4b, 0x9c, 0xd1, 0x7f, 0x32, 0xe5, 0xa8
};

// ============================================
// LOGIQUE METIER LIDAR
// ============================================

uint8_t calculateCRC8(const uint8_t *data, uint8_t len) {
  uint8_t crc = 0;
  for (uint8_t i = 0; i < len; i++) {
    crc = CrcTable[crc ^ data[i]];
  }
  return crc;
}

void processLidarPacket(const uint8_t *data) {
  LidarPacket *pkt = (LidarPacket *)data;

  if (calculateCRC8(data, 46) != pkt->crc8) return; // Erreur CRC

  float startAngle = pkt->start_angle / 100.0f;
  float endAngle = pkt->end_angle / 100.0f;
  
  if (startAngle >= 360.0f) startAngle -= 360.0f;
  if (endAngle >= 360.0f) endAngle -= 360.0f;

  float angleStep;
  if (endAngle >= startAngle) {
    angleStep = (endAngle - startAngle) / 12.0f;
  } else {
    angleStep = (360.0f + endAngle - startAngle) / 12.0f;
  }

  if (xSemaphoreTake(pointsMutex, 5) == pdTRUE) {
    unsigned long now = millis();
    totalPacketsProcessed++;

    for (int i = 0; i < 12; i++) {
      uint16_t dist = (pkt->data[i * 3 + 1] << 8) | pkt->data[i * 3];
      
      if (dist > 0) {
        float angle = startAngle + (angleStep * i);
        if (angle >= 360.0f) angle -= 360.0f;
        
        int idx = (int)(angle + 0.5f) % 360;
        
        lidarPoints[idx].angle = angle;
        lidarPoints[idx].distance = dist;
        lidarPoints[idx].quality = pkt->data[i * 3 + 2];
        lidarPoints[idx].valid = true;
        lidarPoints[idx].lastUpdate = now;
      }
    }
    xSemaphoreGive(pointsMutex);
  }
}

void readLidarData() {
  // On lit jusqu'à ce que buffer soit vide ou qu'on ait un paquet
  while (lidarSerial.available()) {
    uint8_t byte = lidarSerial.read();
    
    // Si on attend le header 0x54
    if (bufferIndex == 0) {
      if (byte == 0x54) {
        serialBuffer[0] = 0x54;
        bufferIndex = 1;
      }
    } else {
      serialBuffer[bufferIndex++] = byte;
      
      // Si paquet complet (47 bytes : 1 header + 46 payload)
      if (bufferIndex == 47) {
        processLidarPacket(serialBuffer);
        bufferIndex = 0;
        return; // On sort pour traiter le suivant au prochain cycle si besoin
      }
    }
  }
}

// Tâche FreeRTOS dédiée à la lecture du Lidar
void lidarTask(void *parameter) {
  while (true) {
    // Lecture continue tant qu'il y a des données
    // On augmente le "chunk" de lecture pour éviter de yield trop souvent
    // si le buffer est plein
    unsigned long start = millis();
    while (lidarSerial.available() > 0 && (millis() - start < 20)) { 
        readLidarData();
    }
    vTaskDelay(1); // Pause minimale de 1 tick (1ms) pour laisser respirer l'OS
  }
}

void initLidar() {
  pointsMutex = xSemaphoreCreateMutex();
  
  // Utilisation de UART 2 (Serial2)
  // RX = LIDAR_RX_PIN (IO5), TX = -1
  lidarSerial.begin(LIDAR_BAUDRATE, SERIAL_8N1, LIDAR_RX_PIN, LIDAR_TX_PIN);
  
  // Augmentation du buffer RX Hard pour éviter les débordements à haute vitesse
  lidarSerial.setRxBufferSize(4096); 

  // Lancement de la tâche sur le Core 1 (Pour laisser le Core 0 au WiFi)
  // Core 1 gère Loop + Lidar, Core 0 gère WiFi/Système
  xTaskCreatePinnedToCore(lidarTask, "Lidar", 8192, NULL, 5, &TaskLidarHandle, 1);
}

#endif
