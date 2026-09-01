#define CUSTOM_SETTINGS
#define INCLUDE_GAMEPAD_MODULE
#include <DabbleESP32.h>
#include <Wire.h>
#include <penguinUtils.h>

dfPlayer dfplayer(16,17);
touchSensor touchsensor(T3, 40, 3);

void TaskComunication(void *pvParameters);
void TaskSensor(void *pvParameters);

void setup() {
  Serial.begin(38400);
  Dabble.begin("Robot");

  // Volume
  dfplayer.setVolume(30);
  delay(3000);

  xTaskCreate(TaskComunication, "comunication", 4096, NULL, 1, NULL);
  xTaskCreate(TaskSensor, "sensor", 4096, NULL, 1, NULL);

}

void loop(){
}

void TaskComunication(void *pvParameters) {
  for (;;) {
    Dabble.processInput();
    if (GamePad.isSquarePressed()) {
      dfplayer.playTrack(1);
    }
    vTaskDelay(pdMS_TO_TICKS(20));   
  }
}

void TaskSensor(void *pvParameters) {
  bool petted;
  for (;;) {
    petted = touchsensor.affection();
    if (petted) dfplayer.playTrack(1);
    vTaskDelay(pdMS_TO_TICKS(200));   
  }
}