#define USE_DABBLE
#include <penguinUtils.h>

dfPlayer dfplayer(16,17);
touchSensor touchsensor(T3, 40, 3);
comDabble com("Robot");

void TaskComunication(void *pvParameters);
void TaskSensor(void *pvParameters);

void setup() {
  Serial.begin(38400);
  com.begin();

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
    com.update();
    GamepadState pad = com.readGamepad();
    if (pad.square) {
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