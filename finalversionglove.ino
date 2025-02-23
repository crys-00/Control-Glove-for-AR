#include <crystalepqcompany-project-1_inferencing.h>
#include "Wire.h"
#include <MPU6050_light.h>
#include <Mouse.h>
#include <Keyboard.h>

#define SAMPLELENGTH 110
#define A_THRESHOLD 0.7

MPU6050 mpu(Wire);

double pitch, roll, ax, ay, az, gx, gy, gz;
long timer = 0;
int numSample = 1;
long long int ttt;
int frameSample = SAMPLELENGTH;

static float features[EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE] = {};

int sol = 0;

/**
 * @brief      Copy raw feature data in out_ptr
 *             Function called by inference library
 *
 * @param[in]  offset   The offset
 * @param[in]  length   The length
 * @param      out_ptr  The out pointer
 *
 * @return     0
 */

int raw_feature_get_data(size_t offset, size_t length, float *out_ptr) {
  memcpy(out_ptr, features + offset, length * sizeof(float));
  return 0;
}

void print_inference_result(ei_impulse_result_t result);

/**
 * @brief      Arduino setup function
 */
void setup() {
  Serial.begin(115200);
  Wire.begin();
  delay(100);
  byte status = mpu.begin();
  while (status != 0) {}
  pinMode(LED_BUILTIN, OUTPUT);
  while (!Serial);
}

/**
 * @brief      Arduino main function
 */

int mode = 0;
int prevmode = 0;
int vid=0;

void loop() {
  prevmode = mode;
  mode = determinemode();
  //mode = 3; //debugging keyboard
  mpu.update();
  roll = mpu.getAngleX();
  pitch = mpu.getAngleY();

  if (prevmode != mode) {  //mode has changed
    digitalWrite(LED_BUILTIN, HIGH);
    delay(500);
    digitalWrite(LED_BUILTIN, LOW);
  }

  if (mode == 1) {  //left click
    if ((prevmode == 3) or (prevmode == 0)) {
      startmouse();
    }
    if ((prevmode == 3)) {
      endkeyboard();
    }
    leftclick();
    delay(1000);
  } 
  else if (mode == 2) {  //right click
    if ((prevmode == 3) or (prevmode == 0)) {
      startmouse();
    }
    if ((prevmode == 3)) {
      endkeyboard();
    }
    rightclick();
    delay(1000);
  } 
  else if (mode == 3) {  //keyboard
    if (prevmode != 3 && prevmode != 0) {
      endmouse();
    }
    if (prevmode != 3) {
      startkeyboard();
      prevmode = 3;
    }
    loopkeyboard();
  } 
  else if (mode == 4) {  //scroll
    if ((prevmode == 3) or (prevmode == 0)) {
      startmouse();
    }
    if ((prevmode == 3)) {
      endkeyboard();
    }
    scroll();
  } 
  else if (mode == 5) {  //movecursor
    if ((prevmode == 3) or (prevmode == 0)) {
      startmouse();
    }
    if ((prevmode == 3)) {
      endkeyboard();
    }
    loopmouse();
  } 
  else if (mode == 0) {  // idle
    if ((prevmode == 3)) {
      endkeyboard();
    }
    if (prevmode != 3 && prevmode != 0) {
      endmouse();
    }
  }
  delay(10);
}

int determinemode() {
  int ap1 = A7;
  int ap2 = A8;
  int ap3 = A9;
  const int threshold[3] = { 480, 480, 650 };
  int fingerup[3] = { 0, 0, 0 };
  const int modes[6][3] = { { 0, 0, 0 }, { 0, 1, 1 }, { 1, 0, 1 }, { 1, 0, 0 }, { 0, 0, 1 }, { 1, 1, 0 } };
  fingerup[0] = analogRead(ap1) < threshold[0];  // 480- is 1, 480+ is 0
  fingerup[1] = analogRead(ap2) < threshold[1]; //480+ is 0, 480- is 1
  fingerup[2] = analogRead(ap3) > threshold[2]; //650+ is 1, 650- is 0
  Serial.print(fingerup[0]);
  Serial.print(fingerup[1]);
  Serial.println(fingerup[2]);
  for (int i = 0; i < 6; i++) {
    int k = 1;
    for (int j = 0; j < 3; j++) {
      if (modes[i][j] != fingerup[j]) {
        k = 0;
        break;
      }
    }
    if (k) return i;
  }
  return 0;
}

void startmouse() {
  mpu.calcOffsets(true, true);
  Mouse.begin();
}

void loopmouse() {
  double xdir = 0;
  double ydir = 0;
  if (abs(roll) >= 10) {  //threshold
    xdir = roll / 10;
  }
  if (abs(pitch) >= 10) {
    ydir = pitch / 10;
  }
  Mouse.move(xdir, ydir, 0);
}


void endmouse() {
  Mouse.end();
}

void startkeyboard() {
  mpu.calcOffsets(true, true);
  Keyboard.begin();
}

void leftclick() {
  Mouse.click(MOUSE_LEFT);
}

void rightclick() {
  Mouse.click(MOUSE_RIGHT);
}

void scroll() {
  double wheel = 0;
  if (abs(pitch) >= 10) {
    wheel = pitch / 10;
  }
  Mouse.move(0, 0, wheel);
}

void loopkeyboard() {
  mpu.update();
  ax = mpu.getAccX();
  ay = mpu.getAccY();
  az = mpu.getAccZ() - 0.98;
  gx = mpu.getGyroX();
  gy = mpu.getGyroY();
  gz = mpu.getGyroZ();
  roll = mpu.getAccAngleX();
  pitch = mpu.getAccAngleY();
  if (abs(ax) + abs(ay) + abs(az) > A_THRESHOLD) {
    fillfeatures();
  }
  //delay(1000);
}

void endkeyboard() {
  Keyboard.end();
}

void fillfeatures() {
  int feature_ix = 0;
  for (int i = 0; i < SAMPLELENGTH; i++) {
    mpu.update();
    ax = mpu.getAccX();
    ay = mpu.getAccY();
    az = mpu.getAccZ() - 0.98;
    gx = mpu.getGyroX();
    gy = mpu.getGyroY();
    gz = mpu.getGyroZ();
    features[feature_ix++] = ax;
    features[feature_ix++] = ay;
    features[feature_ix++] = az;
    features[feature_ix++] = gx;
    features[feature_ix++] = gy;
    features[feature_ix++] = gz;
    delay(20);
  }
  if (feature_ix == EI_CLASSIFIER_DSP_INPUT_FRAME_SIZE && mode == 3) {
    classify();
    mpu.calcOffsets(true, true);
  } else {
    return;
  }
}

void classify() {
  ei_impulse_result_t result = { 0 };
  signal_t features_signal;
  features_signal.total_length = sizeof(features) / sizeof(features[0]);
  features_signal.get_data = &raw_feature_get_data;
  EI_IMPULSE_ERROR res = run_classifier(&features_signal, &result, false /* debug */);
  if (res != EI_IMPULSE_OK) {
    return;
  }
  type_result(result);

}

void debugclassify() {
  ei_printf("\nClassifying...\n");
  ei_impulse_result_t result = { 0 };
  // the features are stored into flash, and we don't want to load everything into RAM
  signal_t features_signal;
  features_signal.total_length = sizeof(features) / sizeof(features[0]);
  features_signal.get_data = &raw_feature_get_data;
  // invoke the impulse
  EI_IMPULSE_ERROR res = run_classifier(&features_signal, &result, false /* debug */);
  if (res != EI_IMPULSE_OK) {
    ei_printf("ERR: Failed to run classifier (%d)\n", res);
    return;
  }
  ei_printf("run_classifier returned: %d\r\n", res);
  print_inference_result(result);
}

void print_inference_result(ei_impulse_result_t result) {
  sol = 0;
  for (uint16_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
    ei_printf(ei_classifier_inferencing_categories[i]);
    ei_printf("%.2f \n", result.classification[i].value);
    if (result.classification[i].value > result.classification[sol].value) {
      sol = i;
    }
  }
  if (result.classification[sol].value > 0.5) {  //confidence threshold = 0.5
    ei_printf(ei_classifier_inferencing_categories[sol]);
  }
  else {
    ei_printf(" no sol found");
  }
}

void type_result(ei_impulse_result_t result) {
  sol = 0;
  for (uint16_t i = 0; i < EI_CLASSIFIER_LABEL_COUNT; i++) {
    if (result.classification[i].value > result.classification[sol].value) {
      sol = i;
    }
  }
  if (result.classification[sol].value > 0.5) {  //confidence threshold = 0.5
    if (ei_classifier_inferencing_categories[sol]=="bs"){
      Keyboard.print(" ");
    }
    else if (ei_classifier_inferencing_categories[sol]=="del"){
      Keyboard.println(" ");
    }
    else{
      Keyboard.print(ei_classifier_inferencing_categories[sol]);
    }
  } 
  else {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
  }
}
