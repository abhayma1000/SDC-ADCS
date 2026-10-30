// Dear chatGPT thank you for this gracious code that I could not have been bothered with writing


#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <math.h>

// ============================================================
// CONFIGURATION
// ============================================================

const float SAMPLE_RATE_HZ = 100.0f;
const unsigned long RUN_DURATION_SECONDS = 120;
const unsigned long REPORT_INTERVAL_MS = 1000;
const unsigned long SETTLING_TIME_MS = 5000;

// ============================================================
// SENSOR
// ============================================================

Adafruit_BNO055 bno = Adafruit_BNO055(55);

// ============================================================
// RUNNING STATISTICS (Welford's algorithm)
// ============================================================

struct RunningStats {
  unsigned long n = 0;
  double mean = 0.0;
  double M2 = 0.0;

  void add(double x) {
    n++;
    double delta = x - mean;
    mean += delta / n;
    double delta2 = x - mean;
    M2 += delta * delta2;
  }

  double variance() const {
    return (n > 1) ? M2 / (n - 1) : 0.0;
  }

  double stddev() const {
    return sqrt(variance());
  }
};

// Gyroscope: rad/s
RunningStats gxStats, gyStats, gzStats;

// Magnetometer: microtesla (uT)
RunningStats mxStats, myStats, mzStats;

// ============================================================
// TIMING
// ============================================================

unsigned long startTimeMs = 0;
unsigned long lastReportMs = 0;
unsigned long nextSampleUs = 0;

const unsigned long SAMPLE_PERIOD_US =
    (unsigned long)(1000000.0f / SAMPLE_RATE_HZ);

bool finished = false;

// ============================================================
// FINAL RESULTS
// ============================================================

void printStatistics() {
  Serial.println();
  Serial.println("========== FINAL RESULTS ==========");
  Serial.print("Samples: ");
  Serial.println(gxStats.n);

  Serial.println();
  Serial.println("Gyroscope units: rad/s");
  Serial.println("Sensor,Axis,Mean,StdDev,Units");

  Serial.print("Gyro,X,");
  Serial.print(gxStats.mean, 8);
  Serial.print(",");
  Serial.print(gxStats.stddev(), 8);
  Serial.println(",rad/s");

  Serial.print("Gyro,Y,");
  Serial.print(gyStats.mean, 8);
  Serial.print(",");
  Serial.print(gyStats.stddev(), 8);
  Serial.println(",rad/s");

  Serial.print("Gyro,Z,");
  Serial.print(gzStats.mean, 8);
  Serial.print(",");
  Serial.print(gzStats.stddev(), 8);
  Serial.println(",rad/s");

  Serial.println();
  Serial.println("Magnetometer units: uT (microtesla)");

  Serial.print("Mag,X,");
  Serial.print(mxStats.mean, 6);
  Serial.print(",");
  Serial.print(mxStats.stddev(), 6);
  Serial.println(",uT");

  Serial.print("Mag,Y,");
  Serial.print(myStats.mean, 6);
  Serial.print(",");
  Serial.print(myStats.stddev(), 6);
  Serial.println(",uT");

  Serial.print("Mag,Z,");
  Serial.print(mzStats.mean, 6);
  Serial.print(",");
  Serial.print(mzStats.stddev(), 6);
  Serial.println(",uT");

  Serial.println("===================================");
}

// ============================================================
// SETUP
// ============================================================

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  Serial.println("BNO055 Gyroscope + Magnetometer Measurement");

  if (!bno.begin()) {
    Serial.println("ERROR: BNO055 not detected.");
    while (1) {}
  }

  delay(1000);
  bno.setExtCrystalUse(true);

  Serial.println("Keep the sensor completely stationary.");
  Serial.println("Allowing the sensor to settle...");
  delay(SETTLING_TIME_MS);

  startTimeMs = millis();
  lastReportMs = startTimeMs;
  nextSampleUs = micros();

  Serial.println("Starting measurement.");
  Serial.println(
      "elapsed_s,"
      "gyro_x_rad_s,gyro_y_rad_s,gyro_z_rad_s,"
      "mag_x_uT,mag_y_uT,mag_z_uT,samples"
  );
}

// ============================================================
// MAIN LOOP
// ============================================================

void loop() {
  if (finished) {
    return;
  }

  unsigned long nowUs = micros();

  if ((long)(nowUs - nextSampleUs) < 0) {
    return;
  }

  nextSampleUs += SAMPLE_PERIOD_US;

  // If behind schedule, restart timing.
  if ((long)(micros() - nextSampleUs) >= 0) {
    nextSampleUs = micros() + SAMPLE_PERIOD_US;
  }

  // Read gyroscope: angular velocity in rad/s.
  sensors_event_t gyroEvent;
  bno.getEvent(&gyroEvent, Adafruit_BNO055::VECTOR_GYROSCOPE);

  gxStats.add(gyroEvent.gyro.x);
  gyStats.add(gyroEvent.gyro.y);
  gzStats.add(gyroEvent.gyro.z);

  // Read magnetometer: magnetic field in microtesla.
  sensors_event_t magEvent;
  bno.getEvent(&magEvent, Adafruit_BNO055::VECTOR_MAGNETOMETER);

  mxStats.add(magEvent.magnetic.x);
  myStats.add(magEvent.magnetic.y);
  mzStats.add(magEvent.magnetic.z);

  unsigned long nowMs = millis();
  unsigned long elapsedMs = nowMs - startTimeMs;

  // Report running means every second.
  if (nowMs - lastReportMs >= REPORT_INTERVAL_MS) {
    lastReportMs = nowMs;

    Serial.print(elapsedMs / 1000.0f, 2);
    Serial.print(",");

    Serial.print(gxStats.mean, 8);
    Serial.print(",");
    Serial.print(gyStats.mean, 8);
    Serial.print(",");
    Serial.print(gzStats.mean, 8);
    Serial.print(",");

    Serial.print(mxStats.mean, 6);
    Serial.print(",");
    Serial.print(myStats.mean, 6);
    Serial.print(",");
    Serial.print(mzStats.mean, 6);
    Serial.print(",");

    Serial.println(gxStats.n);
  }

  if (elapsedMs >= RUN_DURATION_SECONDS * 1000UL) {
    finished = true;
    printStatistics();
  }
}
