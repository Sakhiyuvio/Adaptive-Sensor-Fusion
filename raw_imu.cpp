#include <Adafruit_LSM6DS.h>
#include <Adafruit_LSM6DSL.h>

#include <SPI.h>

// ESP32-S3 SPI pin config - IMU
#define IMU_SPI_SCLK 8
#define IMU_SPI_MISO 9
#define IMU_SPI_MOSI 7
#define IMU_SPI_CS 6
//#define IMU_INT_1 18
//#define IMU_INT_2 19

// serial comm data rate
#define comm_data_rate 115200
#define comm_init_delay 1000

// constants
#define g 9.8
#define INT_MAX 32767

SPIClass SPI2;
Adafruit_LSM6DSL imu_dsl;

// global variables for imu
float acc_x, acc_y, acc_z;
float noisy_acc_x, noisy_acc_y, noisy_acc_z;
float gyr_x, gyr_y;
float noisy_gyr_x, noisy_gyr_y; 
float acc_roll_angle, acc_pitch_angle;
float noisy_acc_roll_angle, noisy_acc_pitch_angle;
float gyr_roll_angle, gyr_pitch_angle;  
float noisy_gyr_roll_angle, noisy_gyr_pitch_angle; 
float curr_pitch_angle;
float noisy_curr_pitch_angle;
float curr_roll_angle;
float noisy_curr_roll_angle; 
float trust_factor = 0.95;
float trust_factor_complement = 1 - trust_factor;

// time-keeping
float delta_t;
unsigned long prev_time;
unsigned long print_prev_time;

float EPSILON = 1e-6;

void setup()
{
    Serial.begin(comm_data_rate);
    while (!Serial) {
      delay(10);
    }

    Serial.println("Testing IMU!");
    // Configure pins
    pinMode(IMU_SPI_CS, OUTPUT);
    digitalWrite(IMU_SPI_CS, HIGH);  // Initially HIGH
    SPI2.begin(IMU_SPI_SCLK, IMU_SPI_MISO, IMU_SPI_MOSI, IMU_SPI_CS);
   
    delay(10);
    // Try to read WHO_AM_I register directly
    Serial.println("Testing SPI communication...");
    uint8_t whoami = readRegister(0x0F);  // WHO_AM_I register
    Serial.printf("WHO_AM_I register value: 0x%02X (Should be 0x6A)\n", whoami);

    Serial.println("Attempting to initialize LSM6DSL...");
    if(!imu_dsl.begin_SPI(IMU_SPI_CS, &SPI2)){
      if(!imu_dsl.begin_SPI(IMU_SPI_CS, IMU_SPI_SCLK, IMU_SPI_MISO, IMU_SPI_MOSI)){
         Serial.println("Failed to find LSM6DSL chip");
       }  
    }
    else {
      Serial.println("IMU initialized successfully");
    }
 
//    // init IMU communication
//    pinMode(IMU_INT_1, INPUT_PULLDOWN);
//    pinMode(IMU_INT_2, INPUT_PULLDOWN);
//  
    acc_x = 0.0;
    acc_y = 0.0;
    acc_z = 0.0;
    noisy_acc_x = 0.0;
    noisy_acc_y = 0.0;
    noisy_acc_z = 0.0; 
    gyr_x = 0.0;
    gyr_y = 0.0;
    noisy_gyr_x = 0.0;
    noisy_gyr_y = 0.0; 
    curr_pitch_angle = 0.0;
    noisy_curr_pitch_angle = 0.0;
    curr_roll_angle = 0.0;
    noisy_curr_roll_angle = 0.0; 
    acc_roll_angle = 0.0;
    noisy_acc_roll_angle = 0.0;
    acc_pitch_angle = 0.0;
    noisy_acc_pitch_angle = 0.0;
    gyr_roll_angle = 0.0;
    noisy_gyr_roll_angle = 0.0; 
    gyr_pitch_angle = 0.0;
    noisy_gyr_pitch_angle = 0.0; 
    delta_t = 0.0;
    prev_time = millis();
    print_prev_time = millis();

}

void loop()
{
    // set up ranging loop for the IMU here
    sensors_event_t accel;
    sensors_event_t gyro;
    sensors_event_t temp;
    imu_dsl.getEvent(&accel, &gyro, &temp);

    acc_x = accel.acceleration.x;
    acc_y = accel.acceleration.y;
    acc_z = accel.acceleration.z;
    gyr_x = gyro.gyro.x;
    gyr_y = gyro.gyro.y;

    // get noisy sensor data for adaptive processing stage
    float mean = 0.0;
    float stddev = 0.1;
    noisy_acc_x = acc_x + gaussianNoise(mean, stddev);
    noisy_acc_y = acc_y + gaussianNoise(mean, stddev);
    noisy_acc_z = acc_z + gaussianNoise(mean, stddev); 
    noisy_gyr_x = gyr_x + gaussianNoise(mean, stddev);
    noisy_gyr_y = gyr_y + gaussianNoise(mean, stddev); 

    // update time variables
    delta_t = (millis() - prev_time) / 1000.0;
    prev_time = millis();

    // process the roll and pitch angle for configurating tilting through IMU sensor
    acc_roll_angle = atan2(acc_y/g, acc_z/g) * 180/PI;
    noisy_acc_roll_angle = atan2(noisy_acc_y/g, noisy_acc_z/g + EPSILON) * 180/PI; 
    acc_pitch_angle = atan2(acc_x/g, acc_z/g) * 180/PI;
    noisy_acc_pitch_angle = atan2(noisy_acc_x/g, noisy_acc_z/g + EPSILON) * 180 / PI; 
    gyr_roll_angle = gyr_x * delta_t;
    noisy_gyr_roll_angle = noisy_gyr_x * delta_t; 
    gyr_pitch_angle = gyr_y * delta_t;
    noisy_gyr_pitch_angle = noisy_gyr_y * delta_t; 

    // complementary filter imp
    curr_roll_angle = (curr_roll_angle + gyr_roll_angle) * trust_factor + acc_roll_angle*trust_factor_complement;
    curr_pitch_angle = (curr_pitch_angle - gyr_pitch_angle) * trust_factor + acc_pitch_angle*trust_factor_complement;
    noisy_curr_roll_angle = (noisy_curr_roll_angle + noisy_gyr_roll_angle) * trust_factor + noisy_acc_roll_angle*trust_factor_complement;
    noisy_curr_pitch_angle = (noisy_curr_pitch_angle - noisy_gyr_pitch_angle) * trust_factor + noisy_acc_pitch_angle*trust_factor_complement;

    if (millis() - print_prev_time > 1000) {
        Serial.print(curr_pitch_angle);
        Serial.print(", ");
        Serial.println(curr_roll_angle);
        Serial.print(", ");
        Serial.print(noisy_curr_pitch_angle);
        Serial.print(", ");
        Serial.println(noisy_curr_roll_angle);
        print_prev_time = millis();
    }

    // done! handle adaptive processing in host device for better computing power 
}

uint8_t readRegister(uint8_t reg) {
    digitalWrite(IMU_SPI_CS, LOW);
    delayMicroseconds(10);
    SPI2.transfer(reg | 0x80);  // Set read bit
    uint8_t value = SPI2.transfer(0x00);
    digitalWrite(IMU_SPI_CS, HIGH);
    return value;
}

// noise gen
float gaussianNoise(float mean, float stddev) {
    float gaussian_noise; 
    float FLOAT_MAX = 32767.0;
    float uni_distb_1 = random(0, INT_MAX) / FLOAT_MAX;
    float uni_distb_2 = random(0, INT_MAX) / FLOAT_MAX;

    if (uni_distb_1 < 0) {
      uni_distb_1 = EPSILON; 
    }
    if (uni_distb_2 < 0) {
      uni_distb_2 = EPSILON; 
    }

    // Box-Muller transform for independent standard normal distribution generation
    float box_mull_coeff = sqrt(-2.0 * log(uni_distb_1)) * cos(2.0 * PI * uni_distb_2);
    gaussian_noise = mean + stddev*box_mull_coeff;
    return gaussian_noise;
}