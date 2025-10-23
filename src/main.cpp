#include <Arduino.h>
#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <avr/interrupt.h>
#include <math.h>

#define F_CPU 16000000UL // CPU clock speed: 16 MHz
#define IMU_ADDR 0x68
#define L_LED PB5  // LED L
#define LED_PIN PB3  // LED on PB3 (D3, OC2A)
#define SERVO_PWM_PIN PB1  // Pin for PWM output to servo

#define ServoRange 84 // yaw degree range limit 
#define LEDRange 79
#define BRIGHT1 255 // LED off
#define BRIGHT2 0 // LED on

// Gyroscope full-scale range options
#define GYRO_RANGE_250  0  // ±250°/s
#define GYRO_RANGE_500  1  // ±500°/s
#define GYRO_RANGE_1000 2  // ±1000°/s
#define GYRO_RANGE_2000 3  // ±2000°/s

#define ACCEL_RANGE_2G  0x00
#define ACCEL_RANGE_4G  0x08
#define ACCEL_RANGE_8G  0x10
#define ACCEL_RANGE_16G 0x18

// Current gyroscope range setting
float gyro_scale = 131.0f;
float acc_scale = 16384.0f;

int16_t AccX, AccY, AccZ, GyrX, GyrY, GyrZ; // raw
float accX, accY, accZ, gyrX, gyrY, gyrZ; // converted to g or deg
float yaw = 0.0f;
float pitch = 0.0f;
float roll = 0.0f;
float velocityX = 0.0f;   // in m/s
float distanceX = 0.0f;
float offset_ax, offset_ay, offset_az, offset_gx, offset_gy, offset_gz;

// ----- Serial Functions ----- //

// Serial begin (9600)
void uart_init() {
	uint16_t ubrr = 103;
	UBRR0H = (uint8_t)(ubrr >> 8);
	UBRR0L = (uint8_t)ubrr;
	UCSR0B = (1 << TXEN0) | (1 << RXEN0); // enable transmitter & receiver
	UCSR0C = (3 << UCSZ00);
}

void uartTransmit(char c) {
    while (!(UCSR0A & (1 << UDRE0))); // wait until buffer empty
    UDR0 = c;
}

void uartPrint(const char* str) {
    while (*str) {
        uartTransmit(*str++);
    }
}

void uartPrintInt(int num) {
    char buffer[10];
    itoa(num, buffer, 10);
    uartPrint(buffer);
}

void uartPrintFloat(float num) {
    char buffer[20];
    dtostrf(num, 6, 2, buffer); // width=6, precision=2
    uartPrint(buffer);
}

// ----- I2C Functions ----- //

void i2c_init(void) {
    TWSR = 0x00;  // prescaler = 1
    TWBR = 12;    // ~400kHz @ 16MHz
}

void i2c_start(void) {
    TWCR = (1 << TWINT) | (1 << TWSTA) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));
}

void i2c_stop(void) {
    TWCR = (1<<TWINT) | (1<<TWEN) | (1<<TWSTO);
    _delay_us(100);  // Small delay for stop condition
}

void i2c_write(uint8_t data) {
    TWDR = data;
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));
}

uint8_t i2c_read_ack(void) {
    TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA);
    while (!(TWCR & (1 << TWINT)));
    return TWDR;
  }
  
  uint8_t i2c_read_nack(void) {
    TWCR = (1 << TWINT) | (1 << TWEN);
    while (!(TWCR & (1 << TWINT)));
    return TWDR;
}

void mpu6050_set_gyro_range(uint8_t range) {
    i2c_start();
    i2c_write((IMU_ADDR << 1) | 0);
    i2c_write(0x1B);  // GYRO_CONFIG register
    i2c_write(range << 3);  // Bits 4:3 set the range
    i2c_stop();
    
    // Update scale factor based on range
    switch(range) {
        case GYRO_RANGE_250:
            gyro_scale = 131.0f;
            uartPrint("Gyro range set to ±250°/s\r\n");
            break;
        case GYRO_RANGE_500:
            gyro_scale = 65.5f;
            uartPrint("Gyro range set to ±500°/s\r\n");
            break;
        case GYRO_RANGE_1000:
            gyro_scale = 32.8f;
            uartPrint("Gyro range set to ±1000°/s\r\n");
            break;
        case GYRO_RANGE_2000:
            gyro_scale = 16.4f;
            uartPrint("Gyro range set to ±2000°/s\r\n");
            break;
    }
}

void mpu6050_set_acc_range(uint8_t range) {
    i2c_start();
    i2c_write((IMU_ADDR << 1) | 0);
    i2c_write(0x1C);  // GYRO_CONFIG register
    i2c_write(range);  // Bits 4:3 set the range
    i2c_stop();
    
    // Update scale factor based on range
    switch(range) {
        case ACCEL_RANGE_2G:
            acc_scale = 16384.0f;
            uartPrint("Accel range set to 2g\r\n");
            break;
        case ACCEL_RANGE_4G:
            acc_scale = 8192.0f;
            uartPrint("Accel range set to 4g\r\n");
            break;
        case ACCEL_RANGE_8G:
            acc_scale = 4096.0f;
            uartPrint("Accel range set to 8g\r\n");
            break;
        case ACCEL_RANGE_16G:
            acc_scale = 2048.0f;
            uartPrint("Accel range set to 16g\r\n");
            break;
    }
}

void mpu6050_init(void) {
    uartPrint("Initializing MPU6050...\r\n");
    
    // Wake up MPU-6050
    i2c_start();
    i2c_write((IMU_ADDR << 1) | 0);
    i2c_write(0x6B); // PWR_MGMT_1
    i2c_write(0x00);
    i2c_stop();

    _delay_ms(10);

    mpu6050_set_gyro_range(GYRO_RANGE_250);
    mpu6050_set_acc_range(ACCEL_RANGE_2G);

    uartPrint("MPU6050 initialized successfully\r\n");
}

// ----- MPU Functions ----- //
void readIMUCalibration() {
    uint8_t data[14];

    // Write starting register, then repeated start for read
    i2c_start();
    i2c_write((IMU_ADDR << 1) | 0);
    i2c_write(0x3B); // ACCEL_XOUT_H
    i2c_start();
    i2c_write((IMU_ADDR << 1) | 1);
    for (uint8_t i = 0; i < 13; i++) data[i] = i2c_read_ack();
    data[13] = i2c_read_nack();
    i2c_stop();

    // raw data readings (skip temp)
    AccX = ((int16_t)data[0] << 8) | data[1];
    AccY = ((int16_t)data[2] << 8) | data[3];
    AccZ = ((int16_t)data[4] << 8) | data[5];
    GyrX = ((int16_t)data[8] << 8) | data[9];
    GyrY = ((int16_t)data[10] << 8) | data[11];
    GyrZ = ((int16_t)data[12] << 8) | data[13];

    // convert to g
    accX = (float)AccX / acc_scale;
    accY = (float)AccY / acc_scale;
    accZ = (float)AccZ / acc_scale;
    gyrX = (float)GyrX / gyro_scale;
    gyrY = (float)GyrY / gyro_scale;
    gyrZ = (float)GyrZ / gyro_scale;
}

void readIMU() {
    readIMUCalibration();  // Read raw values
    
    // Apply offsets
    accX -= offset_ax;
    accY -= offset_ay;
    accZ -= offset_az;
    gyrX -= offset_gx;
    gyrY -= offset_gy;
    gyrZ -= offset_gz;
}

void calibration() {
    uartPrint("Calibrating ... Please do not move IMU\r\n");

    int bufferSize = 1000;
    float buff_ax=0.0f, buff_ay=0.0f, buff_az=0.0f, buff_gx=0.0f, buff_gy=0.0f ,buff_gz=0.0f;
    int i = 0;

    while (i < (bufferSize + 101)) {
        readIMUCalibration();

        if (i > 100 && i <= (bufferSize+100)) // discard the 100 first readings and sum the rest
        {
            buff_ax = buff_ax + accX;
            buff_ay = buff_ay + accY;
            buff_az = buff_az + accZ;
            buff_gx = buff_gx + gyrX;
            buff_gy = buff_gy + gyrY;
            buff_gz = buff_gz + gyrZ;
        }
        
        if (i==(bufferSize+100)) {
            offset_ax=buff_ax/bufferSize;
            offset_ay=buff_ay/bufferSize;
            offset_az=(buff_az/bufferSize)-1.0f;
            offset_gx=buff_gx/bufferSize;
            offset_gy=buff_gy/bufferSize;
            offset_gz=buff_gz/bufferSize;
        }

        i++;
        _delay_ms(2); // to not get repeated measures
    }

    uartPrint("Calibration complete !");
}

// PWM Initialization
void pwm_init() {
    DDRB |= (1 << SERVO_PWM_PIN);  // Set PWM pin as output
    TCCR1A = (1 << COM1A1) | (1 << WGM11);  // Configure PWM mode
    TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11);  // Set frequency to 50Hz
    ICR1 = 19999;  // Define TOP value for 20ms period

    // Timer2 for PB3 (OC2A)
    DDRB |= (1 << LED_PIN);  // Set PB3 as output
    TCCR2A = (1 << COM2A1) | (1 << WGM21) | (1 << WGM20);  // Fast PWM, non-inverted output on OC2A
    TCCR2B = (1 << CS21);  // Prescaler = 8
    OCR2A = 0;  // Start with LED off
}

// ----- Main ----- //
void controlLED(float accel_x, float yaw) {
    // control of L LED
    if (yaw < -LEDRange || yaw > LEDRange) {
        PORTB |= (1 << L_LED);  // integrated LED L ON
    } else {
        PORTB &= ~(1 << L_LED);  // integrated LED L off
    }

    // control of PB3
    if(accel_x < 0.33 && accel_x > -0.33) {
        OCR2A = BRIGHT1; // PB3 off
    } else if (accel_x > 1.33 || accel_x < -1.33) {
        OCR2A = BRIGHT2; // PB3 on
    } else {
        float abs_accel = fabs(accel_x);  // Use absolute value
        
        // Normalize from 0.33-1.33 range to 0-1
        float normalized = (abs_accel - 0.33f) / (1.33f - 0.33f);
        
        // Clamp to 0-1
        if (normalized < 0.0f) normalized = 0.0f;
        if (normalized > 1.0f) normalized = 1.0f;

        float brightness = BRIGHT2 + normalized * (BRIGHT1 - BRIGHT2);

        // clamp
        if (brightness < 12) brightness = 12;
        if (brightness > 255) brightness = 255;

        OCR2A = (uint8_t)brightness;
    }
}

void set_servo_angle(float yaw) {
    if (yaw > ServoRange) yaw = ServoRange;
    if (yaw < -ServoRange) yaw = -ServoRange;
    OCR1A = 500 + ((yaw + ServoRange) * (2000.0 / (2 * ServoRange)));  // Adjust mapping to ensure range is 500 to 2500
}

void getPitchRoll() {
    pitch = atan2(accY, sqrt(accX * accX + accZ * accZ)) * 180.0f / M_PI;
    
    roll = atan2(-accX, accZ) * 180.0f / M_PI;
}

void getDistance() {
    // Convert accX from g to m/s²
    float accelX_ms2 = accX * 9.81f;
    
    // ignore acceleration less than 0.05g
    if (fabs(accelX_ms2) < 0.05 * 9.81f) { 
        accelX_ms2 = 0.0f;
    }
    
    // Integrate accX to get velocity
    velocityX += accelX_ms2 * 0.01f;
    
    // Integrate velocity to get distance
    distanceX += velocityX * 0.01f;

    if (fabs(accX) < 0.1f) {
        // If barely any acceleration AND low velocity, assume stationary
        velocityX = 0.0f;
    }
}

int main(void) {
	uart_init();
    i2c_init();
    mpu6050_init();
    pwm_init();
    DDRB |= (1 << L_LED);  // Set LED pin as output
    PORTB &= ~(1 << L_LED);  // make LED off initially

    calibration();

    int counter = 0;

    while (1)
    {
        readIMU();
        yaw += gyrZ * 0.01f; // gyroscope z-axis data over time (0.01f seconds delay)
        

        controlLED(accX, yaw);
        set_servo_angle(yaw);
        getPitchRoll();
        getDistance();

        // prints only 100 iterations (1s)
        if (counter >= 100) {
            uartPrint("Acc - X:");
            uartPrintFloat(accX);
            uartPrint(" Y:");
            uartPrintFloat(accY);
            uartPrint(" Z:");
            uartPrintFloat(accZ);
            
            uartPrint(" | Yaw:");
            uartPrintFloat(yaw);
            uartPrint(" Pitch:");
            uartPrintFloat(pitch);
            uartPrint(" Roll:");
            uartPrintFloat(roll);

            uartPrint("| Distance X (in m): ");
            uartPrintFloat(distanceX);

            uartPrint("| Gyro Z: ");
            uartPrintFloat(gyrZ);

            uartPrint("\r\n");

            distanceX = 0.0f;
            counter = 0;
        }
        
        counter++;

        _delay_ms(10);
    }
}