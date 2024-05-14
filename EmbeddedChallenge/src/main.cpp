#include <mbed.h>
#include <arm_math.h>
#include "./drivers/LCD_DISCO_F429ZI.h"

// Define control register addresses and their configurations
#define CTRL_REG1 0x20
#define CTRL_REG1_CONFIG 0b01101111
#define CTRL_REG4 0x23
#define CTRL_REG4_CONFIG 0b00100000
#define OUT_X_L 0x28

// SPI pins
#define GYRO_MOSI PF_9
#define GYRO_MISO PF_8
#define GYRO_SCK PF_7
#define GYRO_CS PC_1

// Configuration values
#define MULTIPLEBYTE_CMD 0x40
#define DUMMY_BYTE 0x00
#define FS_500_SENSITIVITY 0.0175 // sensitivity from the datasheet
#define MAX_GYRO 500
#define MIN_GYRO -500

// Constants for tremor detection
#define BUFFER_SIZE 256  // Number of samples for FFT
#define SAMPLE_RATE 100  // Sampling rate in Hz
#define TREMOR_THRESHOLD 10.0f  // Example threshold for tremor detection
#define SERIOUS_TREMOR_THRESHOLD 20.0f  // Threshold for serious tremor
#define MIN_DURATION 3.0  // Minimum duration in seconds to confirm a tremor
#define SCALING_FACTOR (17.5f * 0.017453292519943295769236907684886f / 1000.0f)

// Gyroscope data buffers
float gyro_data[BUFFER_SIZE];
float fft_output[BUFFER_SIZE];
float duration = 0;
float tremor_intensity = 0;

EventFlags flags;
SPI spi(GYRO_MOSI, GYRO_MISO, GYRO_SCK); // SPI object
DigitalOut GyroCs(GYRO_CS);              // Chip select for SPI
LCD_DISCO_F429ZI lcd;
static BufferedSerial serial_port(USBTX, USBRX, 9600);
DigitalOut led1(LED1); // Green LED
DigitalOut led2(LED2); // Red LED

int Gyro_Init() {
    int ID;

    GyroCs = 0;
    spi.write(0x0F | 0x80); // ID_REG_ADDRESS | READ_CMD
    ID = spi.write(DUMMY_BYTE);
    wait_us(3);
    GyroCs = 1;

    GyroCs = 0;
    spi.write(CTRL_REG1);
    spi.write(CTRL_REG1_CONFIG);
    wait_us(3);
    GyroCs = 1;

    GyroCs = 0;
    spi.write(0x21); // REG_2_ADDRESS
    spi.write(0x00); // REG_2_CONFIG
    wait_us(3);
    GyroCs = 1;

    GyroCs = 0;
    spi.write(CTRL_REG4);
    spi.write(CTRL_REG4_CONFIG);
    wait_us(3);
    GyroCs = 1;

    GyroCs = 0;
    spi.write(0x24); // REG_5_ADDRESS
    spi.write(0x10); // REG_5_CONFIG
    wait_us(3);
    GyroCs = 1;

    return ID;
}

void Gyro_Get_XYZ(float xyz[]) {
    int low = 0;
    int high = 0;
    int16_t x, y, z;

    GyroCs = 0;
    spi.write(OUT_X_L | 0x80 | 0x40); // X_REG_ADDRESS | READ_CMD | MULTIPLEBYTE_CMD
    wait_us(3);
    low = spi.write(DUMMY_BYTE);
    high = spi.write(DUMMY_BYTE);
    wait_us(3);
    GyroCs = 1;
    x = (high << 8) | low;

    GyroCs = 0;
    spi.write(0x2A | 0x80 | 0x40); // Y_REG_ADDRESS | READ_CMD | MULTIPLEBYTE_CMD
    wait_us(3);
    low = spi.write(DUMMY_BYTE);
    high = spi.write(DUMMY_BYTE);
    wait_us(3);
    GyroCs = 1;
    y = (high << 8) | low;

    GyroCs = 0;
    spi.write(0x2C | 0x80 | 0x40); // Z_REG_ADDRESS | READ_CMD | MULTIPLEBYTE_CMD
    wait_us(3);
    low = spi.write(DUMMY_BYTE);
    high = spi.write(DUMMY_BYTE);
    wait_us(3);
    GyroCs = 1;
    z = (high << 8) | low;

    xyz[0] = (x - 13) * FS_500_SENSITIVITY; // 13 is zero_X
    xyz[1] = (y - 22) * FS_500_SENSITIVITY; // 22 is zero_Y
    xyz[2] = (z - 3) * FS_500_SENSITIVITY;  // 3 is zero_Z
}

void spi_cb(int event) {
    flags.set(1);
}

void setupGyro() {
    Gyro_Init();
}

void captureGyroData() {
    uint8_t write_buf[1], read_buf[7];
    int16_t raw_gx, raw_gy, raw_gz;
    float gy;

    for (int i = 0; i < BUFFER_SIZE; i++) {
        write_buf[0] = OUT_X_L | 0x80 | 0x40;
        spi.transfer(write_buf, 7, read_buf, 7, spi_cb);
        flags.wait_all(1);

        raw_gx = (((uint16_t)read_buf[2]) << 8) | ((uint16_t) read_buf[1]);
        raw_gy = (((uint16_t)read_buf[4]) << 8) | ((uint16_t) read_buf[3]);
        raw_gz = (((uint16_t)read_buf[6]) << 8) | ((uint16_t) read_buf[5]);

        gy = ((float) raw_gy) * SCALING_FACTOR;
        gyro_data[i] = gy;

        // Print the actual values for Teleplot
        printf("@GyroData %4.5f\n", gy);

        ThisThread::sleep_for(10ms);
    }
}

void analyzeFrequency() {
    arm_rfft_fast_instance_f32 fft_instance;
    arm_rfft_fast_init_f32(&fft_instance, BUFFER_SIZE);
    arm_rfft_fast_f32(&fft_instance, gyro_data, fft_output, 0);

    float fft_magnitude[BUFFER_SIZE / 2];
    arm_cmplx_mag_f32(fft_output, fft_magnitude, BUFFER_SIZE / 2);

    tremor_intensity = 0;
    for (int i = 0; i < BUFFER_SIZE / 2; i++) {
        float frequency = (float)i * SAMPLE_RATE / BUFFER_SIZE;
        if (frequency >= 3 && frequency <= 6) {
            tremor_intensity += fft_magnitude[i];
        }
    }

    // Send data to Teleplot
    printf("@tremor_intensity %4.2f\n", tremor_intensity);
    printf("@duration %4.2f\n", duration);

    // Debugging output
    printf("Tremor Intensity: %4.2f, Duration: %4.2f\n", tremor_intensity, duration);

    if (tremor_intensity > TREMOR_THRESHOLD) {
        duration += (float)BUFFER_SIZE / SAMPLE_RATE;
        led1 = 1; // Green LED on for detected tremors

        if (duration >= MIN_DURATION) {
            if (tremor_intensity >= SERIOUS_TREMOR_THRESHOLD) {
                led2 = !led2; // Flashing red LED for serious tremors
                ThisThread::sleep_for(500ms); // Adjust flashing rate if needed
            } else {
                led2 = 1; // Solid red LED for confirmed tremors
            }
        } else {
            led2 = 0; // Turn off red LED if duration is not sufficient
        }
    } else {
        duration = 0;
        led1 = 0; // Green LED off
        led2 = 0; // Red LED off
    }
}

void initializeLCD() {
    lcd.Clear(LCD_COLOR_WHITE);
    BSP_LCD_SetFont(&Font16);
    lcd.DisplayStringAt(0, LINE(1), (uint8_t *)"Parkinson's Tremor", CENTER_MODE);
    lcd.DisplayStringAt(0, LINE(2), (uint8_t *)"Detector", CENTER_MODE);
    BSP_LCD_SetFont(&Font24);
}

void displayTremorInfo(float intensity, float duration) {
    char buffer[50];
    lcd.ClearStringLine(10);
    sprintf(buffer, "Intensity: %.2f", intensity);
    lcd.DisplayStringAt(0, LINE(10), (uint8_t *)buffer, CENTER_MODE);
    sprintf(buffer, "Duration: %.2f s", duration);
    lcd.DisplayStringAt(0, LINE(12), (uint8_t *)buffer, CENTER_MODE);
}

int main() {
    printf("Starting program\n");
    setupGyro();
    initializeLCD();

    while (1) {
        captureGyroData();
        analyzeFrequency();
        displayTremorInfo(tremor_intensity, duration);
    }
}
