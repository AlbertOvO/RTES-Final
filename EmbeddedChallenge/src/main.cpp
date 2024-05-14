#include "mbed.h"
#include "arm_math.h"  // CMSIS-DSP library
#include "./drivers/LCD_DISCO_F429ZI.h"

// Define control register addresses and their configurations
#define CTRL_REG1 0x20
#define CTRL_REG1_CONFIG 0x6F  // Binary literal changed to hexadecimal
#define CTRL_REG4 0x23
#define CTRL_REG4_CONFIG 0x20  // Binary literal changed to hexadecimal
#define OUT_X_L 0x28

// SPI pins
#define GYRO_MOSI PF_9
#define GYRO_MISO PF_8
#define GYRO_SCK PF_7
#define GYRO_CS PC_1

// Constants for tremor detection
#define BUFFER_SIZE 36  // Number of samples for FFT
#define SAMPLE_RATE 10  // Sampling rate in Hz

// Gyroscope data buffers
float gyro_data[BUFFER_SIZE];
float fft_output[BUFFER_SIZE];
float duration = 0;
float tremor_intensity = 0;

SPI spi(GYRO_MOSI, GYRO_MISO, GYRO_SCK); // SPI object
DigitalOut GyroCs(GYRO_CS);              // Chip select for SPI
LCD_DISCO_F429ZI lcd;
static BufferedSerial serial_port(USBTX, USBRX, 9600);
DigitalOut led1(LED1); // Green LED
DigitalOut led2(LED2); // Red LED

int Gyro_Init() {
    int ID = 0;  // Declare and initialize ID variable

    GyroCs = 0;
    spi.write(0x8F); // Command to read the WHO_AM_I register, assuming 0x8F is correct
    ID = spi.write(0x00); // Dummy byte to read back the ID
    GyroCs = 1;

    // Configure CTRL_REG1
    GyroCs = 0;
    spi.write(CTRL_REG1 | 0x80); // Ensure correct bit is set for write operation
    spi.write(CTRL_REG1_CONFIG);
    GyroCs = 1;

    // Configure CTRL_REG4
    GyroCs = 0;
    spi.write(CTRL_REG4 | 0x80); // Ensure correct bit is set for write operation
    spi.write(CTRL_REG4_CONFIG);
    GyroCs = 1;

    // More configuration as needed

    return ID; // Return the read ID
}



void Gyro_Get_XYZ(float xyz[]) {
    GyroCs = 0;
    spi.write(OUT_X_L | 0xC0); // Command to read all axes with auto increment
    xyz[0] = spi.write(0x00) * 0.0175; // Convert to engineering units
    xyz[1] = spi.write(0x00) * 0.0175;
    xyz[2] = spi.write(0x00) * 0.0175;
    GyroCs = 1;
}

void setupGyro() {
    Gyro_Init();
}

void captureGyroData() {
    float xyz[3];
    for (int i = 0; i < BUFFER_SIZE; i++) {
        Gyro_Get_XYZ(xyz);
        gyro_data[i] = xyz[1]; // Example: use y-axis
        printf("@GyroData %4.5f\n", gyro_data[i]);
        ThisThread::sleep_for(100ms);
    }
}

void analyzeFrequency() {
    arm_rfft_fast_instance_f32 fft_instance;
    arm_rfft_fast_init_f32(&fft_instance, BUFFER_SIZE);
    arm_rfft_fast_f32(&fft_instance, gyro_data, fft_output, 0);

    float max_magnitude = 0.0;
    for (int i = 1; i < BUFFER_SIZE / 2; i++) {  // Start at 1 to skip DC component
        float mag = sqrtf(fft_output[2*i]*fft_output[2*i] + fft_output[2*i+1]*fft_output[2*i+1]);
        if (mag > max_magnitude) {
            max_magnitude = mag;
            tremor_intensity = i * ((float)SAMPLE_RATE / BUFFER_SIZE);
        }
    }

    printf("Max Tremor Intensity at Frequency: %4.2f Hz with Magnitude: %4.2f\n", tremor_intensity, max_magnitude);
}

void initializeLCD() {
    lcd.Clear(LCD_COLOR_WHITE);
    lcd.SetBackColor(LCD_COLOR_WHITE);
    lcd.SetTextColor(LCD_COLOR_BLACK);
    lcd.SetFont(&Font16);

    // Adjust these coordinates based on your display size and desired layout
    lcd.DisplayStringAt(0, 40, (uint8_t *)"Parkinson's Tremor", CENTER_MODE); // Adjusted Y position
    lcd.DisplayStringAt(0, 60, (uint8_t *)"Detector", CENTER_MODE); // Adjusted Y position
}

void displayTremorInfo(float intensity, float duration) {
    char buffer[50];

    // Clear previous text if overlapping issues occur
    lcd.ClearStringLine(10);
    lcd.ClearStringLine(12);

    sprintf(buffer, "Intensity: %.2f", intensity);
    lcd.DisplayStringAt(0, 120, (uint8_t *)buffer, CENTER_MODE); // Adjusted Y position

    sprintf(buffer, "Duration: %.2f s", duration);
    lcd.DisplayStringAt(0, 140, (uint8_t *)buffer, CENTER_MODE); // Adjusted Y position
}

int main() {
    initializeLCD();
    float exampleIntensity = 15.5;
    float exampleDuration = 2.5;

    while (true) {
        displayTremorInfo(exampleIntensity, exampleDuration);
        ThisThread::sleep_for(5s);  // Use chrono literals for time durations
    }
}
