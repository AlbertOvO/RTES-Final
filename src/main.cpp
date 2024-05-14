// Embedded Challenge
// Team memembers:
// Haonan li (hl4798)
// Jiawei he (jh8824)
// devaKUMAR KATTA (dK4945)
// Abdul samad zaheer khan (ak9943)
#include "mbed.h"
#include "arm_math.h"
#include "drivers/LCD_DISCO_F429ZI.h"

// Configuration constants
#define SAMPLE_COUNT 64
#define FFT_SIZE 128
#define INTERVAL 10ms
#define GYRO_SENSITIVITY 0.0003054f

#define CTRL_REG1_ADDR 0x20
#define CTRL_REG1_VAL 0x6F
#define CTRL_REG4_ADDR 0x23
#define CTRL_REG4_VAL 0x20
#define CTRL_REG3_ADDR 0x22
#define CTRL_REG3_VAL 0x08
#define GYRO_OUT_X_L 0x28
#define SPI_FLAG 1
#define DATA_FLAG 2

EventFlags eventFlags;
BufferedSerial pc(USBTX, USBRX);
LCD_DISCO_F429ZI lcd;
DigitalOut greenLED(LED1);
DigitalOut redLED(LED2);

arm_rfft_fast_instance_f32 fftInstance;

float gyroX[SAMPLE_COUNT * 2];
float fftX[FFT_SIZE];
float gyroY[SAMPLE_COUNT * 2];
float fftY[FFT_SIZE];
float gyroZ[SAMPLE_COUNT * 2];
float fftZ[FFT_SIZE];

// Callback function for SPI transfer completion
void spiCallback(int event) {
    eventFlags.set(SPI_FLAG);
}

// Callback function for data ready interrupt
void dataReadyCallback() {
    eventFlags.set(DATA_FLAG);
}

// Function to perform SPI transfer and wait for completion
void setupSPI(SPI &spi, uint8_t *txBuffer, uint8_t *rxBuffer) {
    spi.transfer(txBuffer, 2, rxBuffer, 2, spiCallback);
    eventFlags.wait_all(SPI_FLAG);
}

// Function to initialize the gyroscope
void initializeGyro(SPI &spi) {
    uint8_t txBuffer[2] = {0}, rxBuffer[2] = {0};

    // Setup CTRL_REG1
    txBuffer[0] = CTRL_REG1_ADDR;
    txBuffer[1] = CTRL_REG1_VAL;
    setupSPI(spi, txBuffer, rxBuffer);

    // Setup CTRL_REG4
    txBuffer[0] = CTRL_REG4_ADDR;
    txBuffer[1] = CTRL_REG4_VAL;
    setupSPI(spi, txBuffer, rxBuffer);

    // Setup CTRL_REG3
    txBuffer[0] = CTRL_REG3_ADDR;
    txBuffer[1] = CTRL_REG3_VAL;
    setupSPI(spi, txBuffer, rxBuffer);
}

// Function to fetch gyroscope data and scale it
void fetchGyroData(SPI &spi, float &x, float &y, float &z) {
    uint8_t txBuffer[7] = {0}, rxBuffer[7] = {0};
    txBuffer[0] = GYRO_OUT_X_L | 0xC0;

    spi.transfer(txBuffer, 7, rxBuffer, 7, spiCallback);
    eventFlags.wait_all(SPI_FLAG);

    // Combine high and low bytes for raw data
    int16_t rawX = (int16_t)(((uint16_t)rxBuffer[2] << 8) | (uint16_t)rxBuffer[1]);
    int16_t rawY = (int16_t)(((uint16_t)rxBuffer[4] << 8) | (uint16_t)rxBuffer[3]);
    int16_t rawZ = (int16_t)(((uint16_t)rxBuffer[6] << 8) | (uint16_t)rxBuffer[5]);

    // Scale the raw data
    x = rawX * GYRO_SENSITIVITY;
    y = rawY * GYRO_SENSITIVITY;
    z = rawZ * GYRO_SENSITIVITY;
}

// Function to analyze tremor level and display the results on LCD
void analyzeAndDisplayTremor(float tremorLevel) {
    lcd.Clear(LCD_COLOR_WHITE);
    lcd.SetBackColor(LCD_COLOR_WHITE);
    lcd.SetTextColor(LCD_COLOR_BLACK);
    lcd.DisplayStringAt(0, LINE(5), (uint8_t *)"Tremor Detector", CENTER_MODE);
    char intensity[30];
    if (tremorLevel > 1.0 && tremorLevel < 3.0) {
        lcd.Clear(LCD_COLOR_GREEN);
        lcd.SetBackColor(LCD_COLOR_GREEN);
        lcd.SetTextColor(LCD_COLOR_WHITE);
        lcd.DisplayStringAt(0, LINE(5), (uint8_t *)"Mild Tremor Detected", CENTER_MODE);
        sprintf(intensity, "Intensity: %.2f", tremorLevel);
        lcd.DisplayStringAt(0, LINE(7), (uint8_t *)intensity, CENTER_MODE);
        greenLED = 1;
        redLED = 0;
    } else if (tremorLevel >= 3.0) {
        lcd.Clear(LCD_COLOR_RED);
        lcd.SetBackColor(LCD_COLOR_RED);
        lcd.SetTextColor(LCD_COLOR_WHITE);
        lcd.DisplayStringAt(0, LINE(5), (uint8_t *)"Severe Tremor Detected", CENTER_MODE);
        sprintf(intensity, "Intensity: %.2f", tremorLevel);
        lcd.DisplayStringAt(0, LINE(7), (uint8_t *)intensity, CENTER_MODE);
        greenLED = 0;
        redLED = !redLED;
        ThisThread::sleep_for(10ms);
    } else {
        greenLED = 0;
        redLED = 0;
    }
}

// Function to perform FFT analysis and update tremor data
void performFFTAndAnalyze(int index, float x, float y, float z) {
    gyroX[2 * index] = x;
    gyroY[2 * index] = y;
    gyroZ[2 * index] = z;

    if (index == SAMPLE_COUNT - 1) {
        arm_rfft_fast_f32(&fftInstance, gyroX, fftX, 0);
        arm_rfft_fast_f32(&fftInstance, gyroY, fftY, 0);
        arm_rfft_fast_f32(&fftInstance, gyroZ, fftZ, 0);

        float tremorX = 0, tremorY = 0, tremorZ = 0;
        for (int i = 3; i <= 6; ++i) {
            tremorX += sqrtf(gyroX[2 * i] * gyroX[2 * i] + gyroX[2 * i + 1] * gyroX[2 * i + 1]);
            tremorY += sqrtf(gyroY[2 * i] * gyroY[2 * i] + gyroY[2 * i + 1] * gyroY[2 * i + 1]);
            tremorZ += sqrtf(gyroZ[2 * i] * gyroZ[2 * i] + gyroZ[2 * i + 1] * gyroZ[2 * i + 1]);
        }

        float averageTremor = (tremorX + tremorY + tremorZ) / 3.0;
        analyzeAndDisplayTremor(averageTremor);

        memset(gyroX, 0, sizeof(gyroX));
        memset(gyroY, 0, sizeof(gyroY));
        memset(gyroZ, 0, sizeof(gyroZ));
    }
}

int main() {
    // Initialize the serial port
    pc.set_baud(9600);

    // Configure SPI device
    SPI spiDevice(PF_9, PF_8, PF_7, PC_1, use_gpio_ssel);
    InterruptIn dataInterrupt(PA_2, PullDown);
    dataInterrupt.rise(&dataReadyCallback);

    spiDevice.format(8, 3);
    spiDevice.frequency(1'000'000);

    // Initialize gyroscope
    initializeGyro(spiDevice);

    // Set data flag if already available
    if (!(eventFlags.get() & DATA_FLAG) && (dataInterrupt.read() == 1)) {
        eventFlags.set(DATA_FLAG);
    }

    // Initialize FFT instance
    arm_rfft_fast_init_f32(&fftInstance, FFT_SIZE);

    int sampleIndex = 0;

    while (true) {
        float x, y, z;
        // Wait for data ready event
        eventFlags.wait_all(DATA_FLAG);
        // Fetch gyro data
        fetchGyroData(spiDevice, x, y, z);

        // Perform FFT and analyze tremor data
        performFFTAndAnalyze(sampleIndex, x, y, z);
        sampleIndex = (sampleIndex + 1) % SAMPLE_COUNT;

        // Sleep for the defined interval
        ThisThread::sleep_for(INTERVAL);
    }
}
