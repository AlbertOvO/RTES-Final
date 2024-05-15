// Embedded Challenge
// Youtube video link: https://youtu.be/yd1xXl1Cz9I
// Team memembers:
// Haonan Li (hl4798)
// Jiawei He (jh8824)
// Devakumar Katta (dK4945)
// Abdul Samad Zaheer Khan (ak9943)

#include "mbed.h"
#include "drivers/LCD_DISCO_F429ZI.h"
#include "arm_math.h"

// Serial communication for debugging
BufferedSerial pc(USBTX, USBRX);

// LCD instance
LCD_DISCO_F429ZI lcd;

// SPI and gyroscope configuration
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

void spiCallback(int event) {
    eventFlags.set(SPI_FLAG);
}

void setupSPI(SPI &spi, uint8_t *txBuffer, uint8_t *rxBuffer) {
    spi.transfer(txBuffer, 2, rxBuffer, 2, spiCallback);
    eventFlags.wait_all(SPI_FLAG);
}

void initializeGyro(SPI &spi) {
    uint8_t txBuffer[2] = {0}, rxBuffer[2] = {0};

    txBuffer[0] = CTRL_REG1_ADDR;
    txBuffer[1] = CTRL_REG1_VAL;
    setupSPI(spi, txBuffer, rxBuffer);

    txBuffer[0] = CTRL_REG4_ADDR;
    txBuffer[1] = CTRL_REG4_VAL;
    setupSPI(spi, txBuffer, rxBuffer);

    txBuffer[0] = CTRL_REG3_ADDR;
    txBuffer[1] = CTRL_REG3_VAL;
    setupSPI(spi, txBuffer, rxBuffer);
}

void fetchGyroData(SPI &spi, float &x, float &y, float &z) {
    uint8_t txBuffer[7] = {0}, rxBuffer[7] = {0};
    txBuffer[0] = GYRO_OUT_X_L | 0xC0;

    spi.transfer(txBuffer, 7, rxBuffer, 7, spiCallback);
    eventFlags.wait_all(SPI_FLAG);

    int16_t rawX = (int16_t)(((uint16_t)rxBuffer[2] << 8) | (uint16_t)rxBuffer[1]);
    int16_t rawY = (int16_t)(((uint16_t)rxBuffer[4] << 8) | (uint16_t)rxBuffer[3]);
    int16_t rawZ = (int16_t)(((uint16_t)rxBuffer[6] << 8) | (uint16_t)rxBuffer[5]);

    x = rawX * 0.0003054f;
    y = rawY * 0.0003054f;
    z = rawZ * 0.0003054f;
}

void displayTremorLevel(float tremorLevel) {
    char buffer[32];
    if (tremorLevel > 1.0f && tremorLevel < 3.0f) {
        lcd.Clear(LCD_COLOR_GREEN);
        lcd.SetBackColor(LCD_COLOR_GREEN);
        lcd.SetTextColor(LCD_COLOR_WHITE);
        sprintf(buffer, "Mild Tremor: %.2f", tremorLevel);
    } else if (tremorLevel >= 3.0f) {
        lcd.Clear(LCD_COLOR_RED);
        lcd.SetBackColor(LCD_COLOR_RED);
        lcd.SetTextColor(LCD_COLOR_WHITE);
        sprintf(buffer, "Severe Tremor: %.2f", tremorLevel);
    } else {
        lcd.Clear(LCD_COLOR_WHITE);
        lcd.SetBackColor(LCD_COLOR_WHITE);
        lcd.SetTextColor(LCD_COLOR_BLACK);
        sprintf(buffer, "No Tremor: %.2f", tremorLevel);
    }
    lcd.DisplayStringAt(0, LINE(5), (uint8_t *)"Tremor Level", CENTER_MODE);
    lcd.DisplayStringAt(0, LINE(7), (uint8_t *)buffer, CENTER_MODE);
}

int main() {
    // Initialize the serial port for debugging
    pc.set_baud(9600);
    printf("Starting application...\n");

    // Initialize the LCD
    lcd.Clear(LCD_COLOR_WHITE);
    lcd.SetBackColor(LCD_COLOR_WHITE);
    lcd.SetTextColor(LCD_COLOR_BLACK);
    lcd.DisplayStringAt(0, LINE(5), (uint8_t *)"Tremor Detector Initialized", CENTER_MODE);
    printf("LCD initialization complete.\n");

    // Configure SPI device
    SPI spiDevice(PF_9, PF_8, PF_7, PC_1, use_gpio_ssel);
    printf("Initializing gyroscope...\n");
    initializeGyro(spiDevice);
    printf("Gyroscope initialization complete.\n");

    float x = 0, y = 0, z = 0;
    while (true) {
        fetchGyroData(spiDevice, x, y, z);
        printf("Gyro data: x=%f, y=%f, z=%f\n", x, y, z);

        // float tremorLevel = (fabs(x) + fabs(y) + fabs(z)) / 3.0f;
        float tremorLevel = (fabs(x) + fabs(y) + fabs(z)) / 1.0f;
        // float tremorLevel = (fabs(x) + fabs(z)) / 2.0f;
        displayTremorLevel(tremorLevel);

        // ThisThread::sleep_for(500ms);
        ThisThread::sleep_for(150ms);
    }
}