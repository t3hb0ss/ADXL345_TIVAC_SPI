#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/ssi.h"
#include "driverlib/sysctl.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"


//*******************************************************
//                      ADXL345 Definitions
//*******************************************************

#define READ    0x8000

//ADXL Register Map
#define DEVID           0x0000  //Device ID Register
#define THRESH_TAP      0x1D00  //Tap Threshold
#define OFSX            0x1E00  //X-axis offset
#define OFSY            0x1F00  //Y-axis offset
#define OFSZ            0x2000  //Z-axis offset
#define DUR             0x2100  //Tap Duration
#define Latent          0x2200  //Tap latency
#define Window          0x2300  //Tap window
#define THRESH_ACT      0x2400  //Activity Threshold
#define THRESH_INACT    0x2500  //Inactivity Threshold
#define TIME_INACT      0x2600  //Inactivity Time
#define ACT_INACT_CTL   0x2700  //Axis enable control for activity and inactivity detection
#define THRESH_FF       0x2800  //free-fall threshold
#define TIME_FF         0x2900  //Free-Fall Time
#define TAP_AXES        0x2A00  //Axis control for tap/double tap
#define ACT_TAP_STATUS  0x2B00  //Source of tap/double tap
#define BW_RATE         0x2C00  //Data rate and power mode control
#define POWER_CTL       0x2D00  //Power Control Register
#define INT_ENABLE      0x2E00  //Interrupt Enable Control
#define INT_MAP         0x2F00  //Interrupt Mapping Control
#define INT_SOURCE      0x3000  //Source of interrupts
#define DATA_FORMAT     0x3100  //Data format control
#define DATAX0          0x3200  //X-Axis Data 0
#define DATAX1          0x3300  //X-Axis Data 1
#define DATAY0          0x3400  //Y-Axis Data 0
#define DATAY1          0x3500  //Y-Axis Data 1
#define DATAZ0          0x3600  //Z-Axis Data 0
#define DATAZ1          0x3700  //Z-Axis Data 1
#define FIFO_CTL        0x3800  //FIFO control
#define FIFO_STATUS     0x3900  //FIFO status

//Power Control Register Bits
#define WU_0        (1<<0)    //Wake Up Mode - Bit 0
#define WU_1        (1<<1)    //Wake Up mode - Bit 1
#define SLEEP       (1<<2)    //Sleep Mode
#define MEASURE     (1<<3)    //Measurement Mode
#define AUTO_SLP    (1<<4)    //Auto Sleep Mode bit
#define LINK        (1<<5)    //Link bit

//Interrupt Enable/Interrupt Map/Interrupt Source Register Bits
#define OVERRUN     (1<<0)
#define WATERMARK   (1<<1)
#define FREE_FALL   (1<<2)
#define INACTIVITY  (1<<3)
#define ACTIVITY    (1<<4)
#define DOUBLE_TAP  (1<<5)
#define SINGLE_TAP  (1<<6)
#define DATA_READY  (1<<7)

//Data Format Bits
#define RANGE_0     (1<<0)
#define RANGE_1     (1<<1)
#define JUSTIFY     (1<<2)
#define FULL_RES    (1<<3)

#define INT_INVERT  (1<<5)
#define SPI         (1<<6)
#define SELF_TEST   (1<<7)

#define PIN_LOW 0x00
#define PIN_HIGH 0xFF

//*****************************************************************************
//
// This function sets up UART0 to be used for a console to display information
// as the example is running.
//
//*****************************************************************************

void
InitConsole(void)
{
    //
    // Enable GPIO port A which is used for UART0 pins.
    // TODO: change this to whichever GPIO port you are using.
    //

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

    //
    // Configure the pin muxing for UART0 functions on port A0 and A1.
    // This step is not necessary if your part does not support pin muxing.
    // TODO: change this to select the port/pin you are using.
    //

    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);

    //
    // Enable UART0 so that we can configure the clock.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

    //
    // Use the internal 16MHz oscillator as the UART clock source.
    //
    UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);

    //
    // Select the alternate (UART) function for these pins.
    // TODO: change this to select the port/pin you are using.
    //
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //
    // Initialize the UART for console I/O.
    //
    UARTStdioConfig(0, 115200, 16000000);

}

//*****************************************************************************
 //
 // This function sets up SSI0 for communication with manual Fss clocking
 //
 //*****************************************************************************
 void
 InitADXL345SSI(void)
 {
//
// The SSI0 peripheral must be enabled for use.
//
SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);

//
// For this example SSI0 is used with PortA[5:2].
//
SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

//
// Configure the pin muxing for SSI0 functions on port A2, A3, A4, and A5.
//
GPIOPinConfigure(GPIO_PA2_SSI0CLK); // Clock is PA_2
// GPIOPinConfigure(GPIO_PA3_SSI0FSS);
GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_3); // PA_3 is manually clocked
GPIOPinConfigure(GPIO_PA4_SSI0RX); // MISO is PA_4
GPIOPinConfigure(GPIO_PA5_SSI0TX); // MOSI is PA_5
GPIOPinTypeGPIOOutput(GPIO_PORTA_BASE, GPIO_PIN_3);

//
// Configure the GPIO settings for the SSI pins. This function also gives
// control of these pins to the SSI hardware.
// The pins are assigned as follows:
// PA5 - SSI0Tx
// PA4 - SSI0Rx
// PA3 - SSI0Fss - CONFIGURED FOR MANUAL CLOCKING
// PA2 - SSI0CLK
//
GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_5 | GPIO_PIN_4 | GPIO_PIN_2);

//
// Configure and enable the SSI port for SPI master mode. Use SSI0,
// system clock supply, idle clock level high and active low clock in
// freescale SPI mode, master mode, 1MHz SSI frequency, and 8-bit data.
//
SSIConfigSetExpClk(SSI0_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_3, SSI_MODE_MASTER, 1000000, 8);

//
// delay 5 cycles to allow processing of port configuration settings
//
    SysCtlDelay(5);

//
// Enable the SSI0 module.
//
SSIEnable(SSI0_BASE);

 }

//****************************************************************************
 //
 // This allows user-controlled data frame lengths for SSI communication
 //
 //****************************************************************************
 void receiveDataSPI(uint32_t registerAddress, uint32_t frameLength, uint32_t * data)
 {
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, PIN_LOW);
    SSIDataPut(SSI0_BASE, READ|MULT_READ|registerAddress);

    while(frameLength)
    {
        SSIDataPut(SSI0_BASE, 0x00);
        frameLength--;
    }

GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, PIN_HIGH);
}


//*****************************************************************************
//
// This function sends initialization and configuration data via SPI to the ADXL345
//
//*****************************************************************************
void sendDataSPI(uint32_t modulenumber, uint32_t registerAddress, uint32_t registerData)
 {
    if(modulenumber==0){
        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, PIN_LOW);
        SSIDataPut(SSI0_BASE, registerAddress);
        SSIDataPut(SSI0_BASE, registerData);
        while(SSIBusy(SSI0_BASE)){}
        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, PIN_HIGH);
    }
}

//*****************************************************************************
//
// This function receives SPI Data from the ADXL345
//
//*****************************************************************************
 void receiveDataSPI(uint32_t modulenumber, uint32_t registerAddress, uint32_t frameLength)
 {
    if(modulenumber==0){
        GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, PIN_LOW);
        SSIDataPut(SSI0_BASE, READ|MULT_READ|registerAddress);
    }

    while(frameLength)
    {
        SSIDataPut(SSI0_BASE, 0x00);
        frameLength--;
    }

    while(SSIBusy(SSI0_BASE)){}
    GPIOPinWrite(GPIO_PORTA_BASE, GPIO_PIN_3, PIN_HIGH);

}

//*****************************************************************************
//
// Configure SSI0 in master Freescale (SPI) mode.  This example will send out
// 3 bytes of data, then wait for 3 bytes of data to come in.  This will all be
// done using the polling method.
//
//*****************************************************************************

int
main(void)

{
    //  float x_value;
    uint32_t x_value_raw;
    //  float y_value;
    uint32_t y_value_raw;
    //  float z_value;
    uint32_t z_value_raw;

    //
    // This buffer will be for reading the SSI Rx FIFO
    //
    uint32_t pui32DataRx[7];

    uint32_t dataready;
    //uint32_t pui32DataRx[6]; redundant variable

    //
    // Set the clocking to run directly from the external crystal/oscillator.
    //
    SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_OSC | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

    //
    // Set up the serial console to use for displaying messages.  This is
    // just for this example program and is not needed for SSI operation.
    //
    InitConsole();

    //
    // Set up the custom SSI SPI Interface to the ADXL345 with manual FSS Clocking
    // on SSI Port 0
    //
    InitADXL345SSI();

    //
    // Read any residual data from the SSI port.  This makes sure the receive
    // FIFOs are empty, so we don't read any unwanted junk.  This is done here
    // because the SPI SSI mode is full-duplex, which allows you to send and
    // receive at the same time.  The SSIDataGetNonBlocking function returns
    // "true" when data was returned, and "false" when no data was returned.
    // The "non-blocking" function checks if there is any data in the receive
    // FIFO and does not "hang" if there isn't.
    //
    //while(SSIDataGetNonBlocking(SSI0_BASE, &pui32DataRx[0])){} which one to use?
    while(SSIDataGetNonBlocking(SSI0_BASE, &pui32DataRx[6])){}

    sendDataSPI(0, 0x31, 0x01); // Put accelerometer in +/-8g mode, DATA_FORMAT = 0x31
    //sendDataSPI(0, BW_RATE, 0x0A); // Set Output Rate to 100Hz
    //^^will be used in future for setting higher data rate
    sendDataSPI(0, 0x2D, 0x08); // Put accelerometer into measurement mode, POWER_CTL = 0x2D

    //
    // Display the ADXL345 Configuration Setup on the console.
    //
    UARTprintf("SSI ->\n");
    UARTprintf("  Mode: SPI\n");
    UARTprintf("  Data: 16-bit\n\n");

    while(1){

        // Clear the SSI0 Rx FIFO
    while(SSIDataGetNonBlocking(SSI0_BASE, &pui32DataRx[6])){}

    // Wait for SSI0 to be finished
    while(SSIBusy(SSI0_BASE)){}

    // Read the values from the ADXL345 sensor
    receiveDataSPI(0, 0x32, 6);

    // Store sensor axis acceleration values in the Rx Data buffer
    SSIDataGet(SSI0_BASE, &pui32DataRx[0]);
    SSIDataGet(SSI0_BASE, &pui32DataRx[1]);
    SSIDataGet(SSI0_BASE, &pui32DataRx[2]);
    SSIDataGet(SSI0_BASE, &pui32DataRx[3]);
    SSIDataGet(SSI0_BASE, &pui32DataRx[4]);
    SSIDataGet(SSI0_BASE, &pui32DataRx[5]);

    // Concatenate axis results
    x_axis = (pui32DataRx[1]<<8)|pui32DataRx[0];
    y_axis = (pui32DataRx[3]<<8)|pui32DataRx[2];
    z_axis = (pui32DataRx[5]<<8)|pui32DataRx[4];

    // Display axes results
    UARTprintf("%d\t%d\t%d\n", x_axis, y_axis, z_axis);
     }

    return(0);

}