
/*******************************************************************************
 * MSP432 ADC14 -  Differential Mode with 2's Compliment
 *
 * Description: This example will use the ADC14 module mode in differential
 * mode and take advantage of the 2's compliment mode. One ADC memory location
 * is set to sample the differential voltage between terminals A0 and A1. The
 * sample timer is used to periodically sample the difference in the value,
 * the value is converted is float, and the value is stored in memory for the
 * user to observe in the debugger.
 *
 *                MSP432P401
 *             ------------------
 *         /|\|                  |
 *          | |                  |
 *          --|RST         P5.5  |<----- A0 In
 *            |            P5.4  |<----- A1 In
 *            |                  |
 *            |                  |
 *
 ******************************************************************************/
/* DriverLib Includes */
#include <ti/devices/msp432p4xx/driverlib/driverlib.h>

/* Standard Includes */
#include <stdint.h>

#include <string.h>

/* Statics */
//static bool isHigh(uint16_t result);
static void serviceBus(bool loop);
#define REVEAL_TRUE_ADC
#ifdef REVEAL_TRUE_ADC
static float convertToFloat(uint16_t result);
#endif

#define BIT_LIST_SIZE 1024
static int bitList[BIT_LIST_SIZE];
static uint16_t bitListIdx = 0;

int main(void)
{
    /* Halting WDT */
    MAP_WDT_A_holdTimer();
#if 1
    int i;
    for (i = 0; i < BIT_LIST_SIZE; i++)
    {
        bitList[i] = -1;
    }
#endif
    MAP_Interrupt_enableSleepOnIsrExit();

#if 1
    /* Enabling the FPU with stacking enabled (for use within ISR) */
    MAP_FPU_enableModule();
    MAP_FPU_enableLazyStacking();
#endif
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P1, GPIO_PIN0);
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN0);
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN1);
    MAP_GPIO_setAsOutputPin(GPIO_PORT_P2, GPIO_PIN2);

    MAP_ADC14_enableModule();
#if 1
    /* Initializing ADC (MCLK/1/1) */
    MAP_ADC14_initModule(ADC_CLOCKSOURCE_MCLK, ADC_PREDIVIDER_1, ADC_DIVIDER_1,
            0);
#else
    /* Initializing ADC (MODCLK/1/1) */
    MAP_ADC14_initModule(ADC_CLOCKSOURCE_ADCOSC, ADC_PREDIVIDER_1, ADC_DIVIDER_1,
            0);
#endif
    /*1Msps *whoa!**/
    MAP_ADC14_setPowerMode(ADC_UNRESTRICTED_POWER_MODE);
    //MAP_ADC14_setSampleHoldTime(ADC_PULSE_WIDTH_8, ADC_PULSE_WIDTH_8);

    /* Configuring ADC Memory (ADC_MEM0 A0/A1 Differential) in repeat mode */
    MAP_ADC14_configureSingleSampleMode(ADC_MEM0, true);
    MAP_ADC14_configureConversionMemory(ADC_MEM0, ADC_VREFPOS_AVCC_VREFNEG_VSS,
            ADC_INPUT_A0, true);

    /* Setting up GPIO pins as analog inputs */
    MAP_GPIO_setAsPeripheralModuleFunctionInputPin(GPIO_PORT_P5,
            GPIO_PIN5 | GPIO_PIN4, GPIO_TERTIARY_MODULE_FUNCTION);

    /* Switching data mode to 2's Complement mode */
    MAP_ADC14_setResultFormat(ADC_SIGNED_BINARY);

    /* Configuring SysTick to trigger at 1500 (MCLK is 1.5MHz so this will
     * make it toggle every 1/1000 s) */
    MAP_SysTick_enableModule();
    MAP_SysTick_setPeriod(1500);

    /* Enabling sample timer in auto iteration mode and interrupts*/
    MAP_ADC14_enableSampleTimer(ADC_AUTOMATIC_ITERATION);
    MAP_ADC14_enableInterrupt(ADC_INT0);
    MAP_SysTick_enableInterrupt();

    /* Enabling Interrupts */
    MAP_Interrupt_enableInterrupt(INT_ADC14);
    MAP_Interrupt_enableMaster();

    /* Triggering the start of the sample */
    MAP_ADC14_enableConversion();
    MAP_ADC14_toggleConversionTrigger();

    /* Going to sleep */
    while (1)
    {
        MAP_PCM_gotoLPM0();
    }
}

void SysTick_Handler(void)
{
    serviceBus(false);
}

/*==========*/

#if 1
// 500ksps
#define __READER_IEBUS_IDLE_MINIMUM_HIGH_COUNT          100
#define __READER_IEBUS_START_BIT_MINIMUM_LOW_COUNT      80
#define __READER_IEBUS_LOGIC_BIT_MINIMUM_LOW_COUNT      5
#define __READER_IEBUS_LOGIC_HIGH_MAXIMUM_LOW_COUNT     15
#elif 0
// 1Msps
#define __READER_IEBUS_IDLE_MINIMUM_HIGH_COUNT          200
#define __READER_IEBUS_START_BIT_MINIMUM_LOW_COUNT      160
#define __READER_IEBUS_LOGIC_BIT_MINIMUM_LOW_COUNT      10
#define __READER_IEBUS_LOGIC_HIGH_MAXIMUM_LOW_COUNT     30
#else //guesswork
#define __READER_IEBUS_IDLE_MINIMUM_HIGH_COUNT          20
#define __READER_IEBUS_START_BIT_MINIMUM_LOW_COUNT      16
#define __READER_IEBUS_LOGIC_BIT_MINIMUM_LOW_COUNT      1
#define __READER_IEBUS_LOGIC_HIGH_MAXIMUM_LOW_COUNT     3
#endif

// IEBus address
typedef int16_t IEBUS_ADDRESS;
#define INVALID_IEBUS_ADDRESS   (IEBUS_ADDRESS)-1

// IEBus control
enum {
    IEBUS_CONTROL_SLAVE_STATUS_READ             = 0x0,  // reads slave status (SSR)
    IEBUS_CONTROL_DATA_READ_AND_LOCK            = 0x3,  // reads and locks data
    IEBUS_CONTROL_LOCK_LSB_ADDRESS_READ         = 0x4,  // reads lock address (lower 8 bits)
    IEBUS_CONTROL_LOCK_MSB_ADDRESS_READ         = 0x5,  // reads lock address (higher 4 bits)
    IEBUS_CONTROL_SLAVE_STATUS_READ_AND_UNLOCK  = 0x6,  // reads and unlocks slave status (SSR)
    IEBUS_CONTROL_DATA_READ                     = 0x7,  // reads data
    IEBUS_CONTROL_COMMAND_WRITE_AND_LOCK        = 0xa,  // writes and locks command
    IEBUS_CONTROL_DATA_WRITE_AND_LOCK           = 0xb,  // writes and locks data
    IEBUS_CONTROL_COMMAND_WRITE                 = 0xe,  // writes command
    IEBUS_CONTROL_DATA_WRITE                    = 0xf   // writes data
}; typedef uint8_t IEBUS_CONTROL;

// IEBus frame

// frame fields
enum {
    IEBUS_FRAME_FIELD_BROADCAST_BIT,
    IEBUS_FRAME_FIELD_MASTER_ADDRESS,
    IEBUS_FRAME_FIELD_SLAVE_ADDRESS,
    IEBUS_FRAME_FIELD_CONTROL,
    IEBUS_FRAME_FIELD_MESSAGE_LENGTH,
    IEBUS_FRAME_FIELD_DATA
};

// maximum length of data field in bytes
#define IEBUS_FRAME_MESSAGE_LENGTH_FIELD_MAXIMUM_VALUE      256
#define IEBUS_FRAME_DATA_FIELD_MAXIMUM_LENGTH               32 // Mode 1

// maximum number of bits in a frame
#define IEBUS_FRAME_MAXIMUM_NUMBER_OF_BITS          (45 + (10 * IEBUS_FRAME_DATA_FIELD_MAXIMUM_LENGTH))

// number of bits (excluding parity and acknowledge) for each field
#define IEBUS_FRAME_FIELD_BITS_BROADCAST_BIT        1
#define IEBUS_FRAME_FIELD_BITS_MASTER_ADDRESS       12
#define IEBUS_FRAME_FIELD_BITS_SLAVE_ADDRESS        12
#define IEBUS_FRAME_FIELD_BITS_CONTROL              4
#define IEBUS_FRAME_FIELD_BITS_MESSAGE_LENGTH       8
#define IEBUS_FRAME_FIELD_BITS_DATA                 8

// frame field information
typedef struct {
    uint8_t id;
    bool hasParityBit;
    bool hasAcknowledgeBit;
    uint8_t dataBits;
    uint8_t totalBits;
    bool isTwoBytes;
    uint16_t bitMask;
} IEBUS_FRAME_FIELD;

#define IEBUS_FRAME_FIELD_INITIALIZER(id, hasParityBit, hasAcknowledgeBit, numberOfBits) \
    [id] = { \
            id, \
            hasParityBit, \
            hasAcknowledgeBit, \
            numberOfBits, \
            numberOfBits + hasParityBit + hasAcknowledgeBit, \
            (numberOfBits > 8), \
            0x1 << (numberOfBits - 1) \
    }

static const IEBUS_FRAME_FIELD iebusFrameField[] = {
        IEBUS_FRAME_FIELD_INITIALIZER(IEBUS_FRAME_FIELD_BROADCAST_BIT, false, false, IEBUS_FRAME_FIELD_BITS_BROADCAST_BIT),
        IEBUS_FRAME_FIELD_INITIALIZER(IEBUS_FRAME_FIELD_MASTER_ADDRESS, true, false, IEBUS_FRAME_FIELD_BITS_MASTER_ADDRESS),
        IEBUS_FRAME_FIELD_INITIALIZER(IEBUS_FRAME_FIELD_SLAVE_ADDRESS, true, true, IEBUS_FRAME_FIELD_BITS_SLAVE_ADDRESS),
        IEBUS_FRAME_FIELD_INITIALIZER(IEBUS_FRAME_FIELD_CONTROL, true, true, IEBUS_FRAME_FIELD_BITS_CONTROL),
        IEBUS_FRAME_FIELD_INITIALIZER(IEBUS_FRAME_FIELD_MESSAGE_LENGTH, true, true, IEBUS_FRAME_FIELD_BITS_MESSAGE_LENGTH),
        IEBUS_FRAME_FIELD_INITIALIZER(IEBUS_FRAME_FIELD_DATA, true, true, IEBUS_FRAME_FIELD_BITS_DATA)
};

// indicates that there's activity on the bus; will be used to decide when to write
//   to the bus to minimize bus collision
static volatile bool busIsBusy = true;

// status of the current transmission
static volatile bool writerIsTransmitting = false;

/**
 * IEBus reader - uses ADCs to sample the bus at uniform intervals
 */

// bits buffer
#define READER_BITS_BUFFER_SIZE     1024
static uint32_t readerBitsBuffer[READER_BITS_BUFFER_SIZE];
static volatile uint32_t readerBitsBufferEndIndex = READER_BITS_BUFFER_SIZE - 1;
static volatile uint32_t readerBitsBufferEndMask = 0;
static volatile uint32_t readerBitsBufferEndCount = 0;
static volatile uint32_t readerBitsBufferStartIndex = READER_BITS_BUFFER_SIZE - 1;
static volatile uint32_t readerBitsBufferStartShift = 32;
static volatile uint32_t readerBitsBufferStartCount = 0;
static volatile bool readerBitsBufferNotFull = true;
#define lockReaderBitsBuffer()      // TODO
#define unlockReaderBitsBuffer()    // TODO

enum {
    READER_IEBUS_BIT_LOGIC_LOW,
    READER_IEBUS_BIT_LOGIC_HIGH,
    READER_IEBUS_BIT_START
};

// frame buffer
typedef struct {
    uint8_t valid;
    uint8_t notBroadcast;
    uint8_t masterAddress[2];
    uint8_t slaveAddress[2];
    uint8_t control;
    uint8_t dataLength;
    uint8_t data[IEBUS_FRAME_DATA_FIELD_MAXIMUM_LENGTH];
} READER_FRAME_BUFFER;

enum {
    READER_FRAME_BUFFER_INDEX_BROADCAST_BIT,
    READER_FRAME_BUFFER_INDEX_MASTER_ADDRESS_MSB,
    READER_FRAME_BUFFER_INDEX_MASTER_ADDRESS_LSB,
    READER_FRAME_BUFFER_INDEX_SLAVE_ADDRESS_MSB,
    READER_FRAME_BUFFER_INDEX_SLAVE_ADDRESS_LSB,
    READER_FRAME_BUFFER_INDEX_CONTROL,
    READER_FRAME_BUFFER_INDEX_MESSAGE_LENGTH,
    READER_FRAME_BUFFER_INDEX_DATA
};

#define READER_FRAME_BUFFER_ITEMS   100
static READER_FRAME_BUFFER readerFrameBuffer[READER_FRAME_BUFFER_ITEMS];
static const READER_FRAME_BUFFER *lastReaderFrameBuffer = &readerFrameBuffer[READER_FRAME_BUFFER_ITEMS - 1];
static READER_FRAME_BUFFER *readerFrameBufferEnd = readerFrameBuffer;
static READER_FRAME_BUFFER *readerFrameBufferStart = readerFrameBuffer;

// ADC constants: resolution and reference voltage
#define READER_ADC_MAXIMUM_VALUE        __READER_ADC_MAXIMUM_VALUE
#define READER_ADC_REFERENCE_VOLTAGE    __READER_ADC_REFERENCE_VOLTAGE

// convert voltage (millivolts) to ADC value
#define READER_ADC_VALUE(voltage)       (int16_t)((voltage) * READER_ADC_MAXIMUM_VALUE / READER_ADC_REFERENCE_VOLTAGE + 0.5)

// the maximum difference between BUS+ and BUS- for a sample to be considered a logic high;
//   the IEBus specification sets this value to 20mV
#define READER_IEBUS_LOGIC_HIGH_MAXIMUM_DIFFERENCE      READER_ADC_VALUE(__READER_IEBUS_LOGIC_HIGH_MAXIMUM_DIFFERENCE)

// the minimum difference between BUS+ and BUS- for a sample to be considered a logic low;
//   the IEBus specification sets this value to 120mV; set to a lower value to compensate for error
#define READER_IEBUS_LOGIC_LOW_MINIMUM_DIFFERENCE       READER_ADC_VALUE(__READER_IEBUS_LOGIC_LOW_MINIMUM_DIFFERENCE)

// the minimum number of consecutive high samples for the bus to be considered idle
#define READER_IEBUS_IDLE_MINIMUM_HIGH_COUNT            __READER_IEBUS_IDLE_MINIMUM_HIGH_COUNT

// the minimum number of consecutive low samples for a bit to be considered a start bit
#define READER_IEBUS_START_BIT_MINIMUM_LOW_COUNT        __READER_IEBUS_START_BIT_MINIMUM_LOW_COUNT

// the minimum number of consecutive low samples for the samples to be considered a logic bit
#define READER_IEBUS_LOGIC_BIT_MINIMUM_LOW_COUNT        __READER_IEBUS_LOGIC_BIT_MINIMUM_LOW_COUNT

// the maximum number of consecutive low samples for a bit to be considered a logic high
#define READER_IEBUS_LOGIC_HIGH_MAXIMUM_LOW_COUNT       __READER_IEBUS_LOGIC_HIGH_MAXIMUM_LOW_COUNT
/*======================*/

#ifdef REVEAL_TRUE_ADC
static volatile float adcResult;
#endif
#if 0
static bool isHigh(uint16_t result)
{
    if(0x8000 & result)
    {
        return true;
    }
    return (result >> 2) < 198;//745; //0.3 * 8191 / 3.3
}
#else
//#define resultToAdcSigned(result) (int32_t)((0x8000 & result) ? (result >> 2) | 0xFFFFC000 : (result >> 2))
//#define isHigh(adcSigned) (adcSigned < 198)  //198 = 0.08 * 8191 / 3.3   // 297 = 0.12  * 8191 / 3.3

#ifdef REVEAL_TRUE_ADC
//![Conversion to Real Value]
/* Converts the ADC result (14-bit) to a float with respect to a 3.3v reference
 */
static float convertToFloat(uint16_t result)
{
    int32_t temp;

        if(0x8000 & result)
        {
            temp = (result >> 2) | 0xFFFFC000;
            return ((temp * 3.3f) / 8191);
        }
        else
            return ((result >> 2)*3.3f) / 8191;
}
//![Conversion to Real Value]
#endif
#endif
typedef struct {
    struct {
        unsigned int received;
        unsigned int processed;
        unsigned int valid;
        unsigned int notAcknowledged;
        unsigned int parityError;
        unsigned int incomplete;
        unsigned int tooLong;
        unsigned int idle;
    } readerFrames;
    struct {
        const unsigned int capacity;
        unsigned int currentUsage;
        unsigned int maximumUsage;
        unsigned int overrun;
    } readerBitsBuffer;
    struct {
        const unsigned int capacity;
        unsigned int currentUsage;
        unsigned int maximumUsage;
        unsigned int overrun;
    } readerFrameBuffer;
} VM_IEBUS_STATISTICS;

volatile VM_IEBUS_STATISTICS vmIebusStatistics = {
        { 0, 0, 0, 0, 0, 0, 0, 0}, // frames
        { sizeof(readerBitsBuffer), 0, 0, 0 }, // bits buffer
        { READER_FRAME_BUFFER_ITEMS, 0, 0, 0} // frame buffer
    };

volatile static uint32_t sampleCount = 0;
#define HIGH_LOW_LIST_SIZE 1024
typedef struct  {
    uint8_t high;
    //int32_t adcSigned;
    //uint16_t result;
    float fl;
}highLow_t;
volatile static highLow_t highLowList[HIGH_LOW_LIST_SIZE];
static int32_t highLowListIdx = 0;
static int32_t adcResultMax = 0x0;
static int32_t adcResultMin = 0xfff;

/* This interrupt happens every time a conversion has completed. Since the FPU
 * is enabled in stacking mode, we are able to use the FPU safely to perform
 * efficient floating point arithmetic.*/
void ADC14_IRQHandler(void)
{
    static bool notFrame = true;
    static bool busIsHigh = false;

    uint64_t status;

    status = MAP_ADC14_getEnabledInterruptStatus();
    MAP_ADC14_clearInterruptFlag(status);
#if 1
    if(status & ADC_INT0)
    {
        uint16_t result = MAP_ADC14_getResult(ADC_MEM0);
        float floatResult = convertToFloat(result);
        bool logicHigh = floatResult > 0.28f;
        if(floatResult > 0.28f)
        {
            highLowList[highLowListIdx].fl = floatResult;
            highLowList[highLowListIdx].high = logicHigh;
            if(++highLowListIdx >= HIGH_LOW_LIST_SIZE)
            {
                __no_operation();
                highLowListIdx = 0;
            }
        }

        if(logicHigh) // logic high
        {
            if(busIsHigh)
            {
                ++sampleCount;

                if(sampleCount == READER_IEBUS_IDLE_MINIMUM_HIGH_COUNT)
                {
                    ++vmIebusStatistics.readerFrames.idle;
                    busIsBusy = false;
                    notFrame = true;
                }
            }
            else
            {
                busIsHigh = true;

                while(readerBitsBufferNotFull)
                {
                    if(readerBitsBufferEndMask == 0)
                    {
                        if(++readerBitsBufferEndIndex == READER_BITS_BUFFER_SIZE)
                        {
                            readerBitsBufferEndIndex = 0;
                        }

                        if(readerBitsBufferEndIndex == readerBitsBufferStartIndex)
                        {
                            ++vmIebusStatistics.readerBitsBuffer.overrun;
                            readerBitsBufferNotFull = false;
                            notFrame = true;
                            break;
                        }

                        readerBitsBuffer[readerBitsBufferEndIndex] = 0;
                        readerBitsBufferEndMask = 0x1;
                    }

                    if(sampleCount > READER_IEBUS_START_BIT_MINIMUM_LOW_COUNT)
                    {
                        ++vmIebusStatistics.readerFrames.received;
                        readerBitsBuffer[readerBitsBufferEndIndex] |= (readerBitsBufferEndMask << 1);
                        notFrame = false;
                    }
                    else if(notFrame)
                    {
                        break;
                    }
                    else if(sampleCount < READER_IEBUS_LOGIC_HIGH_MAXIMUM_LOW_COUNT)
                    {
                        if(sampleCount < READER_IEBUS_LOGIC_BIT_MINIMUM_LOW_COUNT)
                        {
                            break;
                        }

                        readerBitsBuffer[readerBitsBufferEndIndex] |= readerBitsBufferEndMask;
                    }

                    readerBitsBufferEndMask <<= 2;
                    ++readerBitsBufferEndCount;
                    break;
                }

                sampleCount = 1;
            }
        }
        else
        {
            busIsBusy = true;

            if(busIsHigh)
            {
                sampleCount = 1;
                busIsHigh = false;
            }
            else
            {
                ++sampleCount;
            }
        }
    }
#else
    if(status & ADC_INT0)
    {
        uint16_t result = MAP_ADC14_getResult(ADC_MEM0);
        int32_t adcSigned = resultToAdcSigned(result);

        uint8_t logicHigh = isHigh(adcSigned);
        adcResultMax = (adcResultMax < adcSigned) ? adcSigned : adcResultMax;
        adcResultMin = (adcResultMin > adcSigned) ? adcSigned : adcResultMin;
        highLowList[highLowListIdx].high = logicHigh;
        highLowList[highLowListIdx].adcSigned = adcSigned;
        highLowList[highLowListIdx].result = result;
#ifdef REVEAL_TRUE_ADC
        float floatResult = convertToFloat(result);
        highLowList [highLowListIdx].fl = floatResult;
#define LOW_LIMIT 0.08f
#if 0
        if(floatResult < LOW_LIMIT != logicHigh)
        {
            //invalid!!
            __no_operation();
            logicHigh = floatResult < LOW_LIMIT;
            highLowList[highLowListIdx].high = logicHigh;
        }
#else
        logicHigh = floatResult < LOW_LIMIT;
        highLowList[highLowListIdx].high = logicHigh;
#endif
#endif
        if(++highLowListIdx >= HIGH_LOW_LIST_SIZE)
        {
            __no_operation();
            highLowListIdx = 0;
        }

        if(logicHigh)
        {
#ifdef REVEAL_TRUE_ADC
            //adcResult = convertToFloat(result);
            //__no_operation();
#endif

            if(busIsHigh)
            {
                ++sampleCount;

                if(sampleCount == READER_IEBUS_IDLE_MINIMUM_HIGH_COUNT)
                {
                    __no_operation();
                    busIsBusy = false;
                    notFrame = true;
                }
            }
            else
            {
                busIsHigh = true;

                while(readerBitsBufferNotFull)
                {
                    if(readerBitsBufferEndMask == 0)
                    {
                        if(++readerBitsBufferEndIndex == READER_BITS_BUFFER_SIZE)
                        {
                            readerBitsBufferEndIndex = 0;
                        }

                        if(readerBitsBufferEndIndex == readerBitsBufferStartIndex)
                        {
                            readerBitsBufferNotFull = false;
                            notFrame = true;
                            break;
                        }

                        readerBitsBuffer[readerBitsBufferEndIndex] = 0;
                        readerBitsBufferEndMask = 0x1;
                    }

                    if(sampleCount > READER_IEBUS_START_BIT_MINIMUM_LOW_COUNT)
                    {
                        readerBitsBuffer[readerBitsBufferEndIndex] |= (readerBitsBufferEndMask << 1);
                        notFrame = false;
                    }
                    else if(notFrame)
                    {
                        break;
                    }
                    else if(sampleCount < READER_IEBUS_LOGIC_HIGH_MAXIMUM_LOW_COUNT)
                    {
                        if(sampleCount < READER_IEBUS_LOGIC_BIT_MINIMUM_LOW_COUNT)
                        {
                            break;
                        }

                        readerBitsBuffer[readerBitsBufferEndIndex] |= readerBitsBufferEndMask;
                    }

                    readerBitsBufferEndMask <<= 2;
                    ++readerBitsBufferEndCount;
                    //__no_operation();
                    break;
                }
                __no_operation();
                sampleCount = 1;
            }
        } else // bus is not high
        {
            busIsBusy = true;

            if(busIsHigh)
            {
                sampleCount = 1;
                __no_operation();
                busIsHigh = false;
            }
            else
            {
                __no_operation();
                ++sampleCount;
            }
        }
    }
#endif
    //MAP_Interrupt_disableSleepOnIsrExit();
    //Interrupt_disableSleepOnIsrExit();
}

enum {
    incomplete,
    notAcknowledged,
    parityError,
    valid,
    tooLong
};

static void serviceBus(bool loop)
{
#if 0
#define finishFrame(reason) \
        { \
            ++vmIebusStatistics.readerFrames.reason; \
            ++vmIebusStatistics.readerFrames.processed; \
            process = false; \
        }
#else
    static uint32_t led_set = 0;
    //__no_operation();
#if 0
#define finishFrame(reason) \
{ \
    ++vmIebusStatistics.readerFrames.reason; \
    ++vmIebusStatistics.readerFrames.processed; \
    switch(reason){ \
    case valid: MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN0); break; \
    case incomplete: MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN2); break; \
    case notAcknowledged: MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN1); break; \
    case parityError: MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN0); break; \
    case tooLong: MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN2); break; \
    }\
    led_set = 20; \
    process = false; \
}
#else
#define finishFrame(reason)  \
{ \
    ++vmIebusStatistics.readerFrames.reason; \
    ++vmIebusStatistics.readerFrames.processed; \
    if(reason == valid) {MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P1, GPIO_PIN0);}\
    else {MAP_GPIO_setOutputHighOnPin(GPIO_PORT_P2, GPIO_PIN2);}\
    led_set = 20; \
    process = false; \
}
#endif
#endif

#define resetLeds() { \
    if(!led_set) { \
        MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P1, GPIO_PIN0);\
        MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN0);\
        MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN1);\
        MAP_GPIO_setOutputLowOnPin(GPIO_PORT_P2, GPIO_PIN2);\
    } else {led_set--;}\
}

    resetLeds();

    static bool process = false; // indicates that a frame is being processed
    static bool checkAcknowledgement; // ordinary (not broadcast) communication should be acknowledged
    static uint8_t remainingFieldBits; // remaining bits for the current field
    static const IEBUS_FRAME_FIELD *currentField; // current field information
    static uint16_t fieldBitMask; // data bit mask for the current field
    static uint16_t fieldDataBuffer; // data buffer for the current field
    static uint16_t frameHighBitCount; // frame high bit count (excluding the broadcast and acknowledge bits) for parity check
    static int remainingBytes; // number of remaining bytes to receive for the current frame
    static uint8_t *data;

    do
    {
#if 0
        if(vmIebusIsActive && clock_getInterval(lastBusActivityTime, clock_get(), CLOCK_UNIT_SECOND) > 60)
        {
            vmIebusIsActive = false;
        }
#endif
        if(readerBitsBufferNotFull)
        {
            if(readerBitsBufferEndCount != readerBitsBufferStartCount)
            {
                if(readerBitsBufferStartShift < 30)
                {
                    readerBitsBufferStartShift += 2;
                }
                else
                {
                    readerBitsBufferStartShift = 0;

                    if(++readerBitsBufferStartIndex == READER_BITS_BUFFER_SIZE)
                    {
                        readerBitsBufferStartIndex = 0;
                    }
                }

                unsigned int endIndex = readerBitsBufferEndIndex;
#if 1
                if(endIndex < readerBitsBufferStartIndex)
                {
                    vmIebusStatistics.readerBitsBuffer.currentUsage = (READER_BITS_BUFFER_SIZE - readerBitsBufferStartIndex + endIndex + 1) * sizeof(unsigned int);
                }
                else
                {
                    vmIebusStatistics.readerBitsBuffer.currentUsage = (endIndex - readerBitsBufferStartIndex + 1) * sizeof(unsigned int);
                }

                if(vmIebusStatistics.readerBitsBuffer.currentUsage > vmIebusStatistics.readerBitsBuffer.maximumUsage)
                {
                    vmIebusStatistics.readerBitsBuffer.maximumUsage = vmIebusStatistics.readerBitsBuffer.currentUsage;
                }
#endif
                int bit = (readerBitsBuffer[readerBitsBufferStartIndex] >> readerBitsBufferStartShift) & 0x3;
                bitList[bitListIdx++] = bit;
                if(bitListIdx >= BIT_LIST_SIZE)
                {
                    __no_operation();
                    bitListIdx = 0;
                }

                __no_operation();

                ++readerBitsBufferStartCount;

                if(bit == READER_IEBUS_BIT_START)
                {
                    // the previous frame didn't finish
                    if(process)
                    {
                        //__no_operation();
                        finishFrame(incomplete);
                    }

                    // prepare buffer
                    data = &readerFrameBufferEnd->notBroadcast;

                    // after the start bit is the broadcast bit
                    currentField = iebusFrameField;
                    remainingFieldBits = currentField->totalBits;
                    fieldBitMask = currentField->bitMask;
                    fieldDataBuffer = 0;

                    // process the frame
                    process = true;
                }
                else if(process)
                {
                    // save the bit
                    if(bit == READER_IEBUS_BIT_LOGIC_HIGH)
                    {
                        ++frameHighBitCount; // track the number of high bits for parity checking later
                        fieldDataBuffer |= fieldBitMask;
                    }

                    fieldBitMask >>= 1;

                    // last bit of field
                    if(--remainingFieldBits == 0)
                    {
                        if(currentField->hasAcknowledgeBit)
                        {
                            // ignore frames that are not acknowledged by slave during ordinary communication
                            if(bit == READER_IEBUS_BIT_LOGIC_HIGH)
                            {
                                --frameHighBitCount;

                                if(checkAcknowledgement)
                                {
                                    //__no_operation();
                                    finishFrame(notAcknowledged);
                                    continue;
                                }
                            }
                        }

                        if(currentField->hasParityBit)
                        {
                            // data is invalid if high bit count is odd
                            if((frameHighBitCount & 0x1))
                            {
                                //__no_operation();
                                finishFrame(parityError);
                                continue;
                            }
                        }

                        // save the field (up to two bytes)
                        if(currentField->isTwoBytes)
                        {
                            *data = fieldDataBuffer >> 8; // MSB
                            ++data;
                        }

                        *data = fieldDataBuffer; // LSB
                        ++data;

                        if(currentField->id == IEBUS_FRAME_FIELD_DATA)
                        {
                            // last byte of frame
                            if(--remainingBytes == 0)
                            {
                                //vmIebusIsActive = true;
                                //lastBusActivityTime = clock_get();

                                //MUTEX LOCK

                                readerFrameBufferEnd->valid = true;
                                readerFrameBufferEnd = (readerFrameBufferEnd == lastReaderFrameBuffer) ? readerFrameBuffer : (readerFrameBufferEnd + 1);

                                if(readerFrameBufferEnd == readerFrameBufferStart)
                                {
                                    ++vmIebusStatistics.readerFrameBuffer.overrun;
                                    readerFrameBufferStart->valid = false;
                                    readerFrameBufferStart = (readerFrameBufferStart == lastReaderFrameBuffer) ? readerFrameBuffer : (readerFrameBufferStart + 1);
                                }
                                else
                                {
                                    ++vmIebusStatistics.readerFrameBuffer.currentUsage;

                                    if(vmIebusStatistics.readerFrameBuffer.currentUsage > vmIebusStatistics.readerFrameBuffer.maximumUsage)
                                    {
                                        vmIebusStatistics.readerFrameBuffer.maximumUsage = vmIebusStatistics.readerFrameBuffer.currentUsage;
                                    }
                                }

                                //MUTEX UNLOCK

                                //__no_operation();
                                finishFrame(valid);
                                continue;
                            }
                        }
                        else
                        {
                            if(currentField->id == IEBUS_FRAME_FIELD_BROADCAST_BIT)
                            {
                                checkAcknowledgement = (bit == READER_IEBUS_BIT_LOGIC_HIGH);
                                frameHighBitCount = 0;
                            }
                            else if(currentField->id == IEBUS_FRAME_FIELD_MESSAGE_LENGTH)
                            {
                                // 0 means maximum
                                if(fieldDataBuffer == 0)
                                {
                                    fieldDataBuffer = IEBUS_FRAME_MESSAGE_LENGTH_FIELD_MAXIMUM_VALUE;
                                }

                                // ignore multi-frame data for now; should be okay since all relevant data are single frame
                                if(fieldDataBuffer > IEBUS_FRAME_DATA_FIELD_MAXIMUM_LENGTH)
                                {
                                    finishFrame(tooLong);
                                    continue;
                                }

                                remainingBytes = fieldDataBuffer;
                            }

                            ++currentField;
                        }

                        // prepare to process the next field
                        remainingFieldBits = currentField->totalBits;
                        fieldBitMask = currentField->bitMask;
                        fieldDataBuffer = 0;
                    }
                }
            }
        }
        else
        {
            readerBitsBufferEndIndex = READER_BITS_BUFFER_SIZE - 1;
            readerBitsBufferEndMask = 0;
            readerBitsBufferEndCount = 0;
            readerBitsBufferStartIndex = READER_BITS_BUFFER_SIZE - 1;
            readerBitsBufferStartShift = 32;
            readerBitsBufferStartCount = 0;
            readerBitsBufferNotFull = true;
        }
    }
    while(loop);

}
