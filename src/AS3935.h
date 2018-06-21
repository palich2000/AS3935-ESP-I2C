#ifndef AS3935_h
#define AS3935_h
#include "Arduino.h"

static const uint8_t AS3935_INT_STRIKE = 0x08;
static const uint8_t AS3935_INT_DISTURBER = 0x04;
static const uint8_t AS3935_INT_NOISE = 0x01;
static const int8_t  AS3935_DISTANCE_OUT_OF_RANGE = -2;
static const uint8_t AS3935_AFE_INDOOR =  0b10010;
static const uint8_t AS3935_AFE_OUTDOOR = 0b01110;

#define  AS3935_LCO_DIV_16   	0
#define  AS3935_LCO_DIV_32   	1
#define  AS3935_LCO_DIV_64   	2
#define  AS3935_LCO_DIV_128  	3

#define AS3935_AFE_GB		0x00, 0b00111110
#define AS3935_PWD		0x00, 0b00000001
#define AS3935_NF_LEV		0x01, 0b01110000
#define AS3935_WDTH		0x01, 0b00001111
#define AS3935_CL_STAT		0x02, 0b01000000
#define AS3935_MIN_NUM_LIGH	0x02, 0b00110000
#define AS3935_SREJ		0x02, 0b00001111
#define AS3935_LCO_FDIV		0x03, 0b11000000
#define AS3935_MASK_DIST	0x03, 0b00100000
#define AS3935_INT		0x03, 0b00001111
#define AS3935_DISTANCE		0x07, 0b00111111
#define AS3935_DISP_LCO		0x08, 0b10000000
#define AS3935_DISP_SRCO	0x08, 0b01000000
#define AS3935_DISP_TRCO	0x08, 0b00100000
#define AS3935_TUN_CAP		0x08, 0b00001111
#define AS3935_ENERGY_1		0x04, 0b11111111
#define AS3935_ENERGY_2		0x05, 0b11111111
#define AS3935_ENERGY_3		0x06, 0b00011111

class AS3935
{
public:
    AS3935(uint8_t address, uint8_t interruptPin);
    ~AS3935(void);
    void begin(void);
    void begin(int sda, int scl);
    void interruptEnable(bool enable);
    uint8_t readRegister(uint8_t reg);
    uint8_t readRegisterWithMask(uint8_t reg, uint8_t mask);
    void writeRegisterWithMask(uint8_t reg, uint8_t mask, uint8_t value);
    void writeRegister(uint8_t reg, uint8_t value);
    void setDefault(void);
    void calibrateRCO(void);
    void disableOscillators(void);
    uint8_t getIntrruptReason(void);
    int8_t getDistance(void);
    bool isIndoor(void);
    bool setIndoor(void);
    bool setIndoor(bool enable);
    bool isOutdoor(void);
    bool setOutdoor(void);
    bool setOutdoor(bool enable);
    uint8_t getMinimumLightning(void);
    bool setMinimumLightning(uint8_t);
    void clearStats(void);

    uint8_t getNoiseFloor(void);
    bool setNoiseFloor(int level);
    uint8_t increaseNoiseFloor(void);
    uint8_t descreseNoiseFloor(void);

    uint8_t setTuningCapacitor(uint8_t);
    bool    calibrate(uint8_t &tuneCapacitor);
    int     tuneAntenna(uint8_t tuneCapacitor);
    int     getBestTune(long int &freq);

    bool waitingInterrupt();
    uint8_t getInterrupt(void);
    int  getInterruptCount();
    void emulateInterrupt(void);
    uint32_t getStrikeEnergyRaw(void);

    uint8_t getWatchdogThreshold(void);
    bool    setWatchdogThreshold(int level);
    uint8_t getSpikeRejection(void);
    bool    setSpikeRejection(int level);

    uint8_t getMaskDisturber(void);
    bool    setMaskDisturber(bool enable);
    uint8_t bitTest(void);
    int     irqTest(void);

    bool    setFreqDivRatio(uint8_t div_ratio);
    uint8_t getFreqDivRatio(void);

    static volatile int _interruptWaiting;
    static unsigned long last_interrupt_time;
    static uint8_t getShift(uint8_t mask);
    unsigned long timeFromLastInterrupt(void);
    void powerDown();
    void powerUp();
    uint8_t transmitError();

    bool displayLCOFreqOnIrq(uint8_t on);

private:
    uint8_t _address;
    uint8_t _interruptPin;
    uint8_t transmit_error;
    const uint8_t _defaultSDA = SDA; // D4
    const uint8_t _defaultSCL = SCL; // D5
    static uint8_t _getShift(uint8_t mask);
};

typedef struct {
  //LSB
  uint8_t PWD:1;
  uint8_t AFE_GB:5;
  uint8_t reserved:2;
} REG00_t;

typedef struct {
  uint8_t WDTH:4;
  uint8_t NF_LEV:3;
  uint8_t reserved:1;
} REG01_t;

typedef struct {
  uint8_t SREJ:4;
  uint8_t MIN_NUM_LIGHT:2;
  uint8_t CL_STAT:1;
  uint8_t reserved:1;
} REG02_t;

typedef struct {
  uint8_t INT:4;
  uint8_t reserved:1;
  uint8_t MASK_DIST:1;
  uint8_t LCO_FDIV:2;
} REG03_t;

typedef struct {
  uint8_t S_LIG_L:8;
} REG04_t;

typedef struct {
  uint8_t S_LIG_M:8;
} REG05_t;

typedef struct {
  uint8_t S_LIG_MM:5;
  uint8_t reserved:3;
} REG06_t;

typedef struct {
  uint8_t DISTANCE:6;
  uint8_t reserved:2;
} REG07_t;

typedef struct {
  // LSB
  uint8_t TUN_CAP:4;
  uint8_t reserved:1;
  uint8_t DISP_TRCO:1;
  uint8_t DISP_SRCO:1;
  uint8_t DISP_LCO:1;
} REG08_t;

typedef union {
  REG00_t R0;
  REG01_t R1;
  REG02_t R2;
  REG03_t R3;
  REG04_t R4;
  REG05_t R5;
  REG06_t R6;
  REG07_t R7;
  REG08_t R8;
  uint8_t data;
} REG_u;

#endif
