    /*
    * PXT_M_and_M extention
    *
    * This is a MakeCode (pxt) extension for the M&M colour sorting machine or Project Recycle.
    * The extension includes blocks for the colour sensor type TCS34725 and the servo driver type PCA9685 connected to a micro:bit via the i2c bus. 
    * The TCS34725 sensor is assumed to be an Adafruit TCS34725 colour sensor board with inbuilt illumination LED. I2C bus address = 0x29.
    * The PCA9685 driver is assummed to be a Kitronix PCA9685 16 channel servo motor driver board. I2C bus address = 0x6A. 
    *
    * Five blocks deal with the TCS32725: 
    * get red light component, get green light component, get blue light component, get total light intensity and get the colour of a M & M confectionery (0-6).
    * The colour component readings are normalised against the total light reading.
    * Interrupts are disabled in the sensor and no provision is made to control the inbuilt white illumination LED. The illumination LED is permanentlky on.
    * The M & M colour block returns a number between 0 and 6. Encoding is shown below.
    * Refer to Adafruit docs and tutorial for more information.
    * 
    * Two blocks deal with the PCA9685:
    * setRange - Set the specified Servo motor output channel to a specified pulse range and servoWrite set the specified servo to the specified angle (0 - 180 degrees).
    * A private initialisation function is provided to initialise the PCA9685 chip, it is called by the first use of the servoWrite function.
    * The initialisation function sets all servo channels to the same default pulse range, currently R700_2400uS. Any call to the setRange function should be done after
    * a call to servoWrite, otherwise the results of the setRange call will be overwritten by the first use of servoWrite. Subsequent calls to servoWrite will not effect
    * any data setup by setRange.
    *
    * The RC servo motor industry default pulse width of 0.5mS (0deg) to 2.5mS (180deg) does not always function correctly. Any excursions outside this default range will
    * result in damage to the servo motor. However, some cheaper servo motors struggle to deal with this standard industry range and begin growl, buzz, draw lots of current
    * and overheat that results in the ultimate failure of the servo motor. This problem mainly occurs at the 0.5mS end of the range. Some servo motors will not extend to 180deg
    * at 2.5mS and require a maximum pulse width of 2.7mS to reach 180deg. Caution should be exercised if extending the pulse range beyond 2.5mS.
    * The setRange block will allow each of the 16 servo outputs of the PCA9685 to be individually configured to one of the following six pulse ranges:
    * 1mS - 2mS, 0.9mS - 2.1mS, 0.8mS - 2.2mS, 0.7mS - 2.3mS, 0.6mS - 2.4mS and 0.5mS - 2.5mS to help eliminate growling, buzzing and overheating.
    * The PWM frequency is set to 50Hz making each bit of the PCA9685 4096 count equal to 4.88uS
    * 
    */
    
//% color="#AA278D" icon="\uf06e"
namespace M_and_M {
    /*
    * TCS34725: Color sensor register address and control bit definitions 
    */
    const TCS34725_ADDRESS: number = 0x29;          // I2C bus address of TCS34725 sensor (0x39 for TCS34721)
    const REG_TCS34725_COMMAND_BIT: number = 0x80;  // Command register access bit
    const REG_TCS34725_ENABLE: number = 0X00;       // Enable register address
    const REG_TCS34725_TIMING: number = 0X01;       // RGBC timing register address
    const REG_TCS34725_WAIT: number = 0x03;         // Wait time register address
    const REG_TCS34725_CONFIG: number = 0x0D;       // Configuration register address
    const REG_TCS34725_CONTROL: number = 0x0F;      // Control register address, sets gain
    const REG_TCS34725_ID: number = 0x12;           // ID register address, should contain 0x44 for TCS34725 or 0x4D for TCS34725
    const REG_TCS34725_STATUS: number = 0x13;       // Status register address
    const REG_CLEAR_CHANNEL_L: number = 0X14;       // Clear data low byte register address
    const REG_RED_CHANNEL_L: number = 0X16;         // Red data low byte register address
    const REG_GREEN_CHANNEL_L: number = 0X18;       // Green data low byte register address
    const REG_BLUE_CHANNEL_L: number = 0X1A;        // Blue data low byte register address
    const TCS34725_AIEN: number = 0X10;             // Enable register RGBC interrupt enable bit, 0 = IRQ not enabled, 1 = IRQ enabled
    const TCS34725_PON: number = 0X01;              // Enable register PON bit, 0 = power off, 1 = power on
    const TCS34725_AEN: number = 0X02;              // Enable register RGBC enable bit, 0 = disable AtoD conversion, 1 = enable AtoD conversion
    const TCS34725_ID: number = 0x44;               // Sensor ID = 0x44 or 68 decimal
    const TCS34729_ID: number = 0x4D;               // Sensor ID = 0x4D or 77 decimal

    /*
    * TSC34725: M and M colour encoding
    */
    const BLANK: number = 0;                        // Broken, missing, discoloured or chipped M & M
    const BROWN: number = 1;
    const RED: number = 2;
    const ORANGE: number = 3;
    const YELLOW: number = 4;
    const GREEN: number = 5;
    const BLUE: number = 6;
    const UNKNOWN: number = 9;                      // Not used, kept for testing purposes

    /*
    * TCS34725: Colour sensor data storage and flag definitions
    */
    let RGBC_C: number = 0;                         // Clear light raw data storage
    let RGBC_R: number = 0;                         // Red light raw data storage
    let RGBC_G: number = 0;                         // Green light raw data storage
    let RGBC_B: number = 0;                         // Blue light raw data storage
    let TCS34725_INIT: number = 0;                  // TSC34725 sensor initialisation flag, 0 = not initialised, 1 = initialised

    /*
    * TCS34725: I2C bus functions: Requires i2c.ts
    */
    function getInt8LE(addr: number, reg: number): number {     // Get 8 bit little-endian integer 
        pins.i2cWriteNumber(addr, reg, NumberFormat.UInt8BE);
        return pins.i2cReadNumber(addr, NumberFormat.Int8LE);
    }

    function getUInt16LE(addr: number, reg: number): number {   // Get 16 bit little-endian unsigned integer
        pins.i2cWriteNumber(addr, reg, NumberFormat.UInt8LE);
        return pins.i2cReadNumber(addr, NumberFormat.UInt16LE);
    }

    function getInt16LE(addr: number, reg: number): number {    // Get 16 bit little-endian integer
        pins.i2cWriteNumber(addr, reg, NumberFormat.UInt8LE);
        return pins.i2cReadNumber(addr, NumberFormat.Int16LE);
    }

    function readReg(addr: number, reg: number): number {       // Read 8 bit big-endian unsigned integer
        pins.i2cWriteNumber(addr, reg, NumberFormat.UInt8LE);
        return pins.i2cReadNumber(addr, NumberFormat.UInt8LE);
    }

    function writeReg(addr: number, reg: number, dat: number) { // Write 8 bit little-endian integer
        let buf = pins.createBuffer(2);
        buf[0] = reg;
        buf[1] = dat;
        pins.i2cWriteBuffer(addr, buf);
    }

    /**
     * TCS34725: Color Sensor Initialisation
     */
    function tcs34725_begin() {
        let id = readReg(TCS34725_ADDRESS, REG_TCS34725_ID | REG_TCS34725_COMMAND_BIT);                 // Get TCS34725 ID
        if (id === 0x44) {                                                                              // Valid ID?
            writeReg(TCS34725_ADDRESS, REG_TCS34725_TIMING | REG_TCS34725_COMMAND_BIT, 0xEB);           // Yes, Set integration time
            writeReg(TCS34725_ADDRESS, REG_TCS34725_WAIT | REG_TCS34725_COMMAND_BIT, 0xFF);             // Set wait time to 2.4mS 
            writeReg(TCS34725_ADDRESS, REG_TCS34725_CONFIG | REG_TCS34725_COMMAND_BIT, 0x00);           // Set WLONG to 0
            writeReg(TCS34725_ADDRESS, REG_TCS34725_CONTROL | REG_TCS34725_COMMAND_BIT, 0x01);          // Set gain to 4
            writeReg(TCS34725_ADDRESS, REG_TCS34725_ENABLE | REG_TCS34725_COMMAND_BIT, TCS34725_PON);   // Power on sensor, disable wait time, disable interrupts 
            basic.pause(3);                                                                             // Need minimum 2.4mS after power on
            writeReg(TCS34725_ADDRESS, REG_TCS34725_ENABLE | REG_TCS34725_COMMAND_BIT, TCS34725_PON | TCS34725_AEN);    // Keep power on, enable RGBC ADC
            TCS34725_INIT = 1;                                                                          // Sensor is connected and initialised
        }
        else {                                                                                          // No
            TCS34725_INIT = 0;                                                                          // Sensor is not connected
        }
    }

    /**
     * TCS34725: Color Sensor, read red, green, blue and clear raw data
     */
    function getRGBC() {
        if (!TCS34725_INIT) {                                                                       // Is the TCS32725 sensor initialised?
             tcs34725_begin();                                                                      // No, then initialise the sensor
        }
        let clear = getUInt16LE(TCS34725_ADDRESS, REG_CLEAR_CHANNEL_L | REG_TCS34725_COMMAND_BIT);  // Read natural (clear) light level
        if (clear == 0) {                                                                           // Prevent divide by zero error if sensor in complete darkness 
            clear = 1; 
        }
        RGBC_C = clear;
        RGBC_R = getUInt16LE(TCS34725_ADDRESS, REG_RED_CHANNEL_L | REG_TCS34725_COMMAND_BIT);      // Read red component of clear light
        RGBC_G = getUInt16LE(TCS34725_ADDRESS, REG_GREEN_CHANNEL_L | REG_TCS34725_COMMAND_BIT);    // Read green component of clear light
        RGBC_B = getUInt16LE(TCS34725_ADDRESS, REG_BLUE_CHANNEL_L | REG_TCS34725_COMMAND_BIT);     // Read blue component of clear light

        basic.pause(50);
        let ret = readReg(TCS34725_ADDRESS, REG_TCS34725_ENABLE | REG_TCS34725_COMMAND_BIT)        // Get current status of enable register
        ret |= TCS34725_AIEN;                                                                      // Set AEIN bit ?
        writeReg(TCS34725_ADDRESS, REG_TCS34725_ENABLE | REG_TCS34725_COMMAND_BIT, ret)            // Re-enable RGBC interrupt ?

    }

    /**
     * TCS34725: getRed - Reporter block that returns the normalised red value from the TCS34725 color sensor
     */
    //% block="red"
    //% weight=60 
    export function getRed(): number {
        getRGBC();                                                      // Get raw light and colour values
        let red = Math.round((RGBC_R / RGBC_C) * 255);                  // Normalise red value
        return red;
    }

    /**
     * TCS34725: getGreen - Reporter block that returns the normalised green value from the TCS34725 color sensor
     */
    //% block="green"
    //% weight=60 
    export function getGreen(): number {
        getRGBC();                                                      // Get raw light and colour values
        let green = Math.round((RGBC_G / RGBC_C) * 255);                // Normalise green value
        return green;
    }

    /**
     * TCS34725: getBlue - Reporter block that returns the normalised blue value from the TCS34725 color sensor
     */
    //% block="blue"
    //% weight=60 
    export function getBlue(): number {
        getRGBC();                                                      // Get raw light and colour values
        let blue = Math.round((RGBC_B / RGBC_C) * 255);                 // Normalise blue value
        return blue;
    }

    /**
     *  TCS34725: getAllLight - Reporter block that returns the raw or total light value from the TCS34725 color sensor
     */
    //% block="raw"
    //% weight=60 
    export function getRaw(): number {
        getRGBC();                                                      // Get raw light and colour values
        return Math.round(RGBC_C);                                      // Return clear natural light level
    }

    /**
     * TCS34725: m_mColour - Reporter block that returns the colour of the M and M
     */
    //% block="m & m colour"
    //% weight=60
    export function m_mColour(): number {
        getRGBC();                                                          // Get colour / light information from TSC34725 sensor
        let red: number = Math.round((RGBC_R / RGBC_C) * 255);              // Normalise red value
        let green: number = Math.round((RGBC_G / RGBC_C) * 255);            // Normalise green value
        let blue: number = Math.round((RGBC_B / RGBC_C) * 255);             // Normalise blue value
        let clear: number = RGBC_C;                                         // Get clear light level
        //basic.showNumber(clear);
        let colour: number = UNKNOWN;                                               // Start with unknown colour
        if (clear < 580 && clear > 540 && red > 80 && green < 100 && blue < 85) {   // Brown M & M?
            colour = BROWN;                                                         // Yes
        }
        else if (clear < 700 && red > 100 && green < 85 && blue < 70) {     // Red M & M?
            colour = RED;                                                   // Yes
        }
        else if (clear > 820 && red > 120 && green < 80 && blue < 60) {     // Orange M & M?
            colour = ORANGE;                                                // Yes
        }
        else if (clear > 1100 && red > 110 && green > 80 && blue < 55) {    // Yellow M & M?
            colour = YELLOW;                                                // Yes
        }
        else if (clear > 700 && red < 80 && green > 100 && blue < 80) {     // Green M & M?
            colour = GREEN;                                                 // Yes
        }
        else if (clear > 630 && red < 80 && green < 100 && blue > 85) {     // Blue M & M?
            colour = BLUE;                                                  // Yes
        }
        else {
            colour = BLANK;                                                 // Broken, missing, discoloured or chipped M & M
        }
        return colour; 
    }
    
    /*
    * PCA9685 register address and control bit definitions 
    */
    const CHIP_ADDRESS: number = 0x6A;              // Default Chip address
    const REG_MODE1: number = 0x00;                 // Mode 1 register address 
    const REG_MODE2: number = 0x01;                 // Mode 2 register address 
    const REG_SUB_ADR1: number = 0x02;              // Sub address register 1 address
    const REG_SUB_ADR2: number = 0x03;              // Sub address register 2 address
    const REG_SUB_ADR3: number = 0x04;              // Sub address register 3 address
    const REG_ALL_CALL: number = 0x05;              // All call address register
    const REG_SERVO1_BASE: number = 0x06;           // Servo 1 base address 
    const REG_SERVO_DISTANCE: number = 4;           // Four registers per servo 
    const REG_ALL_LED_ON_L: number = 0xFA;          // All LED on low register address
    const REG_ALL_LED_ON_H: number = 0xFB;          // All LED on high register address
    const REG_ALL_LED_OFF_L: number = 0xFC;         // All LED off low register address
    const REG_ALL_LED_OFF_H: number = 0xFD;         // All LED off high register address 
    const REG_PRE_SCALE: number = 0xFE;             // Pre-scaler register address

    const PWM_FREQUENCY: number = 0x79;             // Pre-scaler value for 50Hz

    let PCA9685_init: boolean = false;              // Flag to allow us to initialise without explicitly calling the initialisation function 

    // List of possible 16 servo motors 
    export enum Servos {
        Servo1 = 1,
        Servo2 = 2,
        Servo3 = 3,
        Servo4 = 4,
        Servo5 = 5,
        Servo6 = 6,
        Servo7 = 7,
        Servo8 = 8,
        Servo9 = 9,
        Servo10 = 10,
        Servo11 = 11,
        Servo12 = 12,
        Servo13 = 13,
        Servo14 = 14,
        Servo15 = 15,
        Servo16 = 16,
    }

    // 
	export enum BoardAddresses{
		Board1 = 0x6A,
	}

    // List of possible output pulse ranges
    export enum PulseRange {
        R500_2500uS = 1,
        R700_2400uS = 2,    // Currently set as default 
        R700_2300uS = 3,
        R800_2200uS = 4,
        R900_2100uS = 5,
        R1000_2000uS = 6,
    }

    // Time             0.5  0.7  0.7  0.8  0.9  1.0 mS
    const loPulseLim = [102, 143, 143, 164, 184, 204];  // Lower pulse limit width in multiples of 4.88uS

    // Time             2.5  2.4  2.3  2.2  2.1  2.0 mS 
    const hiPulseLim = [512, 500, 471, 451, 430, 409];  // Higher pulse limit width in multiples of 4.88uS

    // Time             2.0  1.8  1.6  1.4  1.2  1.0 mS 
    const range =      [410, 357, 328, 287, 246, 205];  // Pulse width range in multiples of 4.88uS

    // Servo number   1  2  3  4  5  6  7  8  9  10 11 12 13 14 15 16
    let ServoRange = [2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2]; // Individual servo pulse range, default = R700 - 2400uS 

    // Function to read i2c register - for testing purposes
    //function readReg(addr: number, reg: number): number {       // Read 8 bit little-endian unsigned integer
    //    pins.i2cWriteNumber(addr, reg, NumberFormat.UInt8LE);
    //    return pins.i2cReadNumber(addr, NumberFormat.UInt8LE);
    //}

    // Function to map a value from one range into another range 
    function map(value: number, fromLow: number, fromHigh: number, toLow: number, toHigh: number): number {
        return ((value - fromLow) * (toHigh - toLow)) / (fromHigh - fromLow) + toLow;
    }

	/*
	* This initialisation function sets up the PCA9865 servo driver chip. 
    * The PCA9685 comes out of reset in low power mode with the internal oscillator off with no output signals, this allows writes to the pre-scaler register.
    * The pre-scaler register is set to 50Hz producing a refresh rate or frame period of 20mS which inturn makes each bit of the 4096 count equal to 4.88uS.
    * Sets the 16 LED ON registers to 0x00 which starts the high output pulse start at the beginning of each 20mS frame period.
    * Sets the 16 LED OFF registers to 0x133 (4.88uS x 1500) which ends the high output pulse 1.5mS into the frame period. This places all servo motors at 90 degrees or centre travel.
    * It is these LED OFF registers that will be modified to set the pulse high end time to vary the pulse width and the position of the attached servo motor. 
    * Sets the mode1 register to 0x01 to disable restart, use internal clock, disable register auto increment, select normal (run) mode, disable sub addresses and allow LED all call addresses.
    * Finally the initialised flag will be set true.
	* This function should not be called directly by a user, the first servo write will call it.
    * This function initialises all 16 LED ON and LED OFF registers by using a single block write to the 'all LED' addresses.
	*/
	function init(): void {
        let buf = pins.createBuffer(2)                      // Create a buffer for i2c bus data
        buf[0] = REG_PRE_SCALE;                             // Point at pre-scaler register
        buf[1] = PWM_FREQUENCY;                             // Set PWM frequency to 50Hz or repetition rate of 20mS
        pins.i2cWriteBuffer(CHIP_ADDRESS, buf, false);      // Write to PCA9685 
        buf[0] = REG_ALL_LED_ON_L;                          // Point at ALL LED ON low byte register 
        buf[1] = 0x00;                                      // Start high pulse at 0 (0-0x199) 
        pins.i2cWriteBuffer(CHIP_ADDRESS, buf, false);      // Write to PCA9685
        buf[0] = REG_ALL_LED_ON_H;                          //  
        buf[1] = 0x00;                                      // Start each frame with pulse high
        pins.i2cWriteBuffer(CHIP_ADDRESS, buf, false);      // Write to PCA9685
        buf[0] = REG_ALL_LED_OFF_L;                         //
        buf[1] = 0x33;                                      // End high pulse at mid range 1.5mS = 1500/4.88uS = 307 (0x133)
        pins.i2cWriteBuffer(CHIP_ADDRESS, buf, false);      // Write to PCA9685
        buf[0] = REG_ALL_LED_OFF_H;                         //
        buf[1] = 0x01;                                      // End high pulse at mid range 1.5mS = 1500/4.88uS = 307 (0x133)
        pins.i2cWriteBuffer(CHIP_ADDRESS, buf, false);      // Write to PCA9685
        buf[0] = REG_MODE1;                                 //
        buf[1] = 0x01;                                      // Normal mode, start oscillator and allow LED all call registers
        pins.i2cWriteBuffer(CHIP_ADDRESS, buf, false)       // Write to PCA9685
        basic.pause(10);                                    // Let oscillator start and settle 
        PCA9685_init = true;                                // The PCA9685 is now initialised, no need to do it again
    }
	
    /**
     * Sets the requested servo to the reguested angle.
	 * If the PCA9685 has not yet been initialised calls the initialisation routine
	 *
     * @param Servo Which servo to set
	 * @param degrees the angle to set the servo to
     */
    //% blockId=I2C_servo_write
    //% block="set%Servo|to%degrees"
	//% degrees.min=0 degrees.max=180
	
    export function servoWrite(Servo: Servos, degrees: number): void {
        if (PCA9685_init == false) {                                        // PCA9685 initialised?
            init();                                                         // No, then initialise it 
        }
        let range: number = ServoRange[Servo - 1];                          // Get configured pulse range for specified servo
        let lolim: number = loPulseLim[range - 1];                          // Get lower pulse limit for the pulse range
        let hilim: number = hiPulseLim[range - 1];                          // Get upper pulse limit for the pulse range 
        let pulse: number = map(degrees, 0, 180, lolim, hilim);             // Map degrees 0-180 to pulse range
        let final: number = Math.floor(pulse);                              // No decimal points  
        let buf = pins.createBuffer(2);                                     // Create a buffer for i2c bus data 
        buf[0] = REG_SERVO1_BASE + (REG_SERVO_DISTANCE * (Servo - 1)) + 2;  // Calculate address of LED OFF low byte register
        buf[1] = final % 256;                                               // Calculate low byte value 
        pins.i2cWriteBuffer(CHIP_ADDRESS, buf, false);                      // Write low byte to PCA9685 
        buf[0] = REG_SERVO1_BASE + (REG_SERVO_DISTANCE * (Servo - 1)) + 3;  // Calculate address of LED OFF high byte register
        buf[1] = Math.floor(final / 256);                                   // Calculate high byte value
        pins.i2cWriteBuffer(CHIP_ADDRESS, buf, false);                      // Write high byte to PCA9685
    }

    /**
     * Sets the specified servo to the specified pulse range.
	 * On startup all 16 servos are set to the default pulse range of 0.5mS to 2.5mS
	 * This block is used to set the pulse range to a specific range, other than the default
     * 
     * @param Servo Which servo to alter the pulse range.
	 * @param Range The new pulse range for the servo.
     */
    //% blockId=I2C_set_pulse_range
    //% block="set%Servo|pulse range%PulseRange"
	export  function setRange (Servo: Servos, Range: PulseRange): void {
        ServoRange[Servo - 1] = Range;                  // Store new pulse range in servoRange array 
    }
}
