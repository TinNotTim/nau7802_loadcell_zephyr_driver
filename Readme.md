# Out of tree Adafruit NAU7802 loadcell driver module for Zephyr RTOS

*NAU7802 is a precision 24-bit analog-to-digital converter (ADC) designed for weigh scale applications.*

## Recent Major Updates
This update implements 5 major fixes.


* **System Boot Blocking:** Moved the hardware initialization out of the main init function and put it into a delayed work handler. The hardware takes over 900ms to power up and calibrate, which was blocking the Zephyr kernel from booting. Putting this in a background thread lets the system boot without wait, and it will safely return -EBUSY if you try to fetch a sample before it is ready.
* **Operator Precedence Bugs:** I added parentheses to the polling loops checking the CTRL2 register. The inequality operator binds tighter than the bitwise AND operator in C. Because of this, the code was evaluating the inequality first and checking bit 0 instead of the actual calibration status bits.
* **Strict Aliasing Violations:** I swapped out the bit level memcpy commands for standard Zephyr APIs. Copying raw float memory bits directly into the integer member of the sensor value struct goes against strict aliasing rules and causes undefined behavior during compiler optimization. It now correctly parses the integer and fractional millionths.
* **Non Standard Data Types:** I replaced all instances of float32_t with standard C double types. The float32_t type is specific to ARM CMSIS DSP and can cause compilation errors for standard builds. Using standard doubles guarantees the driver stays hardware agnostic while keeping plenty of precision for the 24-bit ADC.
* **Device Tree Hardware Mappings:** The I2C lines and data ready hardware interrupts are now routed via overlay files rather than being hardcoded in the C driver.

## Supported Zephyr versions
* Tested on v3.5.99 using nRF Connect SDK v3.3.1

## [Forked from this repo](https://github.com/nobodyguy/nau7802_loadcell_zephyr_driver)