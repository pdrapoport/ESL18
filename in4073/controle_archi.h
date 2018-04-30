/*------------------------------------------------------------------
 *  control_archi.h - Contains the first draft of architecture (function prototypes)
 *  for the drone side of the control
 *
 *  Vincent Bejach
 *  Embedded Software Lab
 *
 *  April 2018
 *------------------------------------------------------------------
 */

/* General note: each function of this file is assumed to log what it is doing, at the
 * very last before returning, but possibly ¨on the fly¨
 */

// -------------------------------------------------------------------------------------------
// Global variables

uint8_t mode;

int32_t sp; // p angular rate gyro sensor output
int32_t sq; // q angular rate gyro sensor output
int32_t sr; // r angular rate gyro sensor output
int32_t sax; // x-axis accelerometer sensor output
int32_t say; // y-axis accelerometer sensor output
int32_t saz; // z-axis accelerometer sensor output
// lift, roll, pitch, yaw to be added

// -------------------------------------------------------------------------------------------
// Communication functions

/* Send quadcopter attitude & height back to the PC for printing.
 * All communication functions return true if the sending was successful and false otherwise.
 */
bool sendCurrentAngles(int16_t phi, int16_t theta, int16_t psi);

bool sendCurrentHeight(uint8_t heigth);

// Sends back all logging data stored in the flash to the PC for analysis
// Returns the same as the other communication functions
bool flushLogs(void);



// -------------------------------------------------------------------------------------------
// Book keeping

// Read sensor values while in safe mode and calibrate the sensor to know the current attitude is level.
void calibrate(void);

// Switch the drone mode to modeID (that should ultimately be comprised between 0 and 8)
void switchMode(uint8_t modeID);



// -------------------------------------------------------------------------------------------
// Input functions

/* Reads the sensor input on quadcopter attitude.
 * Writes the output in global variables sp, sq, sr.
 * The return values are used in case something goes wrong, to specify what went wrong (unable to access sensor, nonsensical data, etc)
 */
uint8_t readAttitude(void);

// Reads input from barometer and computes height
unin8_t readHeight(void);

