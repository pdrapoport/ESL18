/*------------------------------------------------------------------
 *  pc_archi.h - Contains the first draft of architecture (function prototypes)
 *  for the pc side of the control
 *
 *  Vincent Bejach
 *  Embedded Software Lab
 *
 *  April 2018
 *------------------------------------------------------------------
 */

// -------------------------------------------------------------------------------------------
// Communication functions

// Sends the command to switch mode to the drone. Returns true if the send was successful, false if not
bool sendModeSwitch(uint8_t modeID);

/* Sends the mouvement orders given by the user at the keyboard or the joystick
 * (Basically sends the content of array ae)
 * Similarly to sendModeSwitch, returns true if all went well and false otherwise
 */ 
bool sendMvtCom(void);



// -------------------------------------------------------------------------------------------
// Input functions

/* Periodically checks the state of the keyboard to detect pressed keys.
 * Calls process_key() to deal with the pressed key if there is one, do nothing otherwise.
 */
void sampleKeyboard(void);

/* Basically the same as sampleKeyboard(), but for the joystick input
 * Should call a function process_joystick(), similar to process_keyboard()
 */
void sampleJoystick(void);

/* Processes joystick inputs and translates them into modifications of ae values
 * In case of unmapped command being used, toggle RED led
 */
void process_joystick(uint8_t j);

// -------------------------------------------------------------------------------------------
// GUI functions (TBD - not crucial for the moment)
// Should at least print the angles measured and the height.
// Could additionally print the mode we´re currently in.



// -------------------------------------------------------------------------------------------
// Mission status

// Launch the abort sequence, switching to panic mode and landing the qudcopter
void abort(void);

/* Check battery voltage 
 * Returns false if battery voltage is below 11.5 or 12 V (ie a little while before critical voltage). If battery voltage is below 11 V, this function calls the abort() procedure to land the quadcopter.
 * It returns true if all is well
 */
bool checkBattery(void);

