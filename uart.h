/*****************************************************************************
* Local definitions
*****************************************************************************/

/**@brief Disable logging to UART by commenting out this line. It is recomended to do this if
 * if you want to study timing in the example using a logic analyzer.
 */
#ifndef UART_H_
#define UART_H_
#define USE_UART_LOG

/**@brief Macro defined to output log data on the UART or not, based on the USE_UART_LOGGING flag.
 * If logging is disabled, it will just yield a NOP instruction.
 */
#ifdef USE_UART_LOG
#define __FILENAME__ (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)
#define __LOG(F, ...) (test_logf("BLE_MON: %s: %d: " #F "\r\n", __FILENAME__, __LINE__, ##__VA_ARGS__))
#else
  #define __LOG(F, ...) (void)__NOP()
#endif


/**@brief Local function prototypes.
 */
void test_logf(const char *fmt, ...);

void initialize_uart(void);

void simple_uart_putstring(const uint8_t * str);
#endif
