/*
 * The MIT License (MIT)
 *
 * Copyright (c) [2015] [Marco Russi]
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
*/




/* Uncomment the following define for enabling LED debug regardless UART */
#define LED_DEBUG

/* Uncomment the following define for enabling debug over UART */
//#define UART_DEBUG

/* Uncomment the following define for enabling the test of face index update in the BLE adv packet */
//#define FACE_INDEX_TEST

/* Uncomment the following define for limiting acceleration detection to 2G
   (any values bigger than 2G will be limited to 2G ) */
#define LIMIT_XYZ_VALUES_TO_2G

/* Update time in ms for burst read of MPU6050 */
#define CFG_MPU6050_BURST_READ_UPDATE_MS			1000

/* Update time in ms for BLE advertisement packet */
#define CFG_BLE_ADV_UPDATE_MS						2000




/* End of file */




