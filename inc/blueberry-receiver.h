/*
Copyright (c) 2025 Blue Robotics North Inc.

Permission is hereby granted, free of charge, to any person obtaining a copy of
this software and associated documentation files (the "Software"), to deal in
the Software without restriction, including without limitation the rights to
use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies
of the Software, and to permit persons to whom the Software is furnished to do
so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/

#ifndef BLUEBERRY_RECEIVER_H_
#define BLUEBERRY_RECEIVER_H_

//*******************************************************************************************
//Includes
//*******************************************************************************************


#include <stdbool.h>

#include "blueberry-transcoder.h"

//*******************************************************************************************
//Defines
//*******************************************************************************************
#define BB_UDP_PORT 0x4242
//*******************************************************************************************
//Types
//*******************************************************************************************

typedef enum {
	BB_GOOD,
	BB_FAIL,
	BB_INCOMPLETE
} BbResult;

//*******************************************************************************************
//Variables
//*******************************************************************************************

//*******************************************************************************************
//Function Prototypes
//*******************************************************************************************
//



/**
 * scans the next byte the input queue for packets, if found will respond with a response packet on the output queue.
 * this funcion will only process at most n bytes at a time
 * It uses the input buffer to store the recieving state between calls.
 *
 * @param inP - a packet used for receiving. This should be static
 * @param inQ - the queue that the bytes
 * @param outQ - the queue that a response packet will be sent on
 * @param n - the maximum number of bytes to process in one run of this function - if 0 then assumes the whole packet is in the inQ
 */
BbResult blueberryReceiveAndParseByte(Bb* bb);

/**
 * Decodes a packet from the start of a buffer, assuming it is all there
 * This is indended for use with ethernet
 * This function will not test the CRC
 * @param buf - the buffer for this packet, also the state of the receive routine
 * @return true if a valid packet was received
 */
BbResult blueberryReceiveAndParsePacket(Bb* buf);

//*******************************************************************************************
//Code
//*******************************************************************************************

#endif /* BLUEBERRY_RECEIVER_H_ */
