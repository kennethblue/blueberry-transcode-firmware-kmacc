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


//*******************************************************************************************
//Includes
//*******************************************************************************************
#include <blueberry-parser.h>
#include <blueberry-receiver.h>
#include <blueberry-message.h>
#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>


//*******************************************************************************************
//Defines
//*******************************************************************************************
#define PROCESSOR_NUM (100)
#define MSG_Q_SIZE (50)

#define MAKE_KEY(mod, msg) ((((uint32_t)mod) << 16) | ((uint32_t)msg))

#define PACKET_PREAMBLE (0x65756c42) //(0x45554c42)
#define PACKET_PREAMBLE_INDEX (0)
#define PACKET_LENGTH_INDEX (4)
#define PACKET_CRC_INDEX (6)
#define PACKET_FIRST_MESSAGE_INDEX (8)




//*******************************************************************************************
//Types;
//*******************************************************************************************
typedef struct {
	uint32_t key;
	BbProcessor parser;
} ProcessorKeyValue;

typedef struct {
	ProcessorKeyValue m_processors[PROCESSOR_NUM];
	uint32_t num;
} Processors;

//*******************************************************************************************
//Variables
//*******************************************************************************************
static Processors m_parsers;
static Processors m_builders;
static uint32_t m_rxMsgs[MSG_Q_SIZE];
static uint32_t m_rxIndex = 0;
//*******************************************************************************************
//Function Prototypes
//*******************************************************************************************
/**
 * find the parser that is assigned to the specified key
 * @param key - the specified key
 * @param index - a pointer to the resulting index value - this will indicate the index of the equal or first greater key value
 * @return - -1 if not found, else the index of the function pointer of the parser
 */
static BbProcessor lookup(Processors * ps, uint32_t key, uint32_t * index);
static void registerProcessor(Processors * ps, uint32_t key, BbProcessor p);
//*******************************************************************************************
//Code
//*******************************************************************************************
/**
 * Must be called at init
 */
void initBbParser(void){
	m_parsers.num = 0;
	m_builders.num = 0;

}

/**
 * processes a blueberry packet and parses each message
 * This assumes that the buffer has been properly received and is at the start of the packet
 */
void parseBbPacket(Bb* buf){

	uint32_t packetLength = getBbUint16(buf, 0, PACKET_LENGTH_INDEX)*4;
	if(packetLength == 0){
		return;
	}
	BbBlock msg = PACKET_FIRST_MESSAGE_INDEX;
	m_rxIndex = 0;
	while(msg < packetLength){
		//we have enough data for a message
		uint32_t k = getBbMessageKey(buf, msg);
		uint32_t len = getBbMessageLength(buf, msg);
		//record in the queue that the particular type of message was received
		queueBbMessage(k);
		if(isBbMessageEmpty(buf, msg)){

		} else {


			uint32_t i;
			BbProcessor p = lookup(&m_parsers, k, &i);
			if(p != NULL){
				//call the parser
				(*p)(buf, msg);
			}
		}

		msg += len;

	}
}
/**
 * requests that the next packet should have the message with the specified key added.
 * @param key - the module/message key for the desired message
 */
void queueBbMessage(uint32_t key){
	if(m_rxIndex < MSG_Q_SIZE){
		m_rxMsgs[m_rxIndex] = key;
		++m_rxIndex;

	}

}



/**
 * registers a parser for a given message
 * This will add a parser if module & key are new
 * This will overwrite a parser if module & key already exist in the list
 * Will fail silently if the list is full
 */
void registerBbParser(uint32_t moduleMessageKey, BbProcessor parser){
	registerProcessor(&m_parsers, moduleMessageKey, parser);
}
/**
 * register a message processor for adding a message to a buffer
 */
void registerBbBuilder(uint32_t moduleMessageKey, BbProcessor builder){
	registerProcessor(&m_builders, moduleMessageKey, builder);
}

static void registerProcessor(Processors * ps, uint32_t key, BbProcessor p){

	if(ps->num >= PROCESSOR_NUM){
		return;
	}
	uint32_t i;
	uint32_t k = key;
	BbProcessor pt = lookup(ps, k, &i);
	if(pt == NULL){
		//we didn't find a match so make room by
		//moving everything up one, from the ith location to the end
		for(uint32_t j = ps->num; j > i; --j){
			ProcessorKeyValue * kvNew = &(ps->m_processors[j]);
			ProcessorKeyValue * kvOld = &(ps->m_processors[j - 1]);
			kvNew->key = kvOld->key;
			kvNew->parser = kvOld->parser;
		}
		++ps->num;//we now have one more parser
	}
	ps->m_processors[i].key = k;
	ps->m_processors[i].parser = p;
}




/**
 * find the parser that is assigned to the specified key
 * or find the position where the specified key might be placed in the list if it is not there
 * @param ps - the processors struct to look in
 * @param key - the key to lookup
 * @param index - a pointer to the resulting index value - this will indicate the index of the equal or first greater key value
 * @return - -1 if not found, else the index of the function pointer of the parser
 */
static BbProcessor lookup(Processors * ps, uint32_t key, uint32_t * index){

	BbProcessor result = NULL;
	//use binary search to find parser
	uint32_t min = 0;
	uint32_t max = ps->num == 0 ? 0 : ps->num - 1;//this will be one past the last element of the list
	uint32_t i;
	uint32_t ikey = 0;
	while(true){
		i = (min + max) / 2;//i will be greater than or equal to i because of the integer math
		ProcessorKeyValue kv = ps->m_processors[i];
		ikey = kv.key;
		if(ps->num == 0){
			break;
		} else if(ikey == key){
			result = kv.parser;
			break;
		} else if(ikey < key){
			//the key in the list is less than the search key
			++i;//advance to the next spot
			if(i > max){
				//we're already at the end of the list so
				break;
			}
			min = i;//constrain the range based on the latest test
		} else {
			//the key in the list is greater than the search key
			//so the right answer probably lives below this one
			if(i <= min || min == max){
				//if we're at the minimum of the range or the range has shrunk to 0 then I guess we're done.
				break;
			}
			//the current location is too big so move to the next smaller one
			//and constrain the range accordingly
			--i;
			max = i;
		}
	}
	*index = i;
	return result;
}
/**
 * checks if a potential packet has at least enough bytes received to contain a packet header
 */
bool minBbLengthCheck(Bb* bb){
	return bb->length >= PACKET_FIRST_MESSAGE_INDEX;
}
/**
 * a function to test the start word of the packet. It will check only up to the Bb.length. It should return true so long as the start word is good
 */
bool checkBbPreamble(Bb* bb){
	uint32_t n = bb->length;
	uint32_t a = 0;
	n = n > 4 ? 4 : n;//n will now cap out at 4
	uint32_t b = PACKET_PREAMBLE & (0xffffffff >> ((4 - n)*8));
	for(uint32_t j = 0; j < n; ++j){
		a |= ((uint32_t)getBbUint8(bb, 0, PACKET_PREAMBLE_INDEX + j)) << (j*8);
	}


	return (a ^ b) == 0;
}
/**
 * a function to test the length of the received packet so far. It should return true when enough bytes have been received
 */
bool checkBbLength(Bb* bb){
	uint32_t len = getBbUint16(bb, 0, PACKET_LENGTH_INDEX)*4;
	uint32_t n = bb->length;

	return n >= PACKET_FIRST_MESSAGE_INDEX && n >= len;
}
/**
 * a function to check the CRC of the received bytes. It will return true with a correct match
 */
bool checkBbCrc(Bb* bb){
	uint32_t n = bb->length;
	uint16_t crcA = (uint16_t)getBbUint16(bb, 0, PACKET_CRC_INDEX);
	uint16_t crcB = computeBbCrc(bb, PACKET_FIRST_MESSAGE_INDEX, n);
	return crcA == crcB;
}

/**
 * does any preliminary header setup and computes the location for the starting message
 */
BbBlock startBbPacket(Bb* bb){
	bb->length = PACKET_FIRST_MESSAGE_INDEX;
	setBbUint32(bb, 0, 0, PACKET_PREAMBLE);
//	setBbUint16(bb, 0, PACKET_CRC_INDEX, 0xffff);
//	setBbUint16(bb, 0, PACKET_LENGTH_INDEX, 0);

	return PACKET_FIRST_MESSAGE_INDEX;
}
/**
 * resets the buffer so it's as if there was no packet started
 */
void undoBbPacketStart(Bb* bb){
	bb->length = 0;
}
/**
 * Finalize the packet in preparation for sending
 * This relies on the buffer having the final length set correctly
 * And all messages construted correctly
 * @param bb - the buffer that the packet has been constructed in
 * @param doCrc - will compute a CRC for the packet if true, otherwise not

 */
void finishBbPacket(Bb* bb, bool doCrc){
	uint32_t n = bbAlign(bb->length);
	setBbUint16(bb, 0, PACKET_LENGTH_INDEX, (uint16_t)(n/4));
	uint16_t crc = doCrc ? computeBbCrc(bb, PACKET_FIRST_MESSAGE_INDEX, n) : 0xffff;
	setBbUint16(bb, 0, PACKET_CRC_INDEX, crc);
}

/**
 * indicates that messages were received and should trigger a corresponding packet of messages to be sent
 */
bool isBbPacketRequested(){
	return m_rxIndex > 0;
}

/**
 * Make a packet in the specified buffer that contains all queued messages
 * Note that a packet may not be fully completed, in which case the buffer will still have a length of zero
 * @param bb - the buffer to make the packet in
 * @param doCrc - will compute a CRC for the packet if true, otherwise not

 */
void makeBbPacketWithQueuedMessages(Bb* bb, bool doCrc){
	bool started = false;
	BbBlock msg = PACKET_FIRST_MESSAGE_INDEX;

	for(uint32_t i = 0; i < m_rxIndex; ++i){
		uint32_t key = m_rxMsgs[i];
		uint32_t pi = 0;
		BbProcessor p = lookup(&m_builders, key, &pi);
		if(p != NULL){
			if(!started){
				startBbPacket(bb);
				started = true;
			}
			msg = bb->length;//point to the next free byte of the buffer
			(*p)(bb, msg);

		}



	}
	if(started && bb->length > PACKET_FIRST_MESSAGE_INDEX){
		finishBbPacket(bb, doCrc);
	} else {
		undoBbPacketStart(bb);
	}



}

/**
 * takes the specified value and rounds it up to the nearest multiple of 4
 * this is useful to compute the next greater index that is word-aligned
 * or to round up a message length to the nearest 4-bytes
 */
BbBlock bbAlign(uint16_t i){
	BbBlock result = i;

	if(i & 0b11){
		result |= 0b11;
		result += 1;
	}
	return result;
}


/**
 * computes the  CRC-16-CCITT of the buffer.
 * This function has a weak implementation and probably should be re-implemented with hardware specific code
 * Assumes the buffer contains a packet and the packet will be a multiple of 4-bytes long
 * @param buf - the buffer containing the packet
 * @param block - the location of the first message. This is the index of the first byte after the packet header.
 * @param end - one past the last element
 */
uint16_t __attribute__((weak)) computeBbCrc(Bb* buf, BbBlock block, BbBlock end){

	uint16_t crc = 0xffff;
	uint16_t x;
	uint16_t d;
	for(uint16_t i = block; i < end; ++i){
		d = (uint16_t)getBbUint8(buf, i, 0);
		x = ((crc>>8) ^ d) & 0xff;
		x ^= x>>4;
		crc = (crc << 8) ^ (x << 12) ^ (x <<5) ^ x;
	}

	return crc;
}



