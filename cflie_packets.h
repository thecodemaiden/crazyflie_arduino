#ifndef __CF_ARDU_PKT__
#define __CF_ARDU_PKT__

/***
The MIT License (MIT)
Copyright (c) <2016> <Adeola Bannis>

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation
files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify,
merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished
to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
***/

// specialised packet types to fit in payload
//----CLIENT SIDE----
struct cf_packet
{
    void pack(uint8_t *out) {};
};

struct cl_toc_pkt 
{
	uint8_t command;
	uint8_t index; // set to 0 when requesting crc

    void pack(uint8_t *out) {
        out[0] = command;
        out[1] = index;
    }
};

struct log_var_info 
{
	uint8_t log_type;
	uint8_t varID;
};

struct cl_log_settings
{
	uint8_t command;
	uint8_t blockID;
	union {
		uint8_t period;
		log_var_info variables[14]; // no mem requests for now
	};
    void pack(uint8_t *out) {
        memcpy(out, &command, 1);
        memcpy(out+1, &blockID, 1);
        if (command == 0x3) { // TODO: more enums/defines!
            memcpy(out+2, &period, 1);
        } else {
            int loc = 2;
            for (uint8_t i=0; i<14; i++){
                memcpy(out+loc, &variables[i].log_type, 1);
                memcpy(out+loc+1, &variables[i].varID, 1);
                loc +=2;
             }
        }
    }
};

struct cl_commander
{
	float roll;
	float pitch;
	float yaw;
	uint16_t thrust;

    void pack(uint8_t *out) {
        memcpy(out, &roll, 4);
        memcpy(out+4, &pitch, 4);
        memcpy(out+8, &yaw, 4);
        memcpy(out+12, &thrust,2);
    }
};

// --- CRAZYFLIE SIDE ---

struct cf_toc_crc_pkt
{
	uint8_t command;
	uint8_t num;
	uint32_t crc;
	uint8_t max_blocks;
	uint8_t max_vars;
    void unpack(uint8_t *out) {
        memcpy(&command, out, 1);
        memcpy(&num, out+1, 1);
        memcpy(&crc, out+2, 4);
        memcpy(&max_blocks, out+6, 1);
        memcpy(&max_vars, out+7, 1);
    }
};

struct cf_toc_item_pkt
{
	uint8_t command;
	uint8_t index;
	uint8_t varType;
	char varName[28]; // there is group+var name
    void unpack(uint8_t *out) {
        memcpy(&command, out, 1);
        memcpy(&index, out+1, 1);
        memcpy(&varType, out+2, 1);
        memcpy(varName, out+3, 28);
    }
};

struct cf_log_settings
{
	uint8_t command;
	uint8_t blockID;
	uint8_t status;
    void unpack(uint8_t *out) {
        memcpy(&command, out, 1);
        memcpy(&blockID, out+1, 1);
        memcpy(&status, out+2, 1);
    }
};

struct cf_log_data
{
	uint8_t blockID;
    uint32_t timestamp;
	uint8_t values[28];
    void unpack(uint8_t *out) {
        blockID = out[0];
        timestamp = out[3];
        timestamp = (timestamp << 8) + out[2];
        timestamp = (timestamp << 8) + out[1]; // TODO: check byte order
        memcpy(values, out+4, 28);
    }
};

#endif
