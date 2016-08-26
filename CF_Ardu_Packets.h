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
struct cl_toc_pkt
{
	byte command;
	byte index; // set to 0 when requesting crc
};

struct log_var_info
{
	byte log_type;
	byte varID;
};

struct cl_log_settings
{
	byte command;
	byte blockID;
	union {
		byte period;
		log_var_info variables[14]; // no mem requests for now
	};
};

struct cl_commander
{
	float roll;
	float pitch;
	float yaw;
	uint16_t thrust;
};


// define the packets we send
struct cl_packet
{
	byte header;
	union {
		byte payload[31];
		cl_toc_pkt toc;
		cl_log_settings log_settings;
		cl_commander setpoint;
	};
};

// --- CRAZYFLIE SIDE ---

struct cf_toc_crc_pkt
{
	byte command;
	byte num;
	uint32_t crc;
	byte max_blocks;
	byte max_vars;
};

struct cf_toc_item_pkt
{
	byte command;
	byte index;
	byte varType;
	char varName[28]; // there is group+var name
};

struct cf_log_settings
{
	byte command;
	byte blockID;
	byte status;
};

struct cf_log_data
{
	byte blockID;
	byte timestamp1;
	byte timestamp2;
	byte timestamp3; //sweet jesus...
	byte values[28];
};


struct cf_packet
{
	byte header;
	union {
		byte payload[31];
		cf_toc_crc_pkt crc_info;
		cf_toc_item_pkt item_info;
		cf_log_settings settings_resp;
		cf_log_data data;
	};
};

#endif