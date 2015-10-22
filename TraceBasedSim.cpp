/*********************************************************************************
*  Copyright (c) 2010-2011, Elliott Cooper-Balis
*                             Paul Rosenfeld
*                             Bruce Jacob
*                             University of Maryland 
*                             dramninjas [at] gmail [dot] com
*  All rights reserved.
*  
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions are met:
*  
*     * Redistributions of source code must retain the above copyright notice,
*        this list of conditions and the following disclaimer.
*  
*     * Redistributions in binary form must reproduce the above copyright notice,
*        this list of conditions and the following disclaimer in the documentation
*        and/or other materials provided with the distribution.
*  
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
*  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
*  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
*  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
*  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
*  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
*  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
*  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*********************************************************************************/



//TraceBasedSim.cpp
//
//File to run a trace-based simulation
//

#include <iostream>
#include <fstream>
#include <sstream>
#include <getopt.h>
#include <map>
#include <list>
#include <ctime>


#include "SystemConfiguration.h"
#include "MemorySystem.h"
#include "MultiChannelMemorySystem.h"
#include "Transaction.h"
#include "IniReader.h"


using namespace DRAMSim;
using namespace std;


time_t begin;
time_t end;

size_t cpuCycle=0;


int64_t *buf;
unsigned int timer;
unsigned int r_w;
unsigned long timerActual = 0;
unsigned int oldTimer;
int flag = 1;
FILE *fpTrace;
unsigned long currentNum = 0;
unsigned long NUM = 100000;




//#define RETURN_TRANSACTIONS 1

#ifndef _SIM_
int SHOW_SIM_OUTPUT = 1;
ofstream visDataOut; //mostly used in MemoryController

#ifdef RETURN_TRANSACTIONS
class TransactionReceiver
{
	private: 
		map<uint64_t, list<uint64_t> > pendingReadRequests; 
		map<uint64_t, list<uint64_t> > pendingWriteRequests; 

	public: 
		void add_pending(const Transaction &t, uint64_t cycle)
		{
			// C++ lists are ordered, so the list will always push to the back and
			// remove at the front to ensure ordering
			if (t.transactionType == DATA_READ)
			{
				pendingReadRequests[t.address].push_back(cycle); 
			}
			else if (t.transactionType == DATA_WRITE)
			{
				pendingWriteRequests[t.address].push_back(cycle); 
			}
			else
			{
				ERROR("This should never happen"); 
				exit(-1);
			}
		}

		void read_complete(unsigned id, uint64_t address, uint64_t done_cycle)
		{
			map<uint64_t, list<uint64_t> >::iterator it;
			it = pendingReadRequests.find(address); 
			if (it == pendingReadRequests.end())
			{
				ERROR("Cant find a pending read for this one"); 
				exit(-1);
			}
			else
			{
				if (it->second.size() == 0)
				{
					ERROR("Nothing here, either"); 
					exit(-1); 
				}
			}

			uint64_t added_cycle = pendingReadRequests[address].front();
			uint64_t latency = done_cycle - added_cycle;

			pendingReadRequests[address].pop_front();
			cout << "Read Callback:  0x"<< std::hex << address << std::dec << " latency="<<latency<<"cycles ("<< done_cycle<< "->"<<added_cycle<<")"<<endl;
		}
		void write_complete(unsigned id, uint64_t address, uint64_t done_cycle)
		{
			map<uint64_t, list<uint64_t> >::iterator it;
			it = pendingWriteRequests.find(address); 
			if (it == pendingWriteRequests.end())
			{
				ERROR("Cant find a pending read for this one"); 
				exit(-1);
			}
			else
			{
				if (it->second.size() == 0)
				{
					ERROR("Nothing here, either"); 
					exit(-1); 
				}
			}

			uint64_t added_cycle = pendingWriteRequests[address].front();
			uint64_t latency = done_cycle - added_cycle;

			pendingWriteRequests[address].pop_front();
			cout << "Write Callback: 0x"<< std::hex << address << std::dec << " latency="<<latency<<"cycles ("<< done_cycle<< "->"<<added_cycle<<")"<<endl;
		}
};
#endif

void usage()
{
	cout << "HRAMSim1 Usage: " << endl;
	cout << "HRAMSim1 -t tracefile -s system.ini -d ini/device.ini [-c #] [-p pwd] [-q] [-S 2048] [-n] [-o OPTION_A=1234,tRC=14,tFAW=19]" <<endl;
	cout << "\t-t, --tracefile=FILENAME \tspecify a tracefile to run  "<<endl;
	cout << "\t-b, --debug=FILENAME \t\tspecify a debug ini file "<<endl;
	cout << "\t-s, --systemini=FILENAME \tspecify an ini file that describes the memory system parameters  "<<endl;
	cout << "\t-x, --systeminiPcm=FILENAME \tspecify an ini file that describes the pcm memory system parameters  "<<endl;
	cout << "\t-d, --deviceini=FILENAME \tspecify an ini file that describes the device-level parameters"<<endl;
	cout << "\t-e, --deviceiniPcm=FILENAME \tspecify an pcm ini file that describes the device-level parameters"<<endl;
	cout << "\t-c, --numInstructions=# \tspecify number of instructions to run the simulation for [default=100000] "<<endl;
	cout << "\t-q, --quiet \t\t\tflag to suppress simulation output (except final stats) [default=no]"<<endl;
	cout << "\t-o, --option=OPTION_A=234,tFAW=14\toverwrite any ini file option from the command line"<<endl;
	cout << "\t-p, --pwd=DIRECTORY\t\tSet the working directory (i.e. usually DRAMSim directory where ini/ and results/ are)"<<endl;
	cout << "\t-S, --size=# \t\t\tSize of the memory system in megabytes [default=2048M]"<<endl;
	cout << "\t-X, --sizePcm=# \t\tSize of the pcm memory system in megabytes [default=2048M]"<<endl;
	cout << "\t-n, --notiming \t\t\tDo not use the clock cycle information in the trace file"<<endl;
	cout << "\t-v, --visfile \t\t\tVis output filename"<<endl;
}
#endif

void *parseTraceFileLine_new(uint64_t &addr, enum TransactionType &transType, uint64_t &clockCycle, TraceType type, bool useClockCycle)
{

	uint64_t *dataBuffer = NULL;
	string addressStr="", cmdStr="", dataStr="", ccStr="";
	
	
		
	timer  = (unsigned int)( ((buf[currentNum])>>30) & 0xfffffULL);
	r_w    = (unsigned int)( ((buf[currentNum])>>29) & 0x1ULL );
	addr   = (uint64_t)( (buf[currentNum]<<3) & 0xffffffffULL );
	if(flag)		
	{			
		flag = 0;
	}
	else
	{			
		timerActual += oldTimer;
	}
	
	
	if(r_w == 1)
	{
		transType = DATA_READ;
	}
	else if(r_w == 0)
	{
		transType = DATA_WRITE;
	}
	else
	{
		ERROR("== Unknown Command : "<<cmdStr);
		exit(0);
	}

	//istringstream a(addressStr.substr(2));//gets rid of 0x
//	istringstream a(addressStr);//gets rid of 0x
//	a>>hex>>addr;

	//if this is set to false, clockCycle will remain at 0, and every line read from the trace
	//  will be allowed to be issued
	if (useClockCycle)
	{
		clockCycle = timerActual;
	}
	
	oldTimer = timer*5;
	

	return dataBuffer;
}


#ifndef _SIM_

void alignTransactionAddress(Transaction &trans)
{
	// zero out the low order bits which correspond to the size of a transaction
  /*

	unsigned throwAwayBits = dramsim_log2((BL*JEDEC_DATA_BUS_BITS/8));

	trans.address >>= throwAwayBits;
	trans.address <<= throwAwayBits;
  */
}

/** 
 * Override options can be specified on the command line as -o key1=value1,key2=value2
 * this method should parse the key-value pairs and put them into a map 
 **/ 
IniReader::OverrideMap *parseParamOverrides(const string &kv_str)
{
	IniReader::OverrideMap *kv_map = new IniReader::OverrideMap(); 
	size_t start = 0, comma=0, equal_sign=0;
	// split the commas if they are there
	while (1)
	{
		equal_sign = kv_str.find('=', start); 
		if (equal_sign == string::npos)
		{
			break;
		}

		comma = kv_str.find(',', equal_sign);
		if (comma == string::npos)
		{
			comma = kv_str.length();
		}

		string key = kv_str.substr(start, equal_sign-start);
		string value = kv_str.substr(equal_sign+1, comma-equal_sign-1); 

		(*kv_map)[key] = value; 
		start = comma+1;

	}
	return kv_map; 
}

int main(int argc, char **argv)
{
	
	begin = clock();
	
	int c;
	TraceType traceType;
	string traceFileName;
	string debugIniFilename;
	string systemIniFilename("systemdram.ini");
	string deviceIniFilename;
	string systemIniFilenamePcm("systempcm.ini");
	string deviceIniFilenamePcm;
	string pwdString;
	string *visFilename = new string("xxx.vis");
	unsigned megsOfMemory=2048;
	unsigned megsOfMemoryPcm=2048;
	bool useClockCycle=true;
	
	IniReader::OverrideMap *paramOverrides = NULL; 


	//getopt stuff
	while (1)
	{
		static struct option long_options[] =
		{
			{"deviceini", required_argument, 0, 'd'},
			{"deviceiniPcm", required_argument, 0, 'e'},
			{"tracefile", required_argument, 0, 't'},
			{"systemini", required_argument, 0, 's'},
			{"systeminiPcm", required_argument, 0, 'x'},
			{"debug", required_argument, 0, 'b'},
			{"pwd", required_argument, 0, 'p'},
			{"numInstructions",  required_argument,	0, 'c'},
			{"option",  required_argument,	0, 'o'},
			{"quiet",  no_argument, &SHOW_SIM_OUTPUT, 'q'},
			{"help", no_argument, 0, 'h'},
			{"size", required_argument, 0, 'S'},
			{"sizePcm", required_argument, 0, 'X'},
			{"visfile", required_argument, 0, 'v'},
			{0, 0, 0, 0}
		};
		int option_index=0; //for getopt
		c = getopt_long (argc, argv, "t:b:s:x:c:d:e:o:p:X:S:v:qn", long_options, &option_index);
		if (c == -1)
		{
			break;
		}
		switch (c)
		{
		case 0: //TODO: figure out what the hell this does, cuz it never seems to get called
			if (long_options[option_index].flag != 0) //do nothing on a flag
			{
				printf("setting flag\n");
				break;
			}
			printf("option %s",long_options[option_index].name);
			if (optarg)
			{
				printf(" with arg %s", optarg);
			}
			printf("\n");
			break;
		case 'h':
			usage();
			exit(0);
			break;
		case 't':
			traceFileName = string(optarg);
			break;
		case 'b':
			debugIniFilename = string(optarg);
      break;
		case 's':
			systemIniFilename = string(optarg);
			break;
		case 'x':
			systemIniFilenamePcm = string(optarg);
			break;
		case 'd':
			deviceIniFilename = string(optarg);
			break;
		case 'e':
			deviceIniFilenamePcm = string(optarg);
			break;
		case 'c':
			NUM = atol(optarg);
			break;
		case 'X':
			megsOfMemoryPcm=atoi(optarg);
			break;
		case 'S':
			megsOfMemory=atoi(optarg);
			break;
		case 'p':
			pwdString = string(optarg);
			break;
		case 'q':
			SHOW_SIM_OUTPUT=false;
			break;
		case 'n':
			useClockCycle=false;
			break;
		case 'o':
			paramOverrides = parseParamOverrides(string(optarg)); 
			break;
		case 'v':
			visFilename = new string(optarg);
			break;
		case '?':
			usage();
			exit(-1);
			break;
		}
	}

	// get the trace filename
//	string temp = traceFileName.substr(traceFileName.find_last_of("/")+1);

	//get the prefix of the trace name
/*
	temp = temp.substr(0,temp.find_first_of("_"));
	if (temp=="mase")
	{
		traceType = mase;
	}
	else if (temp=="k6")
	{
		traceType = k6;
	}
	else if (temp=="misc")
	{
		traceType = misc;
	}
	else
	{
		ERROR("== Unknown Tracefile Type : "<<temp);
		exit(0);
	}
*/
	traceType = k6;

	// no default value for the default model name
	if (deviceIniFilename.length() == 0)
	{
		ERROR("Please provide a device ini file");
		usage();
		exit(-1);
	}


	//ignore the pwd argument if the argument is an absolute path
	if (pwdString.length() > 0 && traceFileName[0] != '/')
	{
		traceFileName = pwdString + "/" +traceFileName;
	}

	DEBUG("== Loading trace file '"<<traceFileName<<"' == ");

	ifstream traceFile;
	string line;


	MultiChannelMemorySystem *memorySystem = new MultiChannelMemorySystem(debugIniFilename,deviceIniFilename,deviceIniFilenamePcm, systemIniFilename,systemIniFilenamePcm, pwdString, traceFileName, megsOfMemory,megsOfMemoryPcm, visFilename);//, paramOverrides);
	// set the frequency ratio to 1:1
	//memorySystem->setCPUClockSpeed(0); 
	//std::ostream &dramsim_logfile = memorySystem->getLogFile(); 
	// don't need this anymore 
	delete paramOverrides;


#ifdef RETURN_TRANSACTIONS
	TransactionReceiver transactionReceiver; 
	// create and register our callback functions 
	Callback_t *read_cb = new Callback<TransactionReceiver, void, unsigned, uint64_t, uint64_t>(&transactionReceiver, &TransactionReceiver::read_complete);
	Callback_t *write_cb = new Callback<TransactionReceiver, void, unsigned, uint64_t, uint64_t>(&transactionReceiver, &TransactionReceiver::write_complete);
	memorySystem->RegisterCallbacks(read_cb, write_cb, NULL);
#endif


	uint64_t addr;
	uint64_t clockCycle=0;
	enum TransactionType transType;

	void *data = NULL;
	Transaction *trans=NULL;
	bool pendingTrans = false;

/*
	traceFile.open(traceFileName.c_str());

	if (!traceFile.is_open())
	{
		cout << "== Error - Could not open trace file"<<endl;
		exit(0);
	}
*/	

	buf = (int64_t*)malloc(8*NUM);
	if(!buf)     
	{        
		cout<<"malloc failed"<<endl;       
		exit(0);  
	}
	if((fpTrace = fopen(traceFileName.c_str(), "r")) == NULL)    
	{        
		cout<<"Open traceFile  error"<<endl;
		exit(0);
	}
	if(fread(buf,8, NUM,fpTrace)!=NUM)
	{	
		cout<<"No enough requests in the input trace."<<endl;
		exit(0);
	}

	while(currentNum<NUM)
	{
		cpuCycle++;
		if (!pendingTrans)
		{
			if (!feof(fpTrace))
			{

				
					data = parseTraceFileLine_new(addr, transType,clockCycle, traceType,useClockCycle);
					trans = new Transaction(transType, addr, data);
					alignTransactionAddress(*trans); 

					if (cpuCycle>=clockCycle)
					{
						if (!(*memorySystem).addTransaction(trans))
						{
							pendingTrans = true;
						}
						else
						{
#ifdef RETURN_TRANSACTIONS
							transactionReceiver.add_pending(trans, cpuCycle); 
#endif
							// the memory system accepted our request so now it takes ownership of it
							trans = NULL; 
							currentNum++;

						}
					}
					else
					{
						pendingTrans = true;
					}
				
			}
			else
			{
				//we're out of trace, set pending=false and let the thing spin without adding transactions
				pendingTrans = false; 
			}
		}

		else if (pendingTrans && cpuCycle >= clockCycle)
		{
			pendingTrans = !(*memorySystem).addTransaction(trans);
			if (!pendingTrans)
			{
#ifdef RETURN_TRANSACTIONS
				transactionReceiver.add_pending(trans, cpuCycle); 
#endif
				trans=NULL;
				currentNum++;

			}
		}

		(*memorySystem).update();
	}

	//traceFile.close();
	fclose(fpTrace);
	memorySystem->printStats(true);
	// make valgrind happy
	if (trans)
	{
		delete trans;
	}
	delete(memorySystem);

	end = clock();

	cout<<"clock:"<<cpuCycle<<endl;
	cout<< "Running time is: "<<static_cast<double>(end-begin)/CLOCKS_PER_SEC/60<<"min"<<endl;
	cout<< "Running time is: "<<static_cast<double>(end-begin)/CLOCKS_PER_SEC*1000<<"ms"<<endl;
  
}
#endif
