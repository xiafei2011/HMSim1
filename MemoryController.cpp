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



//MemoryController.cpp
//
//Class file for memory controller object
//

#include "MemoryController.h"
#include "MemorySystem.h"
#include "AddressMapping.h"
#include <algorithm>

#define SEQUENTIAL(rank,bank) (rank*iniReader->NUM_BANKS)+bank

extern size_t cpuCycle;
extern unsigned long timerActual;

double totalBandwidth_MS = 0.0;
//uint64_t latency_MS = 0;
uint64_t totalEpochLatency_MS = 0;
uint64_t totalReads_MS = 0;
uint64_t totalWrites_MS = 0;

uint64_t actpreNum = 0;
uint64_t burstNum = 0;
uint64_t refreshNum = 0;

uint64_t actpreNum_dram = 0;
uint64_t burstNum_dram = 0;
uint64_t refreshNum_dram = 0;
uint64_t actpreNum_pcm = 0;
uint64_t burstNum_pcm = 0;
uint64_t refreshNum_pcm = 0;



//double totalEnergy_MS = 0.0;
double totalActpreEnergy_MS = 0.0;
double totalBurstEnergy_MS = 0.0;
double totalRefreshEnergy_MS = 0.0;

uint64_t dram_read=0;
uint64_t dram_write=0;
uint64_t pcm_read=0;
uint64_t pcm_write=0;

uint64_t rowBufferHitCount_dram = 0;
uint64_t rowBufferHitCount_pcm = 0;

double totalReadsBandwidth_MS=0.0;
double totalWritesBandwidth_MS=0.0;
vector<unsigned> readLatency;
vector<unsigned> writeLatency;
vector<unsigned> transactionQueueDelay;
vector<unsigned> commandQueueDelay;


//ofstream ofs_dram("epoch_dram.txt");
//ofstream ofs_pcm("epoch_pcm.txt");


using namespace DRAMSim;

MemoryController::MemoryController(MemorySystem *parent, CSVWriter &csvOut_, ostream &dramsim_log_) :
    allIniReaders(parent->allIniReaders),
    iniReader(parent->iniReader),
		dramsim_log(dramsim_log_),
		bankStates(parent->iniReader->NUM_RANKS, vector<BankState>(parent->iniReader->NUM_BANKS, dramsim_log)),
		commandQueue(bankStates, dramsim_log_,parent->iniReader),
		poppedBusPacket(NULL),
		csvOut(csvOut_),
		totalTransactions(0),
		refreshRank(0)
{
	//get handle on parent
	parentMemorySystem = parent;


	//bus related fields
	outgoingCmdPacket = NULL;
	outgoingDataPacket = NULL;
	dataCyclesLeft = 0;
	cmdCyclesLeft = 0;

	//set here to avoid compile errors
	currentClockCycle = 0;

	//reserve memory for vectors
	transactionQueue.reserve(iniReader->TRANS_QUEUE_DEPTH);
	powerDown = vector<bool>(iniReader->NUM_RANKS,false);
	grandTotalBankAccesses = vector<uint64_t>(iniReader->NUM_RANKS*iniReader->NUM_BANKS,0);
	totalReadsPerBank = vector<uint64_t>(iniReader->NUM_RANKS*iniReader->NUM_BANKS,0);
	totalWritesPerBank = vector<uint64_t>(iniReader->NUM_RANKS*iniReader->NUM_BANKS,0);
	totalReadsPerRank = vector<uint64_t>(iniReader->NUM_RANKS,0);
	totalWritesPerRank = vector<uint64_t>(iniReader->NUM_RANKS,0);
	totalReadsPerRank_Receive = vector<uint64_t>(iniReader->NUM_RANKS,0);
	totalWritesPerRank_Receive = vector<uint64_t>(iniReader->NUM_RANKS,0);

	writeDataCountdown.reserve(iniReader->NUM_RANKS);
	writeDataToSend.reserve(iniReader->NUM_RANKS);
  if(iniReader->SystemType==TYPE_DRAM) {
    refreshRank=0;// just put it here, though it's been done on the init list
	  refreshCountdown.reserve(iniReader->NUM_RANKS);
  }else if(iniReader->SystemType==TYPE_NVM) {
    refreshRank=-1;
  }else {
  //TODO print some error message
    exit(-1);
  }


	//Power related packets
	backgroundEnergy = vector <double>(iniReader->NUM_RANKS,0.0);
	burstEnergy = vector <double> (iniReader->NUM_RANKS,0.0);
	actpreEnergy = vector <double> (iniReader->NUM_RANKS,0.0);
	refreshEnergy = vector <double> (iniReader->NUM_RANKS,0.0);

	totalEpochLatency = vector<uint64_t> (iniReader->NUM_RANKS*iniReader->NUM_BANKS,0);
	totalEpochLatency_Write = vector<uint64_t> (iniReader->NUM_RANKS*iniReader->NUM_BANKS,0);

	//staggers when each rank is due for a refresh
	for (size_t i=0;i<iniReader->NUM_RANKS;i++)
	{
		refreshCountdown.push_back((int)((iniReader->REFRESH_PERIOD/iniReader->tCK)/iniReader->NUM_RANKS)*(i+1));
	}
}

//get a bus packet from either data or cmd bus
void MemoryController::receiveFromBus(BusPacket *bpacket)
{
	if (bpacket->busPacketType != DATA)
	{
		ERROR("== Error - Memory Controller received a non-DATA bus packet from rank");
		bpacket->print();
		exit(0);
	}

	if (DEBUG_BUS)
	{
		PRINTN(" -- MC Receiving From Data Bus : ");
		bpacket->print();
	}

	//add to return read data queue
	returnTransaction.push_back(new Transaction(RETURN_DATA, bpacket->physicalAddress, bpacket->data));
	totalReadsPerBank[SEQUENTIAL(bpacket->rank,bpacket->bank)]++;

	// this delete statement saves a mindboggling amount of memory
	delete(bpacket);
}

//sends read data back to the CPU
void MemoryController::returnReadData(const Transaction *trans)
{
	if (parentMemorySystem->ReturnReadData!=NULL)
	{
		(*parentMemorySystem->ReturnReadData)(parentMemorySystem->systemID, trans->address, currentClockCycle);
	}
}

//gives the memory controller a handle on the rank objects
void MemoryController::attachRanks(vector<Rank *> *ranks)
{
	this->ranks = ranks;
}

//memory controller update
void MemoryController::update()
{

	//PRINT(" ------------------------- [" << currentClockCycle << "] -------------------------");

	//update bank states
	for (size_t i=0;i<iniReader->NUM_RANKS;i++)
	{
		for (size_t j=0;j<iniReader->NUM_BANKS;j++)
		{
			if (bankStates[i][j].stateChangeCountdown>0)
			{
				//decrement counters
				bankStates[i][j].stateChangeCountdown--;

				//if counter has reached 0, change state
				if (bankStates[i][j].stateChangeCountdown == 0)
				{
					switch (bankStates[i][j].lastCommand)
					{
						//only these commands have an implicit state change
					case WRITE_P:
					case READ_P:
						bankStates[i][j].currentBankState = Precharging;
						bankStates[i][j].lastCommand = PRECHARGE;
						bankStates[i][j].stateChangeCountdown = iniReader->tRP;
						break;
					case READ:
						if(iniReader->SystemType == TYPE_NVM && iniReader->rowBufferPolicy == ClosePage)
						{
							bankStates[i][j].currentBankState = Idle;
						}
						break;

					case REFRESH:
					case PRECHARGE:
						bankStates[i][j].currentBankState = Idle;
						break;
					default:
						break;
					}
				}
			}
		}
	}


	//check for outgoing command packets and handle countdowns
	if (outgoingCmdPacket != NULL)
	{
		cmdCyclesLeft--;
		if (cmdCyclesLeft == 0) //packet is ready to be received by rank
		{
			(*ranks)[outgoingCmdPacket->rank]->receiveFromBus(outgoingCmdPacket);
			outgoingCmdPacket = NULL;
		}
	}

	//check for outgoing data packets and handle countdowns
	if (outgoingDataPacket != NULL)
	{
		dataCyclesLeft--;
		if (dataCyclesLeft == 0)
		{
			//inform upper levels that a write is done
			if (parentMemorySystem->WriteDataDone!=NULL)
			{
				(*parentMemorySystem->WriteDataDone)(parentMemorySystem->systemID,outgoingDataPacket->physicalAddress, currentClockCycle);
			}

			(*ranks)[outgoingDataPacket->rank]->receiveFromBus(outgoingDataPacket);

			
			
			outgoingDataPacket=NULL;
		}
	}


	//if any outstanding write data needs to be sent
	//and the appropriate amount of time has passed (WL)
	//then send data on bus
	//
	//write data held in fifo vector along with countdowns
	if (writeDataCountdown.size() > 0)
	{
		for (size_t i=0;i<writeDataCountdown.size();i++)
		{
			writeDataCountdown[i]--;
		}

		if (writeDataCountdown[0]==0)
		{
			//send to bus and print debug stuff
			if (DEBUG_BUS)
			{
				PRINTN(" -- MC Issuing On Data Bus    : ");
				writeDataToSend[0]->print();
			}

			// queue up the packet to be sent
			if (outgoingDataPacket != NULL)
			{
				ERROR("== Error - Data Bus Collision");
				exit(-1);
			}

			outgoingDataPacket = writeDataToSend[0];
			dataCyclesLeft = iniReader->BL/2;

			for(size_t i=0; i<pendingWriteTransactions.size(); i++)
			{
				if(outgoingDataPacket->physicalAddress == pendingWriteTransactions[i]->address)
				{
					writeLatency.push_back(currentClockCycle - pendingWriteTransactions[i]->timeAdded);
					totalEpochLatency_Write[SEQUENTIAL(outgoingDataPacket->rank,outgoingDataPacket->bank)] += currentClockCycle - pendingWriteTransactions[i]->timeAdded;
					pendingWriteTransactions.erase(pendingWriteTransactions.begin()+i);
					break;
				}
			}
			
			
			totalTransactions++;
			totalWritesPerBank[SEQUENTIAL(writeDataToSend[0]->rank,writeDataToSend[0]->bank)]++;

			writeDataCountdown.erase(writeDataCountdown.begin());
			writeDataToSend.erase(writeDataToSend.begin());
		}
	}

  if(iniReader->SystemType==TYPE_DRAM)
  {
    //if its time for a refresh issue a refresh
    // else pop from command queue if it's not empty
    if (refreshCountdown[refreshRank]==0)
    {
      commandQueue.needRefresh(refreshRank);
      (*ranks)[refreshRank]->refreshWaiting = true;
      refreshCountdown[refreshRank] =	 iniReader->REFRESH_PERIOD/iniReader->tCK;
      refreshRank++;
      //type conversion, possible risks may occur
      if (refreshRank ==(signed) iniReader->NUM_RANKS)
      {
        refreshRank = 0;
      }
    }
    //if a rank is powered down, make sure we power it up in time for a refresh
    else if (powerDown[refreshRank] && refreshCountdown[refreshRank] <= iniReader->tXP)
    {
      (*ranks)[refreshRank]->refreshWaiting = true;
    }
  }

	//pass a pointer to a poppedBusPacket

	//function returns true if there is something valid in poppedBusPacket
	if (commandQueue.pop(&poppedBusPacket))
	{
		if (poppedBusPacket->busPacketType == WRITE || poppedBusPacket->busPacketType == WRITE_P)
		{

			writeDataToSend.push_back(new BusPacket(DATA, poppedBusPacket->physicalAddress, poppedBusPacket->column,
			                                    poppedBusPacket->row, poppedBusPacket->rank, poppedBusPacket->bank,
			                                    poppedBusPacket->data, dramsim_log));
			writeDataCountdown.push_back(iniReader->WL);

			for(size_t i=0; i<pendingWriteTransactions.size(); i++)
			{
				if(pendingWriteTransactions[i]->address == poppedBusPacket->physicalAddress)
				{
					commandQueueDelay.push_back(currentClockCycle-pendingWriteTransactions[i]->timeAdded);
				}
					
			}
		}

		//
		//update each bank's state based on the command that was just popped out of the command queue
		//
		//for readability's sake
		unsigned rank = poppedBusPacket->rank;
		unsigned bank = poppedBusPacket->bank;
		switch (poppedBusPacket->busPacketType)
		{
			case READ_P:
			case READ:
				//add energy to account for total
				if (DEBUG_POWER)
				{
					PRINT(" ++ Adding Read energy to total energy");
				}
				//burstEnergy[rank] += (iniReader->IDD4R - iniReader->IDD3N) * iniReader->BL/2 * iniReader->NUM_DEVICES;

				burstEnergy[rank] += iniReader->RowBufferReadEnergy * iniReader->JEDEC_DATA_BUS_BITS * iniReader->BL;
				burstNum++;

				 if(iniReader->SystemType==TYPE_DRAM)
				 {
				 	burstNum_dram++;
				 }
				 else
				 {
				 	burstNum_pcm++;
				 }
				
				if (poppedBusPacket->busPacketType == READ_P) 
				{
					//Don't bother setting next read or write times because the bank is no longer active
					//bankStates[rank][bank].currentBankState = Idle;
					bankStates[rank][bank].nextActivate = max(currentClockCycle + READ_AUTOPRE_DELAY,
							bankStates[rank][bank].nextActivate);
					bankStates[rank][bank].lastCommand = READ_P;
					bankStates[rank][bank].stateChangeCountdown = READ_TO_PRE_DELAY;

					actpreEnergy[rank] += iniReader->ArrayWriteEnergy * iniReader->NUM_COLS * iniReader->JEDEC_DATA_BUS_BITS;
					actpreNum++;				
					 if(iniReader->SystemType==TYPE_DRAM)
					 {
					 	actpreNum_dram++;
					 }
					 else
					 {
					 	actpreNum_pcm++;
					 }
				}
				else if (poppedBusPacket->busPacketType == READ)
				{
					bankStates[rank][bank].lastCommand = READ;
					
					if(iniReader->SystemType == TYPE_NVM && iniReader->rowBufferPolicy == ClosePage)
					{
						bankStates[rank][bank].nextActivate = iniReader->AL+iniReader->tRTP;
						bankStates[rank][bank].stateChangeCountdown = iniReader->AL+iniReader->BL/2;
					}
					else
					{
						bankStates[rank][bank].nextPrecharge = max(currentClockCycle + READ_TO_PRE_DELAY,
							bankStates[rank][bank].nextPrecharge);
					}
					
					//bankStates[rank][bank].lastCommand = READ;
					

				}

				for (size_t i=0;i<iniReader->NUM_RANKS;i++)
				{
					for (size_t j=0;j<iniReader->NUM_BANKS;j++)
					{
						if (i!=poppedBusPacket->rank)
						{
							//check to make sure it is active before trying to set (save's time?)
							if (bankStates[i][j].currentBankState == RowActive)
							{
								bankStates[i][j].nextRead = max(currentClockCycle + iniReader->BL/2 + iniReader->tRTRS, bankStates[i][j].nextRead);
								bankStates[i][j].nextWrite = max(currentClockCycle + READ_TO_WRITE_DELAY,
										bankStates[i][j].nextWrite);
							}
						}
						else
						{
							bankStates[i][j].nextRead = max(currentClockCycle + max(iniReader->tCCD, iniReader->BL/2), bankStates[i][j].nextRead);
							bankStates[i][j].nextWrite = max(currentClockCycle + READ_TO_WRITE_DELAY,
									bankStates[i][j].nextWrite);
						}
					}
				}

				if (poppedBusPacket->busPacketType == READ_P)
				{
					//set read and write to nextActivate so the state table will prevent a read or write
					//  being issued (in cq.isIssuable())before the bank state has been changed because of the
					//  auto-precharge associated with this command
					bankStates[rank][bank].nextRead = bankStates[rank][bank].nextActivate;
					bankStates[rank][bank].nextWrite = bankStates[rank][bank].nextActivate;
				}
				

				break;
			case WRITE_P:
			case WRITE:
				if (poppedBusPacket->busPacketType == WRITE_P) 
				{
					bankStates[rank][bank].nextActivate = max(currentClockCycle + WRITE_AUTOPRE_DELAY,
							bankStates[rank][bank].nextActivate);
					bankStates[rank][bank].lastCommand = WRITE_P;
					bankStates[rank][bank].stateChangeCountdown = WRITE_TO_PRE_DELAY;

					actpreEnergy[rank] += iniReader->ArrayWriteEnergy * iniReader->NUM_COLS * iniReader->JEDEC_DATA_BUS_BITS;
					actpreNum++;
					
					 if(iniReader->SystemType==TYPE_DRAM)
					 {
					 	actpreNum_dram++;
					 }
					 else
					 {
					 	actpreNum_pcm++;
					 }
				}
				else if (poppedBusPacket->busPacketType == WRITE)
				{
					bankStates[rank][bank].nextPrecharge = max(currentClockCycle + WRITE_TO_PRE_DELAY,
							bankStates[rank][bank].nextPrecharge);
					bankStates[rank][bank].lastCommand = WRITE;
				}


				//add energy to account for total
				if (DEBUG_POWER)
				{
					PRINT(" ++ Adding Write energy to total energy");
				}
				//burstEnergy[rank] += (iniReader->IDD4W - iniReader->IDD3N) * iniReader->BL/2 * iniReader->NUM_DEVICES;
				burstEnergy[rank] += iniReader->RowBufferWriteEnergy * iniReader->JEDEC_DATA_BUS_BITS * iniReader->BL;
				burstNum++;

				 if(iniReader->SystemType==TYPE_DRAM)
				 {
				 	burstNum_dram++;
				 }
				 else
				 {
				 	burstNum_pcm++;
				 }
				 

				for (size_t i=0;i<iniReader->NUM_RANKS;i++)
				{
					for (size_t j=0;j<iniReader->NUM_BANKS;j++)
					{
						if (i!=poppedBusPacket->rank)
						{
							if (bankStates[i][j].currentBankState == RowActive)
							{
								bankStates[i][j].nextWrite = max(currentClockCycle + iniReader->BL/2 + iniReader->tRTRS, bankStates[i][j].nextWrite);
								bankStates[i][j].nextRead = max(currentClockCycle + WRITE_TO_READ_DELAY_R,
										bankStates[i][j].nextRead);
							}
						}
						else
						{
							bankStates[i][j].nextWrite = max(currentClockCycle + max(iniReader->BL/2, iniReader->tCCD), bankStates[i][j].nextWrite);
							bankStates[i][j].nextRead = max(currentClockCycle + WRITE_TO_READ_DELAY_B,
									bankStates[i][j].nextRead);
						}
					}
				}

				//set read and write to nextActivate so the state table will prevent a read or write
				//  being issued (in cq.isIssuable())before the bank state has been changed because of the
				//  auto-precharge associated with this command
				if (poppedBusPacket->busPacketType == WRITE_P)
				{
					bankStates[rank][bank].nextRead = bankStates[rank][bank].nextActivate;
					bankStates[rank][bank].nextWrite = bankStates[rank][bank].nextActivate;
				}

				break;
			case ACTIVATE:
				//add energy to account for total
				if (DEBUG_POWER)
				{
					PRINT(" ++ Adding Activate and Precharge energy to total energy");
				}
				//actpreEnergy[rank] += ((iniReader->IDD0 * iniReader->tRC) - ((iniReader->IDD3N * iniReader->tRAS) + (iniReader->IDD2N * (iniReader->tRC - iniReader->tRAS)))) * iniReader->NUM_DEVICES;
				actpreEnergy[rank] += iniReader->ArrayReadEnergy * iniReader->NUM_COLS * iniReader->JEDEC_DATA_BUS_BITS;
				actpreNum++;
				
				 if(iniReader->SystemType==TYPE_DRAM)
				 {
				 	actpreNum_dram++;
				 }
				 else
				 {
				 	actpreNum_pcm++;
				 }

				
				bankStates[rank][bank].currentBankState = RowActive;
				bankStates[rank][bank].lastCommand = ACTIVATE;
				bankStates[rank][bank].openRowAddress = poppedBusPacket->row;
				bankStates[rank][bank].nextActivate = max(currentClockCycle + iniReader->tRC, bankStates[rank][bank].nextActivate);
				bankStates[rank][bank].nextPrecharge = max(currentClockCycle + iniReader->tRAS, bankStates[rank][bank].nextPrecharge);

				//if we are using posted-CAS, the next column access can be sooner than normal operation

				bankStates[rank][bank].nextRead = max(currentClockCycle + (iniReader->tRCD-iniReader->AL), bankStates[rank][bank].nextRead);
				bankStates[rank][bank].nextWrite = max(currentClockCycle + (iniReader->tRCD-iniReader->AL), bankStates[rank][bank].nextWrite);

				for (size_t i=0;i<iniReader->NUM_BANKS;i++)
				{
					if (i!=poppedBusPacket->bank)
					{
						bankStates[rank][i].nextActivate = max(currentClockCycle + iniReader->tRRD, bankStates[rank][i].nextActivate);
					}
				}

				break;
			case PRECHARGE:
				bankStates[rank][bank].currentBankState = Precharging;
				bankStates[rank][bank].lastCommand = PRECHARGE;
				bankStates[rank][bank].stateChangeCountdown = iniReader->tRP;
				bankStates[rank][bank].nextActivate = max(currentClockCycle + iniReader->tRP, bankStates[rank][bank].nextActivate);

				actpreEnergy[rank] += iniReader->ArrayWriteEnergy * iniReader->NUM_COLS * iniReader->JEDEC_DATA_BUS_BITS;
				actpreNum++;
				 if(iniReader->SystemType==TYPE_DRAM)
				 {
				 	actpreNum_dram++;
				 }
				 else
				 {
				 	actpreNum_pcm++;
				 }
				
				break;
			case REFRESH:
				//add energy to account for total
				if (DEBUG_POWER)
				{
					PRINT(" ++ Adding Refresh energy to total energy");
				}
				//refreshEnergy[rank] += (iniReader->IDD5 - iniReader->IDD3N) * iniReader->tRFC *iniReader->NUM_DEVICES;
				
				refreshEnergy[rank] += (iniReader->ArrayReadEnergy + iniReader->ArrayWriteEnergy)*iniReader->NUM_COLS*iniReader->DEVICE_WIDTH * 8 * iniReader->NUM_BANKS *(iniReader->tRFC/iniReader->tRC);
				refreshNum++;
				 if(iniReader->SystemType==TYPE_DRAM)
				 {
				 	refreshNum_dram++;
				 }
				 else
				 {
				 	refreshNum_pcm++;
				 }
				
				
				for (size_t i=0;i<iniReader->NUM_BANKS;i++)
				{
					bankStates[rank][i].nextActivate = currentClockCycle + iniReader->tRFC;
					bankStates[rank][i].currentBankState = Refreshing;
					bankStates[rank][i].lastCommand = REFRESH;
					bankStates[rank][i].stateChangeCountdown = iniReader->tRFC;
				}

				break;
			default:
				ERROR("== Error - Popped a command we shouldn't have of type : " << poppedBusPacket->busPacketType);
				exit(0);
		}

		//issue on bus and print debug
		if (DEBUG_BUS)
		{
			PRINTN(" -- MC Issuing On Command Bus : ");
			poppedBusPacket->print();
		}

		//check for collision on bus
		if (outgoingCmdPacket != NULL)
		{
			ERROR("== Error - Command Bus Collision");
			exit(-1);
		}
		outgoingCmdPacket = poppedBusPacket;
		cmdCyclesLeft = iniReader->tCMD;

	}

	for (size_t i=0;i<transactionQueue.size();i++)
	{
		//pop off top transaction from queue
		//
		//	assuming simple scheduling at the moment
		//	will eventually add policies here
		Transaction *transaction = transactionQueue[i];

		//map address to rank,bank,row,col
		unsigned newTransactionChan, newTransactionRank, newTransactionBank, newTransactionRow, newTransactionColumn;

		// pass these in as references so they get set by the addressMapping function
		addressMapping(transaction->address, newTransactionChan, newTransactionRank, newTransactionBank, newTransactionRow, newTransactionColumn,allIniReaders);

		//if we have room, break up the transaction into the appropriate commands
		//and add them to the command queue
		if (commandQueue.hasRoomFor(2, newTransactionRank, newTransactionBank))
		{

			if (transaction->transactionType == DATA_READ) 
			{
				totalReadsPerRank_Receive[newTransactionRank]++;
			}
			else
			{
				totalWritesPerRank_Receive[newTransactionRank]++;
			}
			if (DEBUG_ADDR_MAP) 
			{
				PRINTN("== New Transaction - Mapping Address [0x" << hex << transaction->address << dec << "]");
				if (transaction->transactionType == DATA_READ) 
				{
					PRINT(" (Read)");
				}
				else
				{
					PRINT(" (Write)");
				}
				PRINT("  Rank : " << newTransactionRank);
				PRINT("  Bank : " << newTransactionBank);
				PRINT("  Row  : " << newTransactionRow);
				PRINT("  Col  : " << newTransactionColumn);
			}

			
			

			//now that we know there is room in the command queue, we can remove from the transaction queue
			transactionQueue.erase(transactionQueue.begin()+i);

			//create activate command to the row we just translated
			BusPacket *ACTcommand = new BusPacket(ACTIVATE, transaction->address,
					newTransactionColumn, newTransactionRow, newTransactionRank,
					newTransactionBank, 0, dramsim_log);

			//create read or write command and enqueue it
			BusPacketType bpType = transaction->getBusPacketType(parentMemorySystem->systemID,iniReader);
			BusPacket *command = new BusPacket(bpType, transaction->address,
					newTransactionColumn, newTransactionRow, newTransactionRank,
					newTransactionBank, transaction->data, dramsim_log);
			


			commandQueue.enqueue(ACTcommand);
			commandQueue.enqueue(command);

			// If we have a read, save the transaction so when the data comes back
			// in a bus packet, we can staple it back into a transaction and return it
			if (transaction->transactionType == DATA_READ)
			{
				pendingReadTransactions.push_back(transaction);
			}
			else if(transaction->transactionType == DATA_WRITE)
			{
				
				transactionQueueDelay.push_back(currentClockCycle-transaction->timeAdded);
				//transaction->timeAdded = currentClockCycle;
				pendingWriteTransactions.push_back(transaction);
			}
			else
			{
				// just delete the transaction now that it's a buspacket
				delete transaction; 
			}
			/* only allow one transaction to be scheduled per cycle -- this should
			 * be a reasonable assumption considering how much logic would be
			 * required to schedule multiple entries per cycle (parallel data
			 * lines, switching logic, decision logic)
			 */
			break;
		}
		else // no room, do nothing this cycle
		{
			//PRINT( "== Warning - No room in command queue" << endl;
		}
	}


	//calculate power
	//  this is done on a per-rank basis, since power characterization is done per device (not per bank)
	for (size_t i=0;i<iniReader->NUM_RANKS;i++)
	{
		if (iniReader->USE_LOW_POWER)
		{
			//if there are no commands in the queue and that particular rank is not waiting for a refresh...
			if (commandQueue.isEmpty(i) && !(*ranks)[i]->refreshWaiting)
			{
				//check to make sure all banks are idle
				bool allIdle = true;
				for (size_t j=0;j<iniReader->NUM_BANKS;j++)
				{
					if (bankStates[i][j].currentBankState != Idle)
					{
						allIdle = false;
						break;
					}
				}

				//if they ARE all idle, put in power down mode and set appropriate fields
				if (allIdle)
				{
					powerDown[i] = true;
					(*ranks)[i]->powerDown();
					for (size_t j=0;j<iniReader->NUM_BANKS;j++)
					{
						bankStates[i][j].currentBankState = PowerDown;
						bankStates[i][j].nextPowerUp = currentClockCycle + iniReader->tCKE;
					}
				}
			}
			//if there IS something in the queue or there IS a refresh waiting (and we can power up), do it
			else if (currentClockCycle >= bankStates[i][0].nextPowerUp && powerDown[i]) //use 0 since theyre all the same
			{
				powerDown[i] = false;
				(*ranks)[i]->powerUp();
				for (size_t j=0;j<iniReader->NUM_BANKS;j++)
				{
					bankStates[i][j].currentBankState = Idle;
					bankStates[i][j].nextActivate = currentClockCycle + iniReader->tXP;
				}
			}
		}

		//check for open bank
		bool bankOpen = false;
		for (size_t j=0;j<iniReader->NUM_BANKS;j++)
		{
			if (bankStates[i][j].currentBankState == Refreshing ||
			        bankStates[i][j].currentBankState == RowActive)
			{
				bankOpen = true;
				break;
			}
		}

		//background power is dependent on whether or not a bank is open or not
		if (bankOpen)
		{
			if (DEBUG_POWER)
			{
				PRINT(" ++ Adding IDD3N to total energy [from rank "<< i <<"]");
			}
			backgroundEnergy[i] += iniReader->IDD3N * iniReader->NUM_DEVICES;
		}
		else
		{
			//if we're in power-down mode, use the correct current
			if (powerDown[i])
			{
				if (DEBUG_POWER)
				{
					PRINT(" ++ Adding IDD2P to total energy [from rank " << i << "]");
				}
				backgroundEnergy[i] += iniReader->IDD2P * iniReader->NUM_DEVICES;
			}
			else
			{
				if (DEBUG_POWER)
				{
					PRINT(" ++ Adding IDD2N to total energy [from rank " << i << "]");
				}
				backgroundEnergy[i] += iniReader->IDD2N * iniReader->NUM_DEVICES;
			}
		}
	}

	//check for outstanding data to return to the CPU
	if (returnTransaction.size()>0)
	{
		if (DEBUG_BUS)
		{
			PRINTN(" -- MC Issuing to CPU bus : " << *returnTransaction[0]);
		}
		totalTransactions++;

		bool foundMatch=false;
		//find the pending read transaction to calculate latency
		for (size_t i=0;i<pendingReadTransactions.size();i++)
		{
			if (pendingReadTransactions[i]->address == returnTransaction[0]->address)
			{
				//if(currentClockCycle - pendingReadTransactions[i]->timeAdded > 2000)
				//	{
				//		pendingReadTransactions[i]->print();
				//		exit(0);
				//	}
				unsigned chan,rank,bank,row,col;
				addressMapping(returnTransaction[0]->address,chan,rank,bank,row,col,allIniReaders);
				insertHistogram(currentClockCycle-pendingReadTransactions[i]->timeAdded,rank,bank);
				/*
				if(iniReader->SystemType == TYPE_DRAM)
				{
					ofs_dram<<"currentClockCycle: "<<currentClockCycle<<"; "<<currentClockCycle-pendingReadTransactions[i]->timeAdded<<endl;
				}else{
						ofs_pcm<<"currentClockCycle: "<<currentClockCycle<<"; "<<currentClockCycle-pendingReadTransactions[i]->timeAdded<<endl;
					}
				*/
				
				//return latency
				returnReadData(pendingReadTransactions[i]);

				delete pendingReadTransactions[i];
				pendingReadTransactions.erase(pendingReadTransactions.begin()+i);
				foundMatch=true; 
				break;
			}
		}
		if (!foundMatch)
		{
			ERROR("Can't find a matching transaction for 0x"<<hex<<returnTransaction[0]->address<<dec);
			abort(); 
		}
		delete returnTransaction[0];
		returnTransaction.erase(returnTransaction.begin());
	}

	//decrement refresh counters
	for (size_t i=0;i<iniReader->NUM_RANKS;i++)
	{
		refreshCountdown[i]--;
	}

	//
	//print debug
	//
	if (DEBUG_TRANS_Q)
	{
		PRINT("== Printing transaction queue");
		for (size_t i=0;i<transactionQueue.size();i++)
		{
			PRINTN("  " << i << "] "<< *transactionQueue[i]);
		}
	}

	if (DEBUG_BANKSTATE)
	{
		//TODO: move this to BankState.cpp
		PRINT("== Printing bank states (According to MC)");
		for (size_t i=0;i<iniReader->NUM_RANKS;i++)
		{
			for (size_t j=0;j<iniReader->NUM_BANKS;j++)
			{
				if (bankStates[i][j].currentBankState == RowActive)
				{
					PRINTN("[" << bankStates[i][j].openRowAddress << "] ");
				}
				else if (bankStates[i][j].currentBankState == Idle)
				{
					PRINTN("[idle] ");
				}
				else if (bankStates[i][j].currentBankState == Precharging)
				{
					PRINTN("[pre] ");
				}
				else if (bankStates[i][j].currentBankState == Refreshing)
				{
					PRINTN("[ref] ");
				}
				else if (bankStates[i][j].currentBankState == PowerDown)
				{
					PRINTN("[lowp] ");
				}
			}
			PRINT(""); // effectively just cout<<endl;
		}
	}

	if (DEBUG_CMD_Q)
	{
		commandQueue.print();
	}

	commandQueue.step();

}

bool MemoryController::WillAcceptTransaction()
{
	return transactionQueue.size() < iniReader->TRANS_QUEUE_DEPTH;
}

//allows outside source to make request of memory system
bool MemoryController::addTransaction(Transaction *trans)
{
	if (WillAcceptTransaction())
	{
		trans->timeAdded = currentClockCycle;
		transactionQueue.push_back(trans);
		if(iniReader->SystemType == TYPE_DRAM)
		{
			if(trans->transactionType == DATA_WRITE)
			{
				dram_write++;
			}
			else
			{
				dram_read++;
			}
		}
		else
		{
			if(trans->transactionType == DATA_WRITE)
			{
				pcm_write++;
			}
			else
			{
				pcm_read++;
			}
		
		}
		return true;
	}
	else 
	{
		return false;
	}
}

void MemoryController::resetStats()
{
	for (size_t i=0; i<iniReader->NUM_RANKS; i++)
	{
		for (size_t j=0; j<iniReader->NUM_BANKS; j++)
		{
			//XXX: this means the bank list won't be printed for partial epochs
			grandTotalBankAccesses[SEQUENTIAL(i,j)] += totalReadsPerBank[SEQUENTIAL(i,j)] + totalWritesPerBank[SEQUENTIAL(i,j)];
			totalReadsPerBank[SEQUENTIAL(i,j)] = 0;
			totalWritesPerBank[SEQUENTIAL(i,j)] = 0;
			totalEpochLatency[SEQUENTIAL(i,j)] = 0;
		}

		burstEnergy[i] = 0;
		actpreEnergy[i] = 0;
		refreshEnergy[i] = 0;
		backgroundEnergy[i] = 0;
		totalReadsPerRank[i] = 0;
		totalWritesPerRank[i] = 0;
	}
}
//prints statistics at the end of an epoch or  simulation
void MemoryController::printStats(bool finalStats)
{
	unsigned myChannel = parentMemorySystem->systemID;

	//if we are not at the end of the epoch, make sure to adjust for the actual number of cycles elapsed

	//uint64_t cyclesElapsed = (currentClockCycle % iniReader->EPOCH_LENGTH == 0) ? iniReader->EPOCH_LENGTH : currentClockCycle % iniReader->EPOCH_LENGTH;
	uint64_t cyclesElapsed = currentClockCycle;
	unsigned bytesPerTransaction = (iniReader->JEDEC_DATA_BUS_BITS*iniReader->BL)/8;
	uint64_t totalBytesTransferred = totalTransactions * bytesPerTransaction;
	double secondsThisEpoch = (double)cyclesElapsed * iniReader->tCK * 1E-9;
	double powerDeno =  (double)cyclesElapsed*iniReader->tCK * 1E3;

	// only per rank
	vector<double> backgroundPower = vector<double>(iniReader->NUM_RANKS,0.0);
	vector<double> burstPower = vector<double>(iniReader->NUM_RANKS,0.0);
	vector<double> refreshPower = vector<double>(iniReader->NUM_RANKS,0.0);
	vector<double> actprePower = vector<double>(iniReader->NUM_RANKS,0.0);
	vector<double> averagePower = vector<double>(iniReader->NUM_RANKS,0.0);

	// per bank variables
	vector<double> averageLatency = vector<double>(iniReader->NUM_RANKS*iniReader->NUM_BANKS,0.0);
	vector<double> bandwidth = vector<double>(iniReader->NUM_RANKS*iniReader->NUM_BANKS,0.0);

	double totalBandwidth=0.0;
	uint64_t totalChannelEpochLatency=0;
	uint64_t totalReadsPerChannel=0;
	uint64_t totalWritesPerChannel=0; 
	double writesBandwidthPerChannel=0.0;
	double readsBandwidthPerChannel=0.0;
	//double totalEnergyPerChannel=0.0;
	double totalActpreEnergyPerChannel=0.0;
	double totalBurstEnergyPerChennel=0.0;
	double totalRefreshEnergyPerChannel=0.0;
	
	for (size_t i=0;i<iniReader->NUM_RANKS;i++)
	{
		uint64_t totalRankEpochLatency_Write=0;
		for (size_t j=0; j<iniReader->NUM_BANKS; j++)
		{
			bandwidth[SEQUENTIAL(i,j)] = (((double)(totalReadsPerBank[SEQUENTIAL(i,j)]+totalWritesPerBank[SEQUENTIAL(i,j)]) * (double)bytesPerTransaction)/(1024.0*1024.0*1024.0)) / secondsThisEpoch;
			averageLatency[SEQUENTIAL(i,j)] = ((float)totalEpochLatency[SEQUENTIAL(i,j)] / (float)(totalReadsPerBank[SEQUENTIAL(i,j)])) * parentMemorySystem->iniReader->tCK;
			readsBandwidthPerChannel += (((double)(totalReadsPerBank[SEQUENTIAL(i,j)]) * (double)bytesPerTransaction)/(1024.0*1024.0*1024.0)) / secondsThisEpoch;
			writesBandwidthPerChannel += (((double)(totalWritesPerBank[SEQUENTIAL(i,j)]) * (double)bytesPerTransaction)/(1024.0*1024.0*1024.0)) / secondsThisEpoch;
			
			totalBandwidth+=bandwidth[SEQUENTIAL(i,j)];
			totalChannelEpochLatency+=totalEpochLatency[SEQUENTIAL(i,j)];
			
			totalRankEpochLatency_Write+=totalEpochLatency_Write[SEQUENTIAL(i,j)];
			
			totalReadsPerChannel+=totalReadsPerBank[SEQUENTIAL(i,j)];
			totalWritesPerChannel+=totalWritesPerBank[SEQUENTIAL(i,j)];
			totalReadsPerRank[i] += totalReadsPerBank[SEQUENTIAL(i,j)];
			totalWritesPerRank[i] += totalWritesPerBank[SEQUENTIAL(i,j)];
			//csvOut.getOutputStream()<<"totalWritesPerBank["<<i<<"]["<<j<<"]: "<<totalWritesPerBank[SEQUENTIAL(i,j)]<<endl;
		}
		//csvOut.getOutputStream()<<"WriteLatencyPerRank["<<i<<"]: "<<(float)totalRankEpochLatency_Write/(float)totalWritesPerRank[i] * parentMemorySystem->iniReader->tCK<<endl;
		csvOut.getOutputStream()<<"totalReadsPerRank_R_C["<<i<<"]: "<<totalReadsPerRank_Receive[i]<<", "<<totalReadsPerRank[i]<<"    totalWritesPerRank_R_C["<<i<<"]: "<<totalWritesPerRank_Receive[i]<<", "<<totalWritesPerRank[i]<<endl;

	}
#ifdef LOG_OUTPUT
	dramsim_log.precision(3);
	dramsim_log.setf(ios::fixed,ios::floatfield);
#else
	cout.precision(3);
	cout.setf(ios::fixed,ios::floatfield);
#endif

	PRINT( " =======================================================" );
	PRINT( " ============== Printing Statistics [id:"<<parentMemorySystem->systemID<<"]==============" );
	PRINTN( "   Total Return Transactions : " << totalTransactions );
	PRINT( " ("<<totalBytesTransferred <<" bytes) aggregate average bandwidth "<<totalBandwidth<<"GB/s");
/*
	if (VIS_FILE_OUTPUT)
	{
		csvOut.getOutputStream()<<"===========channel:  "<<myChannel<<"============"<<endl;
		csvOut.getOutputStream()<<"Background_Power, ACT_PRE_Power, Burst_Power, Refresh_Power, Rank_Aggregate_Bandwidth, Rank_Average_Bandwidth"<<endl;
	}
*/
	//double totalAggregateBandwidth = 0.0;	
	for (size_t r=0;r<iniReader->NUM_RANKS;r++)
	{

		PRINT( "      -Rank   "<<r<<" : ");
		PRINTN( "        -Reads  : " << totalReadsPerRank[r]);
		PRINT( " ("<<totalReadsPerRank[r] * bytesPerTransaction<<" bytes)");
		PRINTN( "        -Writes : " << totalWritesPerRank[r]);
		PRINT( " ("<<totalWritesPerRank[r] * bytesPerTransaction<<" bytes)");
		/*
		for (size_t j=0;j<iniReader->NUM_BANKS;j++)
		{
			PRINT( "        -Bandwidth / Latency  (Bank " <<j<<"): " <<bandwidth[SEQUENTIAL(r,j)] << " GB/s\t\t" <<averageLatency[SEQUENTIAL(r,j)] << " ns");
		}
		*/
		
		// factor of 1000 at the end is to account for the fact that totalEnergy is accumulated in mJ since IDD values are given in mA
		//backgroundPower[r] = ((double)backgroundEnergy[r] / (double)(cyclesElapsed)) * iniReader->Vdd / 1000.0;
		//burstPower[r] = ((double)burstEnergy[r] / (double)(cyclesElapsed)) * iniReader->Vdd / 1000.0;
		//refreshPower[r] = ((double) refreshEnergy[r] / (double)(cyclesElapsed)) * iniReader->Vdd / 1000.0;
		//actprePower[r] = ((double)actpreEnergy[r] / (double)(cyclesElapsed)) * iniReader->Vdd / 1000.0;
		//averagePower[r] = ((backgroundEnergy[r] + burstEnergy[r] + refreshEnergy[r] + actpreEnergy[r]) / (double)cyclesElapsed) * iniReader->Vdd / 1000.0;

		burstPower[r] = ((double)burstEnergy[r] / powerDeno);
		actprePower[r] = ((double)actpreEnergy[r] / powerDeno);
		refreshPower[r] = ((double) refreshEnergy[r] / powerDeno);
		averagePower[r] = ((burstEnergy[r] + refreshEnergy[r] + actpreEnergy[r]) / powerDeno);

		totalActpreEnergyPerChannel +=  actpreEnergy[r];
		totalBurstEnergyPerChennel += burstEnergy[r];
		totalRefreshEnergyPerChannel += refreshEnergy[r];
		//totalEnergyPerChannel += (burstEnergy[r] + refreshEnergy[r] + actpreEnergy[r]);
		

		if ((*parentMemorySystem->ReportPower)!=NULL)
		{
			(*parentMemorySystem->ReportPower)(backgroundPower[r],burstPower[r],refreshPower[r],actprePower[r]);
		}

		PRINT( " == Power Data for Rank        " << r );
		PRINT( "   Average Power (watts)     : " << averagePower[r] );
		//PRINT( "     -Background (watts)     : " << backgroundPower[r] );
		PRINT( "     -Act/Pre    (watts)     : " << actprePower[r] );
		PRINT( "     -Burst      (watts)     : " << burstPower[r]);
		PRINT( "     -Refresh    (watts)     : " << refreshPower[r] );
		
			
		csvOut.getOutputStream()<<"totalPowerPerRank: "<< averagePower[r]<<"\t"<<"actprePower: " << actprePower[r]<<"\t"<<"burstPower: "<<burstPower[r]<<"\t"<<"refreshPower: "<<refreshPower[r]<<endl;
		csvOut.getOutputStream()<<"totalEnergyPerRank: "<<(burstEnergy[r] + refreshEnergy[r] + actpreEnergy[r])/(1E9)<<"\t"<<"actpreEnergy: "<<actpreEnergy[r]/(1E9)<<"\t"<<"burstEnergy: "<<burstEnergy[r]/(1E9)<<"\t"<<"refreshEnergy: "<<refreshEnergy[r]/(1E9)<<endl;
		//csvOut.getOutputStream()<<"actprePower: " << actprePower[r]<<endl;
		//csvOut.getOutputStream()<<"burstPower: " << burstPower[r]<<endl;
		//csvOut.getOutputStream()<<"refreshPower: " << refreshPower[r]<<endl;

	}


	if(VIS_FILE_OUTPUT)
	{
		csvOut.getOutputStream()<<"bandWidth_channel["<<myChannel<<"]: "<<totalBandwidth<<endl;
		csvOut.getOutputStream()<<"readsBandwidth["<<myChannel<<"]: "<<readsBandwidthPerChannel<<endl;
		csvOut.getOutputStream()<<"writesBandwidth["<<myChannel<<"]: "<<writesBandwidthPerChannel<<endl;
		csvOut.getOutputStream()<<"latency_read_channel["<<myChannel<<"]: "<<((float)totalChannelEpochLatency/ (float)(totalReadsPerChannel)) * parentMemorySystem->iniReader->tCK<<endl;

		csvOut.getOutputStream()<<"totalReadsPerChannel["<<myChannel<<"]: "<<totalReadsPerChannel<<endl;
		csvOut.getOutputStream()<<"totalWritesPerChannel["<<myChannel<<"]: "<<totalWritesPerChannel<<endl;

		csvOut.getOutputStream()<<"totalPowerPerChannel["<<myChannel<<"]: "<<(totalBurstEnergyPerChennel+totalActpreEnergyPerChannel+totalActpreEnergyPerChannel)/powerDeno<<endl;
		csvOut.getOutputStream()<<"totalEnergyPerChannel["<<myChannel<<"]: "<<totalBurstEnergyPerChennel+totalActpreEnergyPerChannel+totalActpreEnergyPerChannel<<endl<<endl;
		


		totalReadsBandwidth_MS += readsBandwidthPerChannel;
		totalWritesBandwidth_MS += writesBandwidthPerChannel;
		totalBandwidth_MS += totalBandwidth;
		totalEpochLatency_MS +=totalChannelEpochLatency;
		totalReads_MS += totalReadsPerChannel;
		totalWrites_MS += totalWritesPerChannel;
		totalBurstEnergy_MS += totalBurstEnergyPerChennel;
		totalActpreEnergy_MS += totalActpreEnergyPerChannel;
		totalRefreshEnergy_MS += totalRefreshEnergyPerChannel;
		//totalEnergy_MS += totalEnergyPerChannel;
	}
		

	// only print the latency histogram at the end of the simulation since it clogs the output too much to print every epoch
	/*
	if (finalStats)
	{
		PRINT( " ---  Latency list ("<<latencies.size()<<")");
		PRINT( "       [lat] : #");
		if (VIS_FILE_OUTPUT)
		{
			//csvOut.getOutputStream() << "!!HISTOGRAM_DATA"<<endl;
		}

		map<unsigned,unsigned>::iterator it; //
		for (it=latencies.begin(); it!=latencies.end(); it++)
		{
			PRINT( "       ["<< it->first <<"-"<<it->first+(HISTOGRAM_BIN_SIZE-1)<<"] : "<< it->second );
			if (VIS_FILE_OUTPUT)
			{
				//csvOut.getOutputStream() << it->first <<"="<< it->second << endl;
			}
		}
		if (currentClockCycle % iniReader->EPOCH_LENGTH == 0)
		{
			PRINT( " --- Grand Total Bank usage list");
			for (size_t i=0;i<iniReader->NUM_RANKS;i++)
			{
				PRINT("Rank "<<i<<":"); 
				for (size_t j=0;j<iniReader->NUM_BANKS;j++)
				{
					PRINT( "  b"<<j<<": "<<grandTotalBankAccesses[SEQUENTIAL(i,j)]);
				}
			}
		}

	}
	*/


	PRINT(endl<< " == Pending Transactions : "<<pendingReadTransactions.size()<<" ("<<currentClockCycle<<")==");
	/*
	for(size_t i=0;i<pendingReadTransactions.size();i++)
		{
			PRINT( i << "] I've been waiting for "<<currentClockCycle-pendingReadTransactions[i].timeAdded<<endl;
		}
	*/
#ifdef LOG_OUTPUT
	dramsim_log.flush();
#endif

	resetStats();

}

MemoryController::~MemoryController()
{
	//ERROR("MEMORY CONTROLLER DESTRUCTOR");
	//abort();
	for (size_t i=0; i<pendingReadTransactions.size(); i++)
	{
		delete pendingReadTransactions[i];
	}

	for (size_t i=0; i<pendingWriteTransactions.size(); i++)
	{
		delete pendingWriteTransactions[i];
	}
	
	for (size_t i=0; i<returnTransaction.size(); i++)
	{
		delete returnTransaction[i];
	}

	if(VIS_FILE_OUTPUT && iniReader->SystemType==TYPE_DRAM)
	{

		uint64_t totalWriteLatency_MS = 0;
		for(size_t i=0; i<writeLatency.size(); i++)
		{
			totalWriteLatency_MS += writeLatency[i];
		}
		
		csvOut.getOutputStream()<<"bandWidth_MS: "<<totalBandwidth_MS<<endl;
		csvOut.getOutputStream()<<"totalReadsBandwidth_MS: "<<totalReadsBandwidth_MS<<endl;
		csvOut.getOutputStream()<<"totalWritesBandwidth_MS: "<<totalWritesBandwidth_MS<<endl;
		csvOut.getOutputStream()<<"latency_MS_Read: "<<((float)totalEpochLatency_MS/ (float)(totalReads_MS)) * parentMemorySystem->iniReader->tCK<<endl;
		csvOut.getOutputStream()<<"latency_MS_Write: "<<((float)totalWriteLatency_MS/ (float)(totalWrites_MS)) * parentMemorySystem->iniReader->tCK<<endl;

		
		csvOut.getOutputStream()<<"totalReads_MS: "<<totalReads_MS<<endl;
		csvOut.getOutputStream()<<"totalWrites_MS: "<<totalWrites_MS<<endl;
	
		csvOut.getOutputStream()<<"currentClockCycle: "<<currentClockCycle<<endl;
		csvOut.getOutputStream()<<"cpuCycle: "<<cpuCycle<<endl;
		csvOut.getOutputStream()<<"trasactionCycle: "<<timerActual<<endl;

		csvOut.getOutputStream()<<"dram: "<<dram_read+dram_write<<endl;
		csvOut.getOutputStream()<<"pcm: "<<pcm_read+pcm_write<<endl;
		csvOut.getOutputStream()<<"dram_read: "<<dram_read<<endl;
		csvOut.getOutputStream()<<"dram_write: "<<dram_write<<endl;
		csvOut.getOutputStream()<<"pcm_read: "<<pcm_read<<endl;
		csvOut.getOutputStream()<<"pcm_write: "<<pcm_write<<endl;

		csvOut.getOutputStream()<<"rowBufferHitCount_dram: "<<rowBufferHitCount_dram<<endl;
		csvOut.getOutputStream()<<"rowBufferHitCount_pcm: "<<rowBufferHitCount_pcm<<endl;

		csvOut.getOutputStream()<<"actpreNum: "<<actpreNum<<endl;
		csvOut.getOutputStream()<<"burstNum: "<<burstNum<<endl;
		csvOut.getOutputStream()<<"refreshNum: "<<refreshNum<<endl;
		
		csvOut.getOutputStream()<<"actpreNum_dram: "<<actpreNum_dram<<endl;
		csvOut.getOutputStream()<<"burstNum_dram: "<<burstNum_dram<<endl;
		csvOut.getOutputStream()<<"refreshNum_dram: "<<refreshNum_dram<<endl;
		
		csvOut.getOutputStream()<<"actpreNum_pcm: "<<actpreNum_pcm<<endl;
		csvOut.getOutputStream()<<"burstNum_pcm: "<<burstNum_pcm<<endl;
		csvOut.getOutputStream()<<"refreshNum_pcm: "<<refreshNum_pcm<<endl;
		
		csvOut.getOutputStream()<<"totalPower_MS: "<<(totalBurstEnergy_MS+totalActpreEnergy_MS+totalRefreshEnergy_MS)/((double)currentClockCycle*iniReader->tCK * 1E3)<<endl;
		csvOut.getOutputStream()<<"totalActprePower_MS: "<<totalActpreEnergy_MS/((double)currentClockCycle*iniReader->tCK * 1E3)<<endl;
		csvOut.getOutputStream()<<"totalBurstPower_MS: "<<totalBurstEnergy_MS/((double)currentClockCycle*iniReader->tCK * 1E3)<<endl;
		csvOut.getOutputStream()<<"totalRefreshPower_MS: "<<totalRefreshEnergy_MS/((double)currentClockCycle*iniReader->tCK * 1E3)<<endl;

		csvOut.getOutputStream()<<"totalEnergy_MS: "<<(totalBurstEnergy_MS+totalActpreEnergy_MS+totalRefreshEnergy_MS)/(1E9)<<endl;
		csvOut.getOutputStream()<<"totalActpreEnergy_MS: "<<totalActpreEnergy_MS/(1E9)<<endl;
		csvOut.getOutputStream()<<"totalBurstEnergy_MS: "<<totalBurstEnergy_MS/(1E9)<<endl;
		csvOut.getOutputStream()<<"totalRefreshEnergy_MS: "<<totalRefreshEnergy_MS/(1E9)<<endl;


		
		
		sort(readLatency.begin(), readLatency.end());
		sort(writeLatency.begin(), writeLatency.end());
		uint64_t totalTransactionQueueDelay = 0;
		uint64_t totalCommandQueueDelay = 0;
		for(size_t i=0; i<transactionQueueDelay.size(); i++)
		{
			totalTransactionQueueDelay+=transactionQueueDelay[i];
		}
		for(size_t i=0; i<commandQueueDelay.size(); i++)
		{
			totalCommandQueueDelay+=commandQueueDelay[i];
		}

		//csvOut.getOutputStream()<<"transactionQueueDelay: "<<(float)totalTransactionQueueDelay/(float)transactionQueueDelay.size() * parentMemorySystem->iniReader->tCK<<endl;
		//csvOut.getOutputStream()<<"commandQueueDelay: "<<(float)totalCommandQueueDelay/(float)commandQueueDelay.size() * parentMemorySystem->iniReader->tCK<<endl;
		


		int n1=0;
		int n2=0;
		for(int m=1; m<=10; m++)
		{
			n1 = readLatency.size()*m/10-1;
			if(n1 < 0)
			{
				n1 = 0;
			}

			csvOut.getOutputStream()<<"readLatency"<<m<<": "<<readLatency[n1]<<endl;

		}
		for(int m=1; m<=10; m++)
		{
			n2 = writeLatency.size()*m/10-1;
			if(n2 < 0)
			{
				n2 = 0;
			}
			csvOut.getOutputStream()<<"writeLatency"<<m<<": "<<writeLatency[n2]<<endl;
		}

		csvOut.getOutputStream()<<"end"<<endl;

	}
}
//inserts a latency into the latency histogram
void MemoryController::insertHistogram(unsigned latencyValue, unsigned rank, unsigned bank)
{
	readLatency.push_back(latencyValue);
	totalEpochLatency[SEQUENTIAL(rank,bank)] += latencyValue;
	//poor man's way to bin things.
	latencies[(latencyValue/HISTOGRAM_BIN_SIZE)*HISTOGRAM_BIN_SIZE]++;
}
