# HMSim1
This Hybrid Memory (DRAM+NVM) Simulator is developed based on DRAMSim2.

1. How does it simulate hybrid memory system?
	We use multiple channels to simulate DRAM and NVM respectively.
	Current version supports two channels, one channel is DRAM, 
the other channel is NVM.
	It's easy to support more channels by modifing the 
MultiChannelMemorySystem construct function.

2. What's the configurations of NVM?
	The conf/PCM_micron_16M_8B_x16_sg25E.ini shows an example 
of PCM configuration [1]. 
Please refer to the MemoryController::update() for energy calculation.

3. Whats' the format of input trace?
	The default memory access trace is an binary file. Each memory 
request occupies 64bits. Please refer to the parseTraceFileLine_new() 
for concrete trace format.

4. How to run the simulator?
	The shell script run.sh gives an example of running the HMSim1.
  

[1]. Architecting phase change memory as a scalable dram alternative. ISCA'2009
