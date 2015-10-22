#This is an example of running the HMSim1.
# -c: number of memory requests.
# -S: DRAM capacity
# -X: NVM capacity

./HMSim1 \
    -t traces/input \
	-b conf/debug_config.ini \
    -s conf/systemdram.ini \
    -x conf/systempcm.ini \
    -d conf/DDR3_micron_32M_8B_x8_sg25E.ini \
    -e conf/PCM_micron_16M_8B_x16_sg25E.ini \
    -c 10000 \
    -S 2048 \
    -X 2048 \
    -v rlt
