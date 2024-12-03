# SimulatorEnvironment

# Compile the FPGA dataflow simulator:
cd [Simulator folder path]
mkdir build && cd build
cmake -G "Unix Makefiles" .. && make

# Add the fpga-dataflow-sim path to the environment variable FPGA_SIM_PATH:
export FPGA_SIM_PATH=[Simulator folder path]
