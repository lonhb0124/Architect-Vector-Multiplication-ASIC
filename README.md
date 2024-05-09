# Architect-Vector-Multiplication-ASIC

Our ASIC module has interfaces for PROC and EXMEM to receive commands and send requests. If instruction from PROC and values in EXMEM are valid, begin calculation CALC module will iterate through MxN matrix, and store output to memory. Once calculation is complete, output is available in memory and signals PROC. All the requirements has been implemented, up to k=0..7, M,N=1..64, f=0/1, a=0..24.
Basic testing of correctness Design follows control-datapath paradigm Some optimization choices while looping through the matrix (i.e. reading whole row for calculation, vector X only loaded once)


# Architectural Description

Our ASIC design has five main states, where some states also have their own sub-states. 

Main state machine’s state:

{STATE_IDLE, STATE_CMD, STATE_MEMORY, STATE_CALC, STATE_DONE}

Sub-state machine ‘s state:

{ MEM_IDLE, MEM_INIT_X,  MEM_LOAD_VALID_X, MEM_INIT_W, MEM_LOAD_VALID_W, MEM_NEXT_N, MEM_INCREMENT_N, MEM_STORE, MEM_STORE_VALID, MEM_NEXT_M, MEM_INCREMENT_M}

Upon reset, ASIC starts in STATE_IDLE. System is ready to receive instructions from PROC. A valid instruction triggers staged gathering of addresses and calculation parameters in STATE_CMD state. The parameters (i.e. a, k, M, N, address of w, x, r, and output type f) are stored in registers. 

Once the instruction has been fully decoded, ASIC is now in STATE_MEMORY. Memory operation consists of its own state machine starting in MEM_IDLE. From memory, N elements from the M-th row (Nx1) of matrix and a column vector X (1xN) are loaded. The calculated value stores to external memory as seen in the figure below. This process is repeated M times for each row. To increase the performance of Asic, Asic loads X only once and stores it into internal registers for next M times calculation. Pipelining and parallel were not used in our Asic architecture.


![image](https://github.com/lonhb0124/Architect-Vector-Multiplication-ASIC/assets/111609834/1d4123b5-f340-4a9b-9ee8-b0839f5c61a8)

# state - flow

![eecs4612_proj2_design-Main State Machine drawio](https://github.com/lonhb0124/Architect-Vector-Multiplication-ASIC/assets/111609834/c0c8aaae-34b3-4ac1-959e-46c224d48470)
![eecs4612_proj2_design-State_Machine drawio](https://github.com/lonhb0124/Architect-Vector-Multiplication-ASIC/assets/111609834/8e47109b-40c3-4a51-8651-c9dbf9a99b8d)




