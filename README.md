# COSE222_Computer_Architecture
## Summary
COSE222: This course covers the computer organization details, mainly focusing on CPU internal and memory hierarchy (cache, memory and storage)

### 학습목표
1. Instructions (Machine Code)  
2. Processor Performance  
3. RISC-V CPU Design  
4. Memory Hierarchy  
5. Storage, Network, and Other Peripherals  

## Contents
### 1. Assignments (Assembly Coding)  

### 2. RISC-V CPU Design (single-cycle, pipeline, ..etc)  
- **1th milestone** : Follow the synthesis & simulation tutorial linked on the class web. The example code in the tutorial will display a number 5 on the HEX0 7 Segment of DE0 Board.  

- **2nd milestone**: Draw the DETAILED schematic diagram of the single-cycle RISC-V processor posted on the class web. The top module is RV32I_System.v and you should draw the schematic of all the hierarchy (from the top to the bottom modules).  

*Schematic of all the hierarchy*  
<img width="830" alt="스크린샷 2021-05-31 오전 11 48 07" src="https://user-images.githubusercontent.com/70363646/120132882-143bfc80-c206-11eb-895e-e57ef892b1f4.png">
<img width="827" alt="스크린샷 2021-05-31 오전 11 48 26" src="https://user-images.githubusercontent.com/70363646/120132909-1ef69180-c206-11eb-9db4-1950ba756cb6.png">
<img width="798" alt="스크린샷 2021-05-31 오전 11 48 44" src="https://user-images.githubusercontent.com/70363646/120132928-29b12680-c206-11eb-90c3-389c65ca1de2.png">


- **3rd milestone**: Add new instructions to the provided single-cycle CPU, so that the 2 programs in the zip file can be executed on the new CPU. The programs will display 5 and A alternatively on HEX0 of the DE0 board with some interval of time if the CPU is implemented correctly.  

- **4th milestone**: Based on the Single-cycle CPU you have designed for the 3rd milestone, implement the pipelining with data hazard detection and handling logic. As studied in the class, the pipelining incurs hazards (structural, data and control hazards). Among them, the structural hazard does not occur in our CPU since memory provides 2 ports: one port for instruction access and the other for data access. The purpose of the 4th milestone is to implement pipelining and detect/resolve the data hazard.  

*pipeline excution diagram*  
<img width="836" alt="스크린샷 2021-05-31 오전 11 49 14" src="https://user-images.githubusercontent.com/70363646/120132968-3d5c8d00-c206-11eb-8f0c-36a3aa797cab.png">  

- **5th milestone**: Add control hazard detection and handling logic on the pipelined CPU you have designed for the 4th milestone. Your pipelined version of CPU should be able to run the following code. It will display 1, 2, 3, and 4 on HEX3 ~ HEX0 if CPU is implemented correctly.  You should satisfy the following design requirements.  
  1. Branch (beq, bne,…) outcome and destination are calculated in the EX stage of
the pipeline  
  2. Jump (jal, jalr) destination are calculated in the EX stage of the pipeline  

- **Last milestone**: Update your pipelined version of RISC-V, so that it is able to execute the code in the zip file. It will repeatedly display 0, 1, 2, 3, 4, 5, 6, 7, 8, and 9 on HEX0 with a certain time interval. RV32I_System.v is an overly simplified version of a computer system, consisting of RISC- V CPU, Timer, and GPIO. Nevertheless, it teaches you almost everything you need to know about computer systems. Please think about the followings;  
