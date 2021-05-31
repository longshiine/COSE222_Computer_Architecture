#
#  Author: Prof. Jay_kim
#          Computer Science & Engineering
#          Korea University
#  Date:   september 01, 2020
#  Description: Bubble Sort
#

.globl __start

.text
.align 4
        #pc는 0x0001_0000에서 시작
        #뭔가 두줄이 실행됨. 초기화? 무튼 pc는 0x0001_0008로 이동

__start:
        la   t0, Input_data # address of Input_data
	la   t1, Output_data # address of output_data
        lui   t5, 0x00004
	
        mv sp, t0 # sp = (stack pointer)temp address of input_data
        sub  a0, t1, t0 # difference between two address
        srai a0, a0, 2 # legnth of array
        addi a1, a0, -1 # variable for inner loop
        addi a2, a0, -1 # variable for outter loop

inner_loop:
	lw   s0, 0(sp) # load first word from input_data
	lw   s1, 4(sp) # load second word from input_data
        bge  s0, s1, next # if s0 >= s1 then next
        # swap
        sw s1, 0(sp)
        sw s0, 4(sp)
	j next

next:   # next
	addi a1, a1, -1   # decrement by 1 variable for inner loop
        beq  a1, zero, outer_loop # if a1 == 0  then outter_loop 
        addi sp, sp, 4
        j    inner_loop # jump to inner_loop

outer_loop:
        addi a2, a2, -1
        beq a2, zero, assign_output # if a2 ==0 then assign_output
        
        # initialize to 0
        mv a1, a2
        mv sp, t0
        j    inner_loop

assign_output:
        lw   s0, 0(t0)
        sw   s0, 0(t1)
        addi a0, a0, -1
        beq  a0, zero, end
        addi t0, t0, 4
        addi t1, t1, 4
        j assign_output
end: 
        nop
        
.data
.align 4
Input_data: 
 .word 2, 0, -7, -1, 3, 8, -4, 10
 .word -9, -16, 15, 13, 1, 4, -3, 14
 .word -8, -10, -15, 6, -13, -5, 9, 12
 .word -11, -14, -6, 11, 5, 7, -2, -12
Output_data:
 .word 0, 0, 0, 0, 0, 0, 0, 0
 .word 0, 0, 0, 0, 0, 0, 0, 0
 .word 0, 0, 0, 0, 0, 0, 0, 0
 .word 0, 0, 0, 0, 0, 0, 0, 0

