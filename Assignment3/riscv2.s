.globl __start

.text
.align 4
        #pc는 0x0001_0000에서 시작
        #뭔가 두줄이 실행됨. 초기화? 무튼 pc는 0x0001_0008로 이동

__start:
        la sp, stack
        addi a0, zero, 6
        jal factorial
        j exit
        
factorial:
        addi t0, zero, 1
        bne a0, t0, else
        ret
else:
        addi sp, sp, -8
        sw a0, 4(sp)
        sw ra, 0(sp)
        addi a0, a0, -1
        jal factorial
        lw t1, 4(sp)
        lw ra, 0(sp)
        addi sp, sp, 8
        mul a0, a0, t1
        ret

exit: 
        nop
        
.data
.align 20
stack:
      .space 1024