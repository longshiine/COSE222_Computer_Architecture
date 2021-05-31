
.global __start
__start: 
          slt t0, s0, s1
          
.align 4
.data
  list: 
    .word 1,2,3,4