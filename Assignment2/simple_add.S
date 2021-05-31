#
#  Author: Prof. Taeweon Suh
#          Computer Science & Engineering
#          Korea University
#  Date:   June 13, 2020
#  Description: Simple addition of 4 words (result = op1 + op2)
#

.globl __start

.text
.align 4

        #pc는 0x0001_0000에서 시작
        #뭔가 두줄이 실행됨. 초기화? 무튼 pc는 0x0001_0008로 이동

__start:
        la   t0, op1 
        # <load address>
        # auipc x5, 0 => pc의 현재 주소를 담는다.
        # addi x5, x5, 108(혹은 다른값) => pc의 현재주소에서 108(혹은 다른 값)을 증가시킨값을 x5에 저장한다
        # => 즉 op1이 시작되는 주소를 t0(x5에 담는다)
        # (auipc, addi) op1의 시작'주소'를 t0에 담는다.(pc(0x0001_0008) + 108(혹은 다른) = 0x0001_0074) 
        #  --> 메모리의 해당 주소(0x0001_0074)에는 4바이트(1word)씩 1,2,3,4
        #  왜 108을 더하는 지는 생각해보면 심플하다. 메모리에 로드된 코드가 108 뒤에서 끝나고 그 뒷부분에 data(op1에 할당된값)가 로드되어 있을 것임.
	la   t1, op2 
        # (auipc, addi) op2의 시작'주소'를 t1에 담는다. (pc(0x0001_0010) + 116 = 0x0001_0084) 
	la   t2, result
        # result의 시작'주소'를 t2에 담는다.
	sub t3, t1, t0 
        # t3에는 t1 - t0 = 0x0001_0084 - 0x0001_0074 = 0x0000_0010 즉 16(4바이트)가 저장
        # 왜냐하면 t1,t0는 주소값이고 t0부터 op1에 해당하는 4바이트가 담기고 이후에 t1부터 op2에 해당하는 4바이트가 담긴다.
        # t3는 1 word의 크기인 4byte를 나타낸다
	srai  t3, t3, 2   # how many words? shift right by 2
        # t3가 우측으로 2bit shift되면 /4의 효과이고, 1/4 word이다. t3 = 0x0000_0004
        # t3가 나타내고 싶은 값은 word의 개수이다. = how many words?
        # 두 주소의 차이를 2bit right shift 할 시에 각 주소범위가 담고 있는 word의 개수를 계산

myloop:
	lw   s0, 0(t0)
        # t0에는 op1의 주소가 담겨있고, 해당 주소에서 1word(4byte)만큼 s0에 로드한다.
        # 원래 t0에는 op1이 시작되는 주소가 담겨있었고, 시작주소부터 4word(16byte)만큼 이후의 주소가 t1이다. 
	lw   s1, 0(t1)
        # t1에는 op2의 주소가 담겨있고, 해당 주소에서 1word(4byte)만큼 s1에 로드한다.
	
        #add  s2, s1, s0
        # s2에 s1과 s0를 더한 값을 담는다.
        # 이때 주의할 것은 s2라는 것은 cpu의 레지스터로 아직 메모리에 저장되지 않았다.
        
        
       
        #sub s2, s0, s1
        
        
        
	#sw   s2, 0(t2)
        
        
        sw   s0, 0(t2)
        
        
        # t2는 result의 주소를 담고 있었다. t2주소에 해당하는 메모리 공간에 s2값을 할당한다.
	addi t3, t3, -1   # decrement by 1
        # 한 word간의 계산을 끝냈으므로 1을 감소시킨다.
	beq  t3, zero, myself
        # goto myself if t3 == 0
	addi t0, t0, 4
        # t0(op1 주소)를 4바이트 증가시킨다. 다음 word로 이동! word address를 증가시킨다고 볼 수 있다.
	addi t1, t1, 4
	# t1(op2 주소)를 4바이트 증가시킨다. 다음 word로 이동!
        addi t2, t2, 4
        # t2(result 주소)를 4바이트 증가시킨다. 다음 word로 이동!
	j    myloop
        # jump to myloop

myself:
        # op1 op2 간의 연산을 result에 모두 저장후에 이곳으로 넘어온다. (t3 == 0, 더이상 계산할 워드가 없을때)
        li   t0, 0x12345678   # just an example pseudo code
        nop	              # just an example pseudo code
        mv   t1, t0           # just an example pseudo code
        # j myself
        

.data
.align 4
op1:    .word  1, 2, 3, 4 
op2:    .word  5, 6, 7, 8
result: .word  0, 0, 0, 0 
