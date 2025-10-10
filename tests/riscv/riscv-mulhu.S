//========================================================================
// riscv-mulhu.S  -- test MULHU (unsigned x unsigned) high 32 bits
//========================================================================

  .section .text
  .globl _start
_start:
  # enable stats (csr 10, bit0=1)
  li   t0, 1
  csrw 10, t0

  # case 1: 2 * 3 = 6 -> hi=0
  li   a0, 2
  li   a1, 3
  mulhu t1, a0, a1
  li   t2, 0x00000000
  bne  t1, t2, fail

  # case 2: 0xffffffff * 0xffffffff = 0xfffffffe00000001 -> hi=0xfffffffe
  li   a0, -1              # as unsigned = 0xffffffff
  li   a1, -1
  mulhu t1, a0, a1
  li   t2, 0xfffffffe
  bne  t1, t2, fail

  # case 3: 0x80000000 * 2 = 0x0000000100000000 -> hi=0x00000001
  li   a0, 0x80000000
  li   a1, 2
  mulhu t1, a0, a1
  li   t2, 0x00000001
  bne  t1, t2, fail

  # case 4: 0xffffffff * 1 = 0xffffffff -> hi=0x00000000
  li   a0, -1
  li   a1, 1
  mulhu t1, a0, a1
  li   t2, 0x00000000
  bne  t1, t2, fail

pass:
  li   t0, 1
  csrw 21, t0
done:
  j    done

fail:
  li   t0, 13
  csrw 21, t0
  j    done
