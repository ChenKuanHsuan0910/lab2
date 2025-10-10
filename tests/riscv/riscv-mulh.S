//========================================================================
// riscv-mulh.S  -- test MULH (signed x signed) high 32 bits
//========================================================================

  .section .text
  .globl _start
_start:
  # enable stats (csr 10, bit0=1)
  li   t0, 1
  csrw 10, t0

  # ------------------------
  # case 1: 2 * 3 = 6 -> hi=0x00000000
  li   a0, 2
  li   a1, 3
  mulh t1, a0, a1
  li   t2, 0x00000000
  bne  t1, t2, fail

  # case 2: (-1) * (-1) = 1 -> hi=0x00000000
  li   a0, -1
  li   a1, -1
  mulh t1, a0, a1
  li   t2, 0x00000000
  bne  t1, t2, fail

  # case 3: 0x7fffffff * 0x7fffffff = 0x3fffffff00000001 -> hi=0x3fffffff
  li   a0, 0x7fffffff
  li   a1, 0x7fffffff
  mulh t1, a0, a1
  li   t2, 0x3fffffff
  bne  t1, t2, fail

  # case 4: (-2) * 3 = -6 -> 0xfffffffffffffffA -> hi=0xffffffff
  li   a0, -2
  li   a1, 3
  mulh t1, a0, a1
  li   t2, 0xffffffff
  bne  t1, t2, fail

  # case 5: (-3) * (-2) = 6 -> hi=0x00000000
  li   a0, -3
  li   a1, -2
  mulh t1, a0, a1
  li   t2, 0x00000000
  bne  t1, t2, fail

pass:
  li   t0, 1         # status=1
  csrw 21, t0
done:
  j    done

fail:
  li   t0, 13        # status=13
  csrw 21, t0
  j    done
