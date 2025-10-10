//========================================================================
// riscv-mulhsu.S  -- test MULHSU (signed x unsigned) high 32 bits
//========================================================================

  .section .text
  .globl _start
_start:
  # enable stats (csr 10, bit0=1)
  li   t0, 1
  csrw 10, t0

  # case 1: (-1) * 0xffffffffu = -4294967295 -> 0xffffffff00000001 -> hi=0xffffffff
  li   a0, -1
  li   a1, -1            # as unsigned = 0xffffffff
  mulhsu t1, a0, a1
  li   t2, 0xffffffff
  bne  t1, t2, fail

  # case 2: (-1) * 0x80000000u = -2147483648 -> 0xffffffff80000000 -> hi=0xffffffff
  li   a0, -1
  li   a1, 0x80000000
  mulhsu t1, a0, a1
  li   t2, 0xffffffff
  bne  t1, t2, fail

  # case 3: 0x7fffffff * 0xffffffffu
  # 2147483647 * 4294967295 = 0x7ffffffe80000001 -> hi=0x7ffffffe
  li   a0, 0x7fffffff
  li   a1, -1
  mulhsu t1, a0, a1
  li   t2, 0x7ffffffe
  bne  t1, t2, fail

  # case 4: (-2) * 3u = -6 -> 0xfffffffffffffffa -> hi=0xffffffff
  li   a0, -2
  li   a1, 3
  mulhsu t1, a0, a1
  li   t2, 0xffffffff
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
