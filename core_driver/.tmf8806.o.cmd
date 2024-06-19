cmd_/home/ams/workspace/mf8806_tmf8806-linux-driver_main/./core_driver/tmf8806.o :=  gcc-12 -Wp,-MMD,/home/ams/workspace/mf8806_tmf8806-linux-driver_main/./core_driver/.tmf8806.o.d -nostdinc -I/usr/src/linux-headers-6.1.0-rpi7-common-rpi/arch/arm/include -I./arch/arm/include/generated -I/usr/src/linux-headers-6.1.0-rpi7-common-rpi/include -I./include -I/usr/src/linux-headers-6.1.0-rpi7-common-rpi/arch/arm/include/uapi -I./arch/arm/include/generated/uapi -I/usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/uapi -I./include/generated/uapi -include /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/compiler-version.h -include /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/kconfig.h -include /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/compiler_types.h -D__KERNEL__ -mlittle-endian -fmacro-prefix-map=/usr/src/linux-headers-6.1.0-rpi7-common-rpi/= -Wall -Wundef -Werror=strict-prototypes -Wno-trigraphs -fno-strict-aliasing -fno-common -fshort-wchar -fno-PIE -Werror=implicit-function-declaration -Werror=implicit-int -Werror=return-type -Wno-format-security -std=gnu11 -fno-dwarf2-cfi-asm -mno-fdpic -fno-ipa-sra -mabi=aapcs-linux -mfpu=vfp -funwind-tables -mtp=cp15 -marm -Wa,-mno-warn-deprecated -D__LINUX_ARM_ARCH__=6 -march=armv6k -mtune=arm1136j-s -msoft-float -Uarm -fno-delete-null-pointer-checks -Wno-frame-address -Wno-format-truncation -Wno-format-overflow -Wno-address-of-packed-member -O2 -fno-allow-store-data-races -Wframe-larger-than=1024 -fstack-protector-strong -Wno-main -Wno-unused-but-set-variable -Wno-unused-const-variable -Wno-dangling-pointer -ftrivial-auto-var-init=zero -fno-stack-clash-protection -pg -Wdeclaration-after-statement -Wvla -Wno-pointer-sign -Wcast-function-type -Wno-stringop-truncation -Wno-stringop-overflow -Wno-restrict -Wno-maybe-uninitialized -Wno-array-bounds -Wno-alloc-size-larger-than -Wimplicit-fallthrough=5 -fno-strict-overflow -fno-stack-check -fconserve-stack -Werror=date-time -Werror=incompatible-pointer-types -Werror=designated-init -Wno-packed-not-aligned -mstack-protector-guard=tls -mstack-protector-guard-offset=1192 -I/home/ams/workspace/mf8806_tmf8806-linux-driver_main/core_driver -I/home/ams/workspace/mf8806_tmf8806-linux-driver_main -Wno-unused-function  -DMODULE  -DKBUILD_BASENAME='"tmf8806"' -DKBUILD_MODNAME='"tmf8806"' -D__KBUILD_MODNAME=kmod_tmf8806 -c -o /home/ams/workspace/mf8806_tmf8806-linux-driver_main/./core_driver/tmf8806.o /home/ams/workspace/mf8806_tmf8806-linux-driver_main/./core_driver/tmf8806.c  

source_/home/ams/workspace/mf8806_tmf8806-linux-driver_main/./core_driver/tmf8806.o := /home/ams/workspace/mf8806_tmf8806-linux-driver_main/./core_driver/tmf8806.c

deps_/home/ams/workspace/mf8806_tmf8806-linux-driver_main/./core_driver/tmf8806.o := \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/compiler-version.h \
    $(wildcard include/config/CC_VERSION_TEXT) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/kconfig.h \
    $(wildcard include/config/CPU_BIG_ENDIAN) \
    $(wildcard include/config/BOOGER) \
    $(wildcard include/config/FOO) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/compiler_types.h \
    $(wildcard include/config/DEBUG_INFO_BTF) \
    $(wildcard include/config/PAHOLE_HAS_BTF_TAG) \
    $(wildcard include/config/HAVE_ARCH_COMPILER_H) \
    $(wildcard include/config/CC_HAS_ASM_INLINE) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/compiler_attributes.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/compiler-gcc.h \
    $(wildcard include/config/RETPOLINE) \
    $(wildcard include/config/ARCH_USE_BUILTIN_BSWAP) \
    $(wildcard include/config/SHADOW_CALL_STACK) \
    $(wildcard include/config/KCOV) \
  /home/ams/workspace/mf8806_tmf8806-linux-driver_main/./core_driver/tmf8806.h \
  /home/ams/workspace/mf8806_tmf8806-linux-driver_main/tmf8806_shim.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/kernel.h \
    $(wildcard include/config/PREEMPT_VOLUNTARY_BUILD) \
    $(wildcard include/config/PREEMPT_DYNAMIC) \
    $(wildcard include/config/HAVE_PREEMPT_DYNAMIC_CALL) \
    $(wildcard include/config/HAVE_PREEMPT_DYNAMIC_KEY) \
    $(wildcard include/config/PREEMPT_) \
    $(wildcard include/config/DEBUG_ATOMIC_SLEEP) \
    $(wildcard include/config/SMP) \
    $(wildcard include/config/MMU) \
    $(wildcard include/config/PROVE_LOCKING) \
    $(wildcard include/config/TRACING) \
    $(wildcard include/config/FTRACE_MCOUNT_RECORD) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/stdarg.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/align.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/const.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/vdso/const.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/uapi/linux/const.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/limits.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/uapi/linux/limits.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/types.h \
    $(wildcard include/config/HAVE_UID16) \
    $(wildcard include/config/UID16) \
    $(wildcard include/config/ARCH_DMA_ADDR_T_64BIT) \
    $(wildcard include/config/PHYS_ADDR_T_64BIT) \
    $(wildcard include/config/64BIT) \
    $(wildcard include/config/ARCH_32BIT_USTAT_F_TINODE) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/uapi/linux/types.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/arch/arm/include/uapi/asm/types.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/asm-generic/int-ll64.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/uapi/asm-generic/int-ll64.h \
  arch/arm/include/generated/uapi/asm/bitsperlong.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/asm-generic/bitsperlong.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/uapi/asm-generic/bitsperlong.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/uapi/linux/posix_types.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/stddef.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/uapi/linux/stddef.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/compiler_types.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/arch/arm/include/uapi/asm/posix_types.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/uapi/asm-generic/posix_types.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/vdso/limits.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/linkage.h \
    $(wildcard include/config/ARCH_USE_SYM_ANNOTATIONS) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/stringify.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/export.h \
    $(wildcard include/config/MODVERSIONS) \
    $(wildcard include/config/HAVE_ARCH_PREL32_RELOCATIONS) \
    $(wildcard include/config/MODULES) \
    $(wildcard include/config/TRIM_UNUSED_KSYMS) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/arch/arm/include/asm/linkage.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/compiler.h \
    $(wildcard include/config/TRACE_BRANCH_PROFILING) \
    $(wildcard include/config/PROFILE_ALL_BRANCHES) \
    $(wildcard include/config/OBJTOOL) \
  arch/arm/include/generated/asm/rwonce.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/asm-generic/rwonce.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/kasan-checks.h \
    $(wildcard include/config/KASAN_GENERIC) \
    $(wildcard include/config/KASAN_SW_TAGS) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/kcsan-checks.h \
    $(wildcard include/config/KCSAN) \
    $(wildcard include/config/KCSAN_WEAK_MEMORY) \
    $(wildcard include/config/KCSAN_IGNORE_ATOMICS) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/container_of.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/build_bug.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/err.h \
  arch/arm/include/generated/uapi/asm/errno.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/uapi/asm-generic/errno.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/uapi/asm-generic/errno-base.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/bitops.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/bits.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/vdso/bits.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/typecheck.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/uapi/linux/kernel.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/uapi/linux/sysinfo.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/asm-generic/bitops/generic-non-atomic.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/arch/arm/include/asm/barrier.h \
    $(wildcard include/config/CPU_32v6K) \
    $(wildcard include/config/THUMB2_KERNEL) \
    $(wildcard include/config/CPU_XSC3) \
    $(wildcard include/config/CPU_FA526) \
    $(wildcard include/config/ARM_HEAVY_MB) \
    $(wildcard include/config/ARM_DMA_MEM_BUFFERABLE) \
    $(wildcard include/config/CPU_SPECTRE) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/asm-generic/barrier.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/arch/arm/include/asm/bitops.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/irqflags.h \
    $(wildcard include/config/TRACE_IRQFLAGS) \
    $(wildcard include/config/PREEMPT_RT) \
    $(wildcard include/config/IRQSOFF_TRACER) \
    $(wildcard include/config/PREEMPT_TRACER) \
    $(wildcard include/config/DEBUG_IRQFLAGS) \
    $(wildcard include/config/TRACE_IRQFLAGS_SUPPORT) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/arch/arm/include/asm/irqflags.h \
    $(wildcard include/config/CPU_V7M) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/arch/arm/include/asm/ptrace.h \
    $(wildcard include/config/ARM_THUMB) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/arch/arm/include/uapi/asm/ptrace.h \
    $(wildcard include/config/CPU_ENDIAN_BE8) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/arch/arm/include/asm/hwcap.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/arch/arm/include/uapi/asm/hwcap.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/asm-generic/irqflags.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/arch/arm/include/asm/percpu.h \
    $(wildcard include/config/CPU_V6) \
    $(wildcard include/config/ARM_HAS_GROUP_RELOCS) \
    $(wildcard include/config/ARM_MODULE_PLTS) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/arch/arm/include/asm/insn.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/asm-generic/percpu.h \
    $(wildcard include/config/DEBUG_PREEMPT) \
    $(wildcard include/config/HAVE_SETUP_PER_CPU_AREA) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/threads.h \
    $(wildcard include/config/NR_CPUS) \
    $(wildcard include/config/BASE_SMALL) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/percpu-defs.h \
    $(wildcard include/config/DEBUG_FORCE_WEAK_PER_CPU) \
    $(wildcard include/config/AMD_MEM_ENCRYPT) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/asm-generic/bitops/non-atomic.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/asm-generic/bitops/non-instrumented-non-atomic.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/asm-generic/bitops/builtin-__fls.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/asm-generic/bitops/builtin-__ffs.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/asm-generic/bitops/builtin-fls.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/asm-generic/bitops/builtin-ffs.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/asm-generic/bitops/ffz.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/asm-generic/bitops/fls64.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/asm-generic/bitops/sched.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/asm-generic/bitops/hweight.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/asm-generic/bitops/arch_hweight.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/asm-generic/bitops/const_hweight.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/asm-generic/bitops/lock.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/atomic.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/arch/arm/include/asm/atomic.h \
    $(wildcard include/config/GENERIC_ATOMIC64) \
    $(wildcard include/config/ARM_LPAE) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/prefetch.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/arch/arm/include/asm/processor.h \
    $(wildcard include/config/HAVE_HW_BREAKPOINT) \
    $(wildcard include/config/BINFMT_ELF_FDPIC) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/arch/arm/include/asm/hw_breakpoint.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/arch/arm/include/asm/unified.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/arch/arm/include/asm/vdso/processor.h \
    $(wildcard include/config/ARM_ERRATA_754327) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/arch/arm/include/asm/cache.h \
    $(wildcard include/config/ARM_L1_CACHE_SHIFT) \
    $(wildcard include/config/AEABI) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/arch/arm/include/asm/cmpxchg.h \
    $(wildcard include/config/CPU_SA1100) \
    $(wildcard include/config/CPU_SA110) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/asm-generic/cmpxchg-local.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/atomic/atomic-arch-fallback.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/atomic/atomic-long.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/atomic/atomic-instrumented.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/instrumented.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/kmsan-checks.h \
    $(wildcard include/config/KMSAN) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/asm-generic/bitops/instrumented-lock.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/asm-generic/bitops/le.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/arch/arm/include/uapi/asm/byteorder.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/byteorder/little_endian.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/uapi/linux/byteorder/little_endian.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/swab.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/uapi/linux/swab.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/arch/arm/include/asm/swab.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/arch/arm/include/uapi/asm/swab.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/byteorder/generic.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/asm-generic/bitops/ext2-atomic-setbit.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/kstrtox.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/log2.h \
    $(wildcard include/config/ARCH_HAS_ILOG2_U32) \
    $(wildcard include/config/ARCH_HAS_ILOG2_U64) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/math.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/arch/arm/include/asm/div64.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/arch/arm/include/asm/compiler.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/asm-generic/div64.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/minmax.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/panic.h \
    $(wildcard include/config/PANIC_TIMEOUT) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/printk.h \
    $(wildcard include/config/MESSAGE_LOGLEVEL_DEFAULT) \
    $(wildcard include/config/CONSOLE_LOGLEVEL_DEFAULT) \
    $(wildcard include/config/CONSOLE_LOGLEVEL_QUIET) \
    $(wildcard include/config/EARLY_PRINTK) \
    $(wildcard include/config/PRINTK) \
    $(wildcard include/config/PRINTK_INDEX) \
    $(wildcard include/config/DYNAMIC_DEBUG) \
    $(wildcard include/config/DYNAMIC_DEBUG_CORE) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/init.h \
    $(wildcard include/config/STRICT_KERNEL_RWX) \
    $(wildcard include/config/STRICT_MODULE_RWX) \
    $(wildcard include/config/LTO_CLANG) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/kern_levels.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/ratelimit_types.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/uapi/linux/param.h \
  arch/arm/include/generated/uapi/asm/param.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/asm-generic/param.h \
    $(wildcard include/config/HZ) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/uapi/asm-generic/param.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/spinlock_types_raw.h \
    $(wildcard include/config/DEBUG_SPINLOCK) \
    $(wildcard include/config/DEBUG_LOCK_ALLOC) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/spinlock_types_up.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/lockdep_types.h \
    $(wildcard include/config/PROVE_RAW_LOCK_NESTING) \
    $(wildcard include/config/LOCKDEP) \
    $(wildcard include/config/LOCK_STAT) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/once_lite.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/static_call_types.h \
    $(wildcard include/config/HAVE_STATIC_CALL) \
    $(wildcard include/config/HAVE_STATIC_CALL_INLINE) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/instruction_pointer.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/time.h \
    $(wildcard include/config/POSIX_TIMERS) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/cache.h \
    $(wildcard include/config/ARCH_HAS_CACHE_LINE_SIZE) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/math64.h \
    $(wildcard include/config/ARCH_SUPPORTS_INT128) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/vdso/math64.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/time64.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/vdso/time64.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/uapi/linux/time.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/uapi/linux/time_types.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/time32.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/timex.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/uapi/linux/timex.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/arch/arm/include/asm/timex.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/vdso/time32.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/vdso/time.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/i2c.h \
    $(wildcard include/config/I2C) \
    $(wildcard include/config/I2C_SLAVE) \
    $(wildcard include/config/I2C_BOARDINFO) \
    $(wildcard include/config/I2C_MUX) \
    $(wildcard include/config/OF) \
    $(wildcard include/config/ACPI) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/acpi.h \
    $(wildcard include/config/ACPI_DEBUGGER) \
    $(wildcard include/config/ACPI_TABLE_LIB) \
    $(wildcard include/config/X86) \
    $(wildcard include/config/IA64) \
    $(wildcard include/config/LOONGARCH) \
    $(wildcard include/config/ARM64) \
    $(wildcard include/config/ACPI_PROCESSOR_CSTATE) \
    $(wildcard include/config/ACPI_HOTPLUG_CPU) \
    $(wildcard include/config/ACPI_HOTPLUG_IOAPIC) \
    $(wildcard include/config/X86_IO_APIC) \
    $(wildcard include/config/PCI) \
    $(wildcard include/config/ACPI_WMI) \
    $(wildcard include/config/ACPI_NUMA) \
    $(wildcard include/config/HIBERNATION) \
    $(wildcard include/config/PM_SLEEP) \
    $(wildcard include/config/ACPI_HOTPLUG_MEMORY) \
    $(wildcard include/config/ACPI_CONTAINER) \
    $(wildcard include/config/ACPI_GTDT) \
    $(wildcard include/config/PM) \
    $(wildcard include/config/GPIOLIB) \
    $(wildcard include/config/ACPI_TABLE_UPGRADE) \
    $(wildcard include/config/ACPI_WATCHDOG) \
    $(wildcard include/config/ACPI_SPCR_TABLE) \
    $(wildcard include/config/ACPI_GENERIC_GSI) \
    $(wildcard include/config/ACPI_LPIT) \
    $(wildcard include/config/ACPI_PPTT) \
    $(wildcard include/config/ACPI_PCC) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/errno.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/uapi/linux/errno.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/ioport.h \
    $(wildcard include/config/MEMORY_HOTREMOVE) \
    $(wildcard include/config/MEMORY_HOTPLUG) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/irqdomain.h \
    $(wildcard include/config/IRQ_DOMAIN_HIERARCHY) \
    $(wildcard include/config/GENERIC_IRQ_DEBUGFS) \
    $(wildcard include/config/IRQ_DOMAIN) \
    $(wildcard include/config/IRQ_DOMAIN_NOMAP) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/irqhandler.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/of.h \
    $(wildcard include/config/OF_DYNAMIC) \
    $(wildcard include/config/SPARC) \
    $(wildcard include/config/OF_PROMTREE) \
    $(wildcard include/config/OF_KOBJ) \
    $(wildcard include/config/NUMA) \
    $(wildcard include/config/OF_NUMA) \
    $(wildcard include/config/OF_OVERLAY) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/kobject.h \
    $(wildcard include/config/UEVENT_HELPER) \
    $(wildcard include/config/DEBUG_KOBJECT_RELEASE) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/list.h \
    $(wildcard include/config/DEBUG_LIST) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/poison.h \
    $(wildcard include/config/ILLEGAL_POINTER_VALUE) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/sysfs.h \
    $(wildcard include/config/SYSFS) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/kernfs.h \
    $(wildcard include/config/KERNFS) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/mutex.h \
    $(wildcard include/config/MUTEX_SPIN_ON_OWNER) \
    $(wildcard include/config/DEBUG_MUTEXES) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/arch/arm/include/asm/current.h \
    $(wildcard include/config/CURRENT_POINTER_IN_TPIDRURO) \
    $(wildcard include/config/CC_IS_CLANG) \
    $(wildcard include/config/CLANG_VERSION) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/spinlock_types.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/rwlock_types.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/lockdep.h \
    $(wildcard include/config/DEBUG_LOCKING_API_SELFTESTS) \
    $(wildcard include/config/PREEMPT_COUNT) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/smp.h \
    $(wildcard include/config/UP_LATE_INIT) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/cpumask.h \
    $(wildcard include/config/FORCE_NR_CPUS) \
    $(wildcard include/config/HOTPLUG_CPU) \
    $(wildcard include/config/DEBUG_PER_CPU_MAPS) \
    $(wildcard include/config/CPUMASK_OFFSTACK) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/bitmap.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/find.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/string.h \
    $(wildcard include/config/BINARY_PRINTF) \
    $(wildcard include/config/FORTIFY_SOURCE) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/uapi/linux/string.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/arch/arm/include/asm/string.h \
    $(wildcard include/config/KASAN) \
    $(wildcard include/config/BCM2835_FAST_MEMCPY) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/bug.h \
    $(wildcard include/config/GENERIC_BUG) \
    $(wildcard include/config/BUG_ON_DATA_CORRUPTION) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/arch/arm/include/asm/bug.h \
    $(wildcard include/config/DEBUG_BUGVERBOSE) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/arch/arm/include/asm/opcodes.h \
    $(wildcard include/config/CPU_ENDIAN_BE32) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/asm-generic/bug.h \
    $(wildcard include/config/BUG) \
    $(wildcard include/config/GENERIC_BUG_RELATIVE_POINTERS) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/instrumentation.h \
    $(wildcard include/config/NOINSTR_VALIDATION) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/gfp_types.h \
    $(wildcard include/config/KASAN_HW_TAGS) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/numa.h \
    $(wildcard include/config/NODES_SHIFT) \
    $(wildcard include/config/NUMA_KEEP_MEMINFO) \
    $(wildcard include/config/HAVE_ARCH_NODE_DEV_GROUP) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/smp_types.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/llist.h \
    $(wildcard include/config/ARCH_HAVE_NMI_SAFE_CMPXCHG) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/osq_lock.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/debug_locks.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/idr.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/radix-tree.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/percpu.h \
    $(wildcard include/config/NEED_PER_CPU_EMBED_FIRST_CHUNK) \
    $(wildcard include/config/NEED_PER_CPU_PAGE_FIRST_CHUNK) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/mmdebug.h \
    $(wildcard include/config/DEBUG_VM) \
    $(wildcard include/config/DEBUG_VM_IRQSOFF) \
    $(wildcard include/config/DEBUG_VIRTUAL) \
    $(wildcard include/config/DEBUG_VM_PGFLAGS) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/preempt.h \
    $(wildcard include/config/TRACE_PREEMPT_TOGGLE) \
    $(wildcard include/config/PREEMPTION) \
    $(wildcard include/config/PREEMPT_NOTIFIERS) \
  arch/arm/include/generated/asm/preempt.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/asm-generic/preempt.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/thread_info.h \
    $(wildcard include/config/THREAD_INFO_IN_TASK) \
    $(wildcard include/config/GENERIC_ENTRY) \
    $(wildcard include/config/HAVE_ARCH_WITHIN_STACK_FRAMES) \
    $(wildcard include/config/HARDENED_USERCOPY) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/restart_block.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/arch/arm/include/asm/thread_info.h \
    $(wildcard include/config/VMAP_STACK) \
    $(wildcard include/config/ARM_THUMBEE) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/arch/arm/include/asm/fpstate.h \
    $(wildcard include/config/VFPv3) \
    $(wildcard include/config/IWMMXT) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/arch/arm/include/asm/page.h \
    $(wildcard include/config/CPU_COPY_V4WT) \
    $(wildcard include/config/CPU_COPY_V4WB) \
    $(wildcard include/config/CPU_COPY_FEROCEON) \
    $(wildcard include/config/CPU_COPY_FA) \
    $(wildcard include/config/CPU_XSCALE) \
    $(wildcard include/config/CPU_COPY_V6) \
    $(wildcard include/config/KUSER_HELPERS) \
    $(wildcard include/config/HAVE_ARCH_PFN_VALID) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/arch/arm/include/asm/glue.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/arch/arm/include/asm/pgtable-2level-types.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/arch/arm/include/asm/memory.h \
    $(wildcard include/config/NEED_MACH_MEMORY_H) \
    $(wildcard include/config/PAGE_OFFSET) \
    $(wildcard include/config/HIGHMEM) \
    $(wildcard include/config/DRAM_BASE) \
    $(wildcard include/config/DRAM_SIZE) \
    $(wildcard include/config/XIP_KERNEL) \
    $(wildcard include/config/HAVE_TCM) \
    $(wildcard include/config/ARM_PATCH_PHYS_VIRT) \
    $(wildcard include/config/PHYS_OFFSET) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/sizes.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/arch/arm/include/asm/kasan_def.h \
    $(wildcard include/config/KASAN_SHADOW_OFFSET) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/asm-generic/memory_model.h \
    $(wildcard include/config/FLATMEM) \
    $(wildcard include/config/SPARSEMEM_VMEMMAP) \
    $(wildcard include/config/SPARSEMEM) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/pfn.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/asm-generic/getorder.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/rcupdate.h \
    $(wildcard include/config/PREEMPT_RCU) \
    $(wildcard include/config/TINY_RCU) \
    $(wildcard include/config/RCU_STRICT_GRACE_PERIOD) \
    $(wildcard include/config/TASKS_RCU_GENERIC) \
    $(wildcard include/config/RCU_STALL_COMMON) \
    $(wildcard include/config/NO_HZ_FULL) \
    $(wildcard include/config/KVM_XFER_TO_GUEST_WORK) \
    $(wildcard include/config/RCU_NOCB_CPU) \
    $(wildcard include/config/TASKS_RCU) \
    $(wildcard include/config/TASKS_TRACE_RCU) \
    $(wildcard include/config/TASKS_RUDE_RCU) \
    $(wildcard include/config/TREE_RCU) \
    $(wildcard include/config/DEBUG_OBJECTS_RCU_HEAD) \
    $(wildcard include/config/PROVE_RCU) \
    $(wildcard include/config/ARCH_WEAK_RELEASE_ACQUIRE) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/bottom_half.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/context_tracking_irq.h \
    $(wildcard include/config/CONTEXT_TRACKING_IDLE) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/rcutiny.h \
    $(wildcard include/config/SRCU) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/spinlock.h \
  arch/arm/include/generated/asm/mmiowb.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/asm-generic/mmiowb.h \
    $(wildcard include/config/MMIOWB) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/spinlock_up.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/rwlock.h \
    $(wildcard include/config/PREEMPT) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/spinlock_api_up.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/xarray.h \
    $(wildcard include/config/XARRAY_MULTI) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/gfp.h \
    $(wildcard include/config/ZONE_DMA) \
    $(wildcard include/config/ZONE_DMA32) \
    $(wildcard include/config/ZONE_DEVICE) \
    $(wildcard include/config/CONTIG_ALLOC) \
    $(wildcard include/config/CMA) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/mmzone.h \
    $(wildcard include/config/ARCH_FORCE_MAX_ORDER) \
    $(wildcard include/config/MEMORY_ISOLATION) \
    $(wildcard include/config/ZSMALLOC) \
    $(wildcard include/config/SWAP) \
    $(wildcard include/config/NUMA_BALANCING) \
    $(wildcard include/config/TRANSPARENT_HUGEPAGE) \
    $(wildcard include/config/LRU_GEN) \
    $(wildcard include/config/LRU_GEN_STATS) \
    $(wildcard include/config/MEMCG) \
    $(wildcard include/config/COMPACTION) \
    $(wildcard include/config/PAGE_EXTENSION) \
    $(wildcard include/config/DEFERRED_STRUCT_PAGE_INIT) \
    $(wildcard include/config/HAVE_MEMORYLESS_NODES) \
    $(wildcard include/config/SPARSEMEM_EXTREME) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/wait.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/uapi/linux/wait.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/seqlock.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/nodemask.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/random.h \
    $(wildcard include/config/VMGENID) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/once.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/jump_label.h \
    $(wildcard include/config/JUMP_LABEL) \
    $(wildcard include/config/HAVE_ARCH_JUMP_LABEL_RELATIVE) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/arch/arm/include/asm/jump_label.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/uapi/linux/random.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/uapi/linux/ioctl.h \
  arch/arm/include/generated/uapi/asm/ioctl.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/asm-generic/ioctl.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/uapi/asm-generic/ioctl.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/irqnr.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/uapi/linux/irqnr.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/prandom.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/arch/arm/include/asm/archrandom.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/asm-generic/archrandom.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/pageblock-flags.h \
    $(wildcard include/config/HUGETLB_PAGE) \
    $(wildcard include/config/HUGETLB_PAGE_SIZE_VARIABLE) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/page-flags-layout.h \
  include/generated/bounds.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/mm_types.h \
    $(wildcard include/config/HAVE_ALIGNED_STRUCT_PAGE) \
    $(wildcard include/config/USERFAULTFD) \
    $(wildcard include/config/HAVE_ARCH_COMPAT_MMAP_BASES) \
    $(wildcard include/config/MEMBARRIER) \
    $(wildcard include/config/AIO) \
    $(wildcard include/config/MMU_NOTIFIER) \
    $(wildcard include/config/ARCH_WANT_BATCHED_UNMAP_TLB_FLUSH) \
    $(wildcard include/config/IOMMU_SVA) \
    $(wildcard include/config/KSM) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/mm_types_task.h \
    $(wildcard include/config/SPLIT_PTLOCK_CPUS) \
    $(wildcard include/config/ARCH_ENABLE_SPLIT_PMD_PTLOCK) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/auxvec.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/uapi/linux/auxvec.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/arch/arm/include/asm/auxvec.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/arch/arm/include/uapi/asm/auxvec.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/kref.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/refcount.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/rbtree.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/rbtree_types.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/maple_tree.h \
    $(wildcard include/config/MAPLE_RCU_DISABLED) \
    $(wildcard include/config/DEBUG_MAPLE_TREE_VERBOSE) \
    $(wildcard include/config/DEBUG_MAPLE_TREE) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/rwsem.h \
    $(wildcard include/config/RWSEM_SPIN_ON_OWNER) \
    $(wildcard include/config/DEBUG_RWSEMS) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/completion.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/swait.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/uprobes.h \
    $(wildcard include/config/UPROBES) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/workqueue.h \
    $(wildcard include/config/DEBUG_OBJECTS_WORK) \
    $(wildcard include/config/FREEZER) \
    $(wildcard include/config/WQ_WATCHDOG) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/timer.h \
    $(wildcard include/config/DEBUG_OBJECTS_TIMERS) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/ktime.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/jiffies.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/vdso/jiffies.h \
  include/generated/timeconst.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/vdso/ktime.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/timekeeping.h \
    $(wildcard include/config/GENERIC_CMOS_UPDATE) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/clocksource_ids.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/debugobjects.h \
    $(wildcard include/config/DEBUG_OBJECTS) \
    $(wildcard include/config/DEBUG_OBJECTS_FREE) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/arch/arm/include/asm/mmu.h \
    $(wildcard include/config/CPU_HAS_ASID) \
    $(wildcard include/config/VDSO) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/page-flags.h \
    $(wildcard include/config/ARCH_USES_PG_UNCACHED) \
    $(wildcard include/config/MEMORY_FAILURE) \
    $(wildcard include/config/PAGE_IDLE_FLAG) \
    $(wildcard include/config/HUGETLB_PAGE_OPTIMIZE_VMEMMAP) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/local_lock.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/local_lock_internal.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/memory_hotplug.h \
    $(wildcard include/config/HAVE_ARCH_NODEDATA_EXTENSION) \
    $(wildcard include/config/ARCH_HAS_ADD_PAGES) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/notifier.h \
    $(wildcard include/config/TREE_SRCU) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/srcu.h \
    $(wildcard include/config/TINY_SRCU) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/rcu_segcblist.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/srcutiny.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/topology.h \
    $(wildcard include/config/USE_PERCPU_NUMA_NODE_ID) \
    $(wildcard include/config/SCHED_SMT) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/arch_topology.h \
    $(wildcard include/config/ACPI_CPPC_LIB) \
    $(wildcard include/config/GENERIC_ARCH_TOPOLOGY) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/arch/arm/include/asm/topology.h \
    $(wildcard include/config/ARM_CPU_TOPOLOGY) \
    $(wildcard include/config/BL_SWITCHER) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/asm-generic/topology.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/kconfig.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/sched/mm.h \
    $(wildcard include/config/ARCH_HAS_MEMBARRIER_CALLBACKS) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/sched.h \
    $(wildcard include/config/VIRT_CPU_ACCOUNTING_NATIVE) \
    $(wildcard include/config/SCHED_INFO) \
    $(wildcard include/config/SCHEDSTATS) \
    $(wildcard include/config/SCHED_CORE) \
    $(wildcard include/config/FAIR_GROUP_SCHED) \
    $(wildcard include/config/RT_GROUP_SCHED) \
    $(wildcard include/config/RT_MUTEXES) \
    $(wildcard include/config/UCLAMP_TASK) \
    $(wildcard include/config/UCLAMP_BUCKETS_COUNT) \
    $(wildcard include/config/KMAP_LOCAL) \
    $(wildcard include/config/CGROUP_SCHED) \
    $(wildcard include/config/BLK_DEV_IO_TRACE) \
    $(wildcard include/config/PSI) \
    $(wildcard include/config/COMPAT_BRK) \
    $(wildcard include/config/CGROUPS) \
    $(wildcard include/config/BLK_CGROUP) \
    $(wildcard include/config/PAGE_OWNER) \
    $(wildcard include/config/EVENTFD) \
    $(wildcard include/config/CPU_SUP_INTEL) \
    $(wildcard include/config/TASK_DELAY_ACCT) \
    $(wildcard include/config/STACKPROTECTOR) \
    $(wildcard include/config/ARCH_HAS_SCALED_CPUTIME) \
    $(wildcard include/config/VIRT_CPU_ACCOUNTING_GEN) \
    $(wildcard include/config/POSIX_CPUTIMERS) \
    $(wildcard include/config/POSIX_CPU_TIMERS_TASK_WORK) \
    $(wildcard include/config/KEYS) \
    $(wildcard include/config/SYSVIPC) \
    $(wildcard include/config/DETECT_HUNG_TASK) \
    $(wildcard include/config/IO_URING) \
    $(wildcard include/config/AUDIT) \
    $(wildcard include/config/AUDITSYSCALL) \
    $(wildcard include/config/UBSAN) \
    $(wildcard include/config/UBSAN_TRAP) \
    $(wildcard include/config/TASK_XACCT) \
    $(wildcard include/config/CPUSETS) \
    $(wildcard include/config/X86_CPU_RESCTRL) \
    $(wildcard include/config/FUTEX) \
    $(wildcard include/config/COMPAT) \
    $(wildcard include/config/PERF_EVENTS) \
    $(wildcard include/config/RSEQ) \
    $(wildcard include/config/FAULT_INJECTION) \
    $(wildcard include/config/LATENCYTOP) \
    $(wildcard include/config/KUNIT) \
    $(wildcard include/config/FUNCTION_GRAPH_TRACER) \
    $(wildcard include/config/BCACHE) \
    $(wildcard include/config/LIVEPATCH) \
    $(wildcard include/config/SECURITY) \
    $(wildcard include/config/BPF_SYSCALL) \
    $(wildcard include/config/GCC_PLUGIN_STACKLEAK) \
    $(wildcard include/config/X86_MCE) \
    $(wildcard include/config/KRETPROBES) \
    $(wildcard include/config/RETHOOK) \
    $(wildcard include/config/ARCH_HAS_PARANOID_L1D_FLUSH) \
    $(wildcard include/config/RV) \
    $(wildcard include/config/ARCH_TASK_STRUCT_ON_STACK) \
    $(wildcard include/config/PREEMPT_NONE) \
    $(wildcard include/config/PREEMPT_VOLUNTARY) \
    $(wildcard include/config/DEBUG_RSEQ) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/uapi/linux/sched.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/pid.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/rculist.h \
    $(wildcard include/config/PROVE_RCU_LIST) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/sem.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/uapi/linux/sem.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/ipc.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/uidgid.h \
    $(wildcard include/config/MULTIUSER) \
    $(wildcard include/config/USER_NS) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/highuid.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/rhashtable-types.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/uapi/linux/ipc.h \
  arch/arm/include/generated/uapi/asm/ipcbuf.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/uapi/asm-generic/ipcbuf.h \
  arch/arm/include/generated/uapi/asm/sembuf.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/uapi/asm-generic/sembuf.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/shm.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/uapi/linux/shm.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/uapi/asm-generic/hugetlb_encode.h \
  arch/arm/include/generated/uapi/asm/shmbuf.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/uapi/asm-generic/shmbuf.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/arch/arm/include/asm/shmparam.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/kmsan_types.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/plist.h \
    $(wildcard include/config/DEBUG_PLIST) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/hrtimer.h \
    $(wildcard include/config/HIGH_RES_TIMERS) \
    $(wildcard include/config/TIME_LOW_RES) \
    $(wildcard include/config/TIMERFD) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/hrtimer_defs.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/timerqueue.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/seccomp.h \
    $(wildcard include/config/SECCOMP) \
    $(wildcard include/config/HAVE_ARCH_SECCOMP_FILTER) \
    $(wildcard include/config/SECCOMP_FILTER) \
    $(wildcard include/config/CHECKPOINT_RESTORE) \
    $(wildcard include/config/SECCOMP_CACHE_DEBUG) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/uapi/linux/seccomp.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/arch/arm/include/asm/seccomp.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/asm-generic/seccomp.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/uapi/linux/unistd.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/arch/arm/include/asm/unistd.h \
    $(wildcard include/config/OABI_COMPAT) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/arch/arm/include/uapi/asm/unistd.h \
  arch/arm/include/generated/uapi/asm/unistd-eabi.h \
  arch/arm/include/generated/asm/unistd-nr.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/resource.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/uapi/linux/resource.h \
  arch/arm/include/generated/uapi/asm/resource.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/asm-generic/resource.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/uapi/asm-generic/resource.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/latencytop.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/sched/prio.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/sched/types.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/signal_types.h \
    $(wildcard include/config/OLD_SIGACTION) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/uapi/linux/signal.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/arch/arm/include/asm/signal.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/arch/arm/include/uapi/asm/signal.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/uapi/asm-generic/signal-defs.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/arch/arm/include/uapi/asm/sigcontext.h \
  arch/arm/include/generated/uapi/asm/siginfo.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/uapi/asm-generic/siginfo.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/syscall_user_dispatch.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/task_io_accounting.h \
    $(wildcard include/config/TASK_IO_ACCOUNTING) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/posix-timers.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/alarmtimer.h \
    $(wildcard include/config/RTC_CLASS) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/uapi/linux/rseq.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/kcsan.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/rv.h \
    $(wildcard include/config/RV_REACTORS) \
  arch/arm/include/generated/asm/kmap_size.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/asm-generic/kmap_size.h \
    $(wildcard include/config/DEBUG_KMAP_LOCAL) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/sync_core.h \
    $(wildcard include/config/ARCH_HAS_SYNC_CORE_BEFORE_USERMODE) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/ioasid.h \
    $(wildcard include/config/IOASID) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/kobject_ns.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/stat.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/arch/arm/include/uapi/asm/stat.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/uapi/linux/stat.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/mod_devicetable.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/uuid.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/uapi/linux/uuid.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/property.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/fwnode.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/resource_ext.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/slab.h \
    $(wildcard include/config/DEBUG_SLAB) \
    $(wildcard include/config/FAILSLAB) \
    $(wildcard include/config/MEMCG_KMEM) \
    $(wildcard include/config/KFENCE) \
    $(wildcard include/config/SLAB) \
    $(wildcard include/config/SLUB) \
    $(wildcard include/config/SLOB) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/overflow.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/percpu-refcount.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/kasan.h \
    $(wildcard include/config/KASAN_STACK) \
    $(wildcard include/config/KASAN_VMALLOC) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/kasan-enabled.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/static_key.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/device.h \
    $(wildcard include/config/GENERIC_MSI_IRQ_DOMAIN) \
    $(wildcard include/config/GENERIC_MSI_IRQ) \
    $(wildcard include/config/ENERGY_MODEL) \
    $(wildcard include/config/PINCTRL) \
    $(wildcard include/config/DMA_OPS) \
    $(wildcard include/config/DMA_DECLARE_COHERENT) \
    $(wildcard include/config/DMA_CMA) \
    $(wildcard include/config/SWIOTLB) \
    $(wildcard include/config/ARCH_HAS_SYNC_DMA_FOR_DEVICE) \
    $(wildcard include/config/ARCH_HAS_SYNC_DMA_FOR_CPU) \
    $(wildcard include/config/ARCH_HAS_SYNC_DMA_FOR_CPU_ALL) \
    $(wildcard include/config/DMA_OPS_BYPASS) \
    $(wildcard include/config/DEVTMPFS) \
    $(wildcard include/config/SYSFS_DEPRECATED) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/dev_printk.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/ratelimit.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/energy_model.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/sched/cpufreq.h \
    $(wildcard include/config/CPU_FREQ) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/sched/topology.h \
    $(wildcard include/config/SCHED_DEBUG) \
    $(wildcard include/config/SCHED_CLUSTER) \
    $(wildcard include/config/SCHED_MC) \
    $(wildcard include/config/CPU_FREQ_GOV_SCHEDUTIL) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/sched/idle.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/klist.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/pm.h \
    $(wildcard include/config/VT_CONSOLE_SLEEP) \
    $(wildcard include/config/CXL_SUSPEND) \
    $(wildcard include/config/PM_CLK) \
    $(wildcard include/config/PM_GENERIC_DOMAINS) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/device/bus.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/device/class.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/device/driver.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/module.h \
    $(wildcard include/config/MODULES_TREE_LOOKUP) \
    $(wildcard include/config/STACKTRACE_BUILD_ID) \
    $(wildcard include/config/ARCH_USES_CFI_TRAPS) \
    $(wildcard include/config/MODULE_SIG) \
    $(wildcard include/config/ARCH_WANTS_MODULES_DATA_IN_VMALLOC) \
    $(wildcard include/config/KALLSYMS) \
    $(wildcard include/config/TRACEPOINTS) \
    $(wildcard include/config/BPF_EVENTS) \
    $(wildcard include/config/EVENT_TRACING) \
    $(wildcard include/config/KPROBES) \
    $(wildcard include/config/MODULE_UNLOAD) \
    $(wildcard include/config/CONSTRUCTORS) \
    $(wildcard include/config/FUNCTION_ERROR_INJECTION) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/buildid.h \
    $(wildcard include/config/CRASH_CORE) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/kmod.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/umh.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/sysctl.h \
    $(wildcard include/config/SYSCTL) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/uapi/linux/sysctl.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/elf.h \
    $(wildcard include/config/ARCH_USE_GNU_PROPERTY) \
    $(wildcard include/config/ARCH_HAVE_ELF_PROT) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/arch/arm/include/asm/elf.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/arch/arm/include/asm/vdso_datapage.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/vdso/datapage.h \
    $(wildcard include/config/ARCH_HAS_VDSO_DATA) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/uapi/asm-generic/errno-base.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/vdso/clocksource.h \
    $(wildcard include/config/GENERIC_GETTIMEOFDAY) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/vdso/processor.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/arch/arm/include/asm/vdso/gettimeofday.h \
    $(wildcard include/config/ARM_ARCH_TIMER) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/arch/arm/include/asm/vdso/cp15.h \
    $(wildcard include/config/CPU_CP15) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/arch/arm/include/asm/user.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/uapi/linux/elf.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/uapi/linux/elf-em.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/moduleparam.h \
    $(wildcard include/config/ALPHA) \
    $(wildcard include/config/PPC64) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/rbtree_latch.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/error-injection.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/asm-generic/error-injection.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/tracepoint-defs.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/arch/arm/include/asm/module.h \
    $(wildcard include/config/ARM_UNWIND) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/asm-generic/module.h \
    $(wildcard include/config/HAVE_MOD_ARCH_SPECIFIC) \
    $(wildcard include/config/MODULES_USE_ELF_REL) \
    $(wildcard include/config/MODULES_USE_ELF_RELA) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/arch/arm/include/asm/unwind.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/arch/arm/include/asm/device.h \
    $(wildcard include/config/ARM_DMA_USE_IOMMU) \
    $(wildcard include/config/ARCH_OMAP) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/pm_wakeup.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/acpi/acpi.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/acpi/platform/acenv.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/acpi/platform/acgcc.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/acpi/platform/aclinux.h \
    $(wildcard include/config/ACPI_REDUCED_HARDWARE_ONLY) \
    $(wildcard include/config/ACPI_DEBUG) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/ctype.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/acpi/acnames.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/acpi/actypes.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/acpi/acexcep.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/acpi/actbl.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/acpi/actbl1.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/acpi/actbl2.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/acpi/actbl3.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/acpi/acrestyp.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/acpi/platform/acenvex.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/acpi/platform/aclinuxex.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/acpi/platform/acgccex.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/acpi/acoutput.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/acpi/acpiosxf.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/acpi/acpixf.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/acpi/acconfig.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/acpi/acbuffer.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/acpi/acpi_numa.h \
    $(wildcard include/config/ACPI_HMAT) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/regulator/consumer.h \
    $(wildcard include/config/REGULATOR) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/suspend.h \
    $(wildcard include/config/VT) \
    $(wildcard include/config/SUSPEND) \
    $(wildcard include/config/HIBERNATION_SNAPSHOT_DEV) \
    $(wildcard include/config/PM_SLEEP_DEBUG) \
    $(wildcard include/config/PM_AUTOSLEEP) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/swap.h \
    $(wildcard include/config/PTE_MARKER) \
    $(wildcard include/config/DEVICE_PRIVATE) \
    $(wildcard include/config/MIGRATION) \
    $(wildcard include/config/FRONTSWAP) \
    $(wildcard include/config/THP_SWAP) \
    $(wildcard include/config/ZSWAP) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/memcontrol.h \
    $(wildcard include/config/CGROUP_WRITEBACK) \
    $(wildcard include/config/SHRINKER_DEBUG) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/cgroup.h \
    $(wildcard include/config/CGROUP_CPUACCT) \
    $(wildcard include/config/SOCK_CGROUP_DATA) \
    $(wildcard include/config/CGROUP_DATA) \
    $(wildcard include/config/CGROUP_BPF) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/uapi/linux/cgroupstats.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/uapi/linux/taskstats.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/fs.h \
    $(wildcard include/config/READ_ONLY_THP_FOR_FS) \
    $(wildcard include/config/FS_POSIX_ACL) \
    $(wildcard include/config/IMA) \
    $(wildcard include/config/FILE_LOCKING) \
    $(wildcard include/config/FSNOTIFY) \
    $(wildcard include/config/FS_ENCRYPTION) \
    $(wildcard include/config/FS_VERITY) \
    $(wildcard include/config/EPOLL) \
    $(wildcard include/config/UNICODE) \
    $(wildcard include/config/QUOTA) \
    $(wildcard include/config/FS_DAX) \
    $(wildcard include/config/BLOCK) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/wait_bit.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/kdev_t.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/uapi/linux/kdev_t.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/dcache.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/rculist_bl.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/list_bl.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/bit_spinlock.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/lockref.h \
    $(wildcard include/config/ARCH_USE_CMPXCHG_LOCKREF) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/stringhash.h \
    $(wildcard include/config/DCACHE_WORD_ACCESS) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/hash.h \
    $(wildcard include/config/HAVE_ARCH_HASH) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/path.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/list_lru.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/shrinker.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/capability.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/uapi/linux/capability.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/semaphore.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/fcntl.h \
    $(wildcard include/config/ARCH_32BIT_OFF_T) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/uapi/linux/fcntl.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/arch/arm/include/uapi/asm/fcntl.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/uapi/asm-generic/fcntl.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/uapi/linux/openat2.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/migrate_mode.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/percpu-rwsem.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/rcuwait.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/sched/signal.h \
    $(wildcard include/config/SCHED_AUTOGROUP) \
    $(wildcard include/config/BSD_PROCESS_ACCT) \
    $(wildcard include/config/TASKSTATS) \
    $(wildcard include/config/STACK_GROWSUP) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/signal.h \
    $(wildcard include/config/DYNAMIC_SIGFRAME) \
    $(wildcard include/config/PROC_FS) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/sched/jobctl.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/sched/task.h \
    $(wildcard include/config/HAVE_EXIT_THREAD) \
    $(wildcard include/config/ARCH_WANTS_DYNAMIC_TASK_STRUCT) \
    $(wildcard include/config/HAVE_ARCH_THREAD_STRUCT_WHITELIST) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/uaccess.h \
    $(wildcard include/config/ARCH_HAS_SUBPAGE_FAULTS) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/fault-inject-usercopy.h \
    $(wildcard include/config/FAULT_INJECTION_USERCOPY) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/arch/arm/include/asm/uaccess.h \
    $(wildcard include/config/CPU_SW_DOMAIN_PAN) \
    $(wildcard include/config/CPU_USE_DOMAINS) \
    $(wildcard include/config/HAVE_EFFICIENT_UNALIGNED_ACCESS) \
    $(wildcard include/config/UACCESS_WITH_MEMCPY) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/arch/arm/include/asm/domain.h \
    $(wildcard include/config/IO_36) \
    $(wildcard include/config/CPU_CP15_MMU) \
  arch/arm/include/generated/asm/unaligned.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/asm-generic/unaligned.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/unaligned/packed_struct.h \
  arch/arm/include/generated/asm/extable.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/asm-generic/extable.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/asm-generic/access_ok.h \
    $(wildcard include/config/ALTERNATE_USER_ADDRESS_SPACE) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/cred.h \
    $(wildcard include/config/DEBUG_CREDENTIALS) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/key.h \
    $(wildcard include/config/KEY_NOTIFICATIONS) \
    $(wildcard include/config/NET) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/assoc_array.h \
    $(wildcard include/config/ASSOCIATIVE_ARRAY) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/sched/user.h \
    $(wildcard include/config/VFIO_PCI_ZDEV_KVM) \
    $(wildcard include/config/WATCH_QUEUE) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/percpu_counter.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/rcu_sync.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/delayed_call.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/errseq.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/ioprio.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/sched/rt.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/iocontext.h \
    $(wildcard include/config/BLK_ICQ) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/uapi/linux/ioprio.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/fs_types.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/mount.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/mnt_idmapping.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/uapi/linux/fs.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/quota.h \
    $(wildcard include/config/QUOTA_NETLINK_INTERFACE) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/uapi/linux/dqblk_xfs.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/dqblk_v1.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/dqblk_v2.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/dqblk_qtree.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/projid.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/uapi/linux/quota.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/nfs_fs_i.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/seq_file.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/string_helpers.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/ns_common.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/nsproxy.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/user_namespace.h \
    $(wildcard include/config/INOTIFY_USER) \
    $(wildcard include/config/FANOTIFY) \
    $(wildcard include/config/PERSISTENT_KEYRINGS) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/kernel_stat.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/interrupt.h \
    $(wildcard include/config/IRQ_FORCED_THREADING) \
    $(wildcard include/config/GENERIC_IRQ_PROBE) \
    $(wildcard include/config/IRQ_TIMINGS) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/irqreturn.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/hardirq.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/context_tracking_state.h \
    $(wildcard include/config/CONTEXT_TRACKING_USER) \
    $(wildcard include/config/CONTEXT_TRACKING) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/ftrace_irq.h \
    $(wildcard include/config/HWLAT_TRACER) \
    $(wildcard include/config/OSNOISE_TRACER) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/vtime.h \
    $(wildcard include/config/VIRT_CPU_ACCOUNTING) \
    $(wildcard include/config/IRQ_TIME_ACCOUNTING) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/arch/arm/include/asm/hardirq.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/arch/arm/include/asm/irq.h \
    $(wildcard include/config/SPARSE_IRQ) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/asm-generic/hardirq.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/irq.h \
    $(wildcard include/config/GENERIC_IRQ_EFFECTIVE_AFF_MASK) \
    $(wildcard include/config/GENERIC_IRQ_IPI) \
    $(wildcard include/config/DEPRECATED_IRQ_CPU_ONOFFLINE) \
    $(wildcard include/config/GENERIC_IRQ_MIGRATION) \
    $(wildcard include/config/GENERIC_PENDING_IRQ) \
    $(wildcard include/config/HARDIRQS_SW_RESEND) \
    $(wildcard include/config/GENERIC_IRQ_LEGACY) \
    $(wildcard include/config/GENERIC_IRQ_MULTI_HANDLER) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/io.h \
    $(wildcard include/config/HAS_IOPORT_MAP) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/arch/arm/include/asm/io.h \
    $(wildcard include/config/PCMCIA) \
    $(wildcard include/config/NEED_MACH_IO_H) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/asm-generic/pci_iomap.h \
    $(wildcard include/config/NO_GENERIC_PCI_IOPORT_MAP) \
    $(wildcard include/config/GENERIC_PCI_IOMAP) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/asm-generic/io.h \
    $(wildcard include/config/GENERIC_IOMAP) \
    $(wildcard include/config/TRACE_MMIO_ACCESS) \
    $(wildcard include/config/GENERIC_IOREMAP) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/logic_pio.h \
    $(wildcard include/config/INDIRECT_PIO) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/vmalloc.h \
    $(wildcard include/config/HAVE_ARCH_HUGE_VMALLOC) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/arch/arm/include/asm/vmalloc.h \
  arch/arm/include/generated/asm/irq_regs.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/asm-generic/irq_regs.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/irqdesc.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/arch/arm/include/asm/hw_irq.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/arch/arm/include/asm/sections.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/asm-generic/sections.h \
    $(wildcard include/config/HAVE_FUNCTION_DESCRIPTORS) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/cgroup-defs.h \
    $(wildcard include/config/CGROUP_NET_CLASSID) \
    $(wildcard include/config/CGROUP_NET_PRIO) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/u64_stats_sync.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/bpf-cgroup-defs.h \
    $(wildcard include/config/BPF_LSM) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/psi_types.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/kthread.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/cgroup_subsys.h \
    $(wildcard include/config/CGROUP_DEVICE) \
    $(wildcard include/config/CGROUP_FREEZER) \
    $(wildcard include/config/CGROUP_PERF) \
    $(wildcard include/config/CGROUP_HUGETLB) \
    $(wildcard include/config/CGROUP_PIDS) \
    $(wildcard include/config/CGROUP_RDMA) \
    $(wildcard include/config/CGROUP_MISC) \
    $(wildcard include/config/CGROUP_DEBUG) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/vm_event_item.h \
    $(wildcard include/config/HAVE_ARCH_TRANSPARENT_HUGEPAGE_PUD) \
    $(wildcard include/config/MEMORY_BALLOON) \
    $(wildcard include/config/BALLOON_COMPACTION) \
    $(wildcard include/config/DEBUG_TLBFLUSH) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/page_counter.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/vmpressure.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/eventfd.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/mm.h \
    $(wildcard include/config/HAVE_ARCH_MMAP_RND_BITS) \
    $(wildcard include/config/HAVE_ARCH_MMAP_RND_COMPAT_BITS) \
    $(wildcard include/config/MEM_SOFT_DIRTY) \
    $(wildcard include/config/ARCH_USES_HIGH_VMA_FLAGS) \
    $(wildcard include/config/ARCH_HAS_PKEYS) \
    $(wildcard include/config/PPC) \
    $(wildcard include/config/PARISC) \
    $(wildcard include/config/SPARC64) \
    $(wildcard include/config/ARM64_MTE) \
    $(wildcard include/config/HAVE_ARCH_USERFAULTFD_MINOR) \
    $(wildcard include/config/SHMEM) \
    $(wildcard include/config/ARCH_HAS_PTE_SPECIAL) \
    $(wildcard include/config/ARCH_HAS_PTE_DEVMAP) \
    $(wildcard include/config/DEBUG_VM_RB) \
    $(wildcard include/config/HAVE_FAST_GUP) \
    $(wildcard include/config/PAGE_POISONING) \
    $(wildcard include/config/INIT_ON_ALLOC_DEFAULT_ON) \
    $(wildcard include/config/INIT_ON_FREE_DEFAULT_ON) \
    $(wildcard include/config/DEBUG_PAGEALLOC) \
    $(wildcard include/config/HUGETLBFS) \
    $(wildcard include/config/MAPPING_DIRTY_HELPERS) \
    $(wildcard include/config/ANON_VMA_NAME) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/mmap_lock.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/range.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/page_ext.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/stacktrace.h \
    $(wildcard include/config/ARCH_STACKWALK) \
    $(wildcard include/config/STACKTRACE) \
    $(wildcard include/config/HAVE_RELIABLE_STACKTRACE) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/stackdepot.h \
    $(wildcard include/config/STACKDEPOT) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/page_ref.h \
    $(wildcard include/config/DEBUG_PAGE_REF) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/pgtable.h \
    $(wildcard include/config/PGTABLE_LEVELS) \
    $(wildcard include/config/HIGHPTE) \
    $(wildcard include/config/ARCH_HAS_NONLEAF_PMD_YOUNG) \
    $(wildcard include/config/GUP_GET_PTE_LOW_HIGH) \
    $(wildcard include/config/HAVE_ARCH_SOFT_DIRTY) \
    $(wildcard include/config/ARCH_ENABLE_THP_MIGRATION) \
    $(wildcard include/config/HAVE_ARCH_HUGE_VMAP) \
    $(wildcard include/config/X86_ESPFIX64) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/arch/arm/include/asm/pgtable.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/arch/arm/include/asm/proc-fns.h \
    $(wildcard include/config/BIG_LITTLE) \
    $(wildcard include/config/HARDEN_BRANCH_PREDICTOR) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/arch/arm/include/asm/glue-proc.h \
    $(wildcard include/config/CPU_ARM7TDMI) \
    $(wildcard include/config/CPU_ARM720T) \
    $(wildcard include/config/CPU_ARM740T) \
    $(wildcard include/config/CPU_ARM9TDMI) \
    $(wildcard include/config/CPU_ARM920T) \
    $(wildcard include/config/CPU_ARM922T) \
    $(wildcard include/config/CPU_ARM925T) \
    $(wildcard include/config/CPU_ARM926T) \
    $(wildcard include/config/CPU_ARM940T) \
    $(wildcard include/config/CPU_ARM946E) \
    $(wildcard include/config/CPU_ARM1020) \
    $(wildcard include/config/CPU_ARM1020E) \
    $(wildcard include/config/CPU_ARM1022) \
    $(wildcard include/config/CPU_ARM1026) \
    $(wildcard include/config/CPU_MOHAWK) \
    $(wildcard include/config/CPU_FEROCEON) \
    $(wildcard include/config/CPU_V6K) \
    $(wildcard include/config/CPU_PJ4B) \
    $(wildcard include/config/CPU_V7) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/asm-generic/pgtable-nopud.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/asm-generic/pgtable-nop4d.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/arch/arm/include/asm/pgtable-hwdef.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/arch/arm/include/asm/pgtable-2level-hwdef.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/arch/arm/include/asm/tlbflush.h \
    $(wildcard include/config/SMP_ON_UP) \
    $(wildcard include/config/CPU_TLB_V4WT) \
    $(wildcard include/config/CPU_TLB_FA) \
    $(wildcard include/config/CPU_TLB_V4WBI) \
    $(wildcard include/config/CPU_TLB_FEROCEON) \
    $(wildcard include/config/CPU_TLB_V4WB) \
    $(wildcard include/config/CPU_TLB_V6) \
    $(wildcard include/config/CPU_TLB_V7) \
    $(wildcard include/config/ARM_ERRATA_720789) \
    $(wildcard include/config/ARM_ERRATA_798181) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/arch/arm/include/asm/pgtable-2level.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/asm-generic/pgtable_uffd.h \
    $(wildcard include/config/HAVE_ARCH_USERFAULTFD_WP) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/page_table_check.h \
    $(wildcard include/config/PAGE_TABLE_CHECK) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/memremap.h \
    $(wildcard include/config/PCI_P2PDMA) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/huge_mm.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/sched/coredump.h \
    $(wildcard include/config/CORE_DUMP_DEFAULT_ELF_HEADERS) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/vmstat.h \
    $(wildcard include/config/VM_EVENT_COUNTERS) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/writeback.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/flex_proportions.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/backing-dev-defs.h \
    $(wildcard include/config/DEBUG_FS) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/blk_types.h \
    $(wildcard include/config/FAIL_MAKE_REQUEST) \
    $(wildcard include/config/BLK_CGROUP_IOCOST) \
    $(wildcard include/config/BLK_INLINE_ENCRYPTION) \
    $(wildcard include/config/BLK_DEV_INTEGRITY) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/bvec.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/highmem.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/cacheflush.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/arch/arm/include/asm/cacheflush.h \
    $(wildcard include/config/ARM_ERRATA_411920) \
    $(wildcard include/config/CPU_CACHE_VIPT) \
    $(wildcard include/config/OUTER_CACHE) \
    $(wildcard include/config/CPU_ICACHE_MISMATCH_WORKAROUND) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/arch/arm/include/asm/glue-cache.h \
    $(wildcard include/config/CPU_CACHE_V4) \
    $(wildcard include/config/CPU_CACHE_V4WB) \
    $(wildcard include/config/CACHE_B15_RAC) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/arch/arm/include/asm/cachetype.h \
    $(wildcard include/config/CPU_CACHE_VIVT) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/arch/arm/include/asm/outercache.h \
    $(wildcard include/config/OUTER_CACHE_SYNC) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/kmsan.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/dma-direction.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/highmem-internal.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/bio.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/mempool.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/uio.h \
    $(wildcard include/config/ARCH_HAS_UACCESS_FLUSHCACHE) \
    $(wildcard include/config/ARCH_HAS_COPY_MC) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/uapi/linux/uio.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/node.h \
    $(wildcard include/config/HMEM_REPORTING) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/pagemap.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/hugetlb_inline.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/uapi/linux/mempolicy.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/freezer.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/rtmutex.h \
    $(wildcard include/config/DEBUG_RT_MUTEXES) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/uapi/linux/i2c.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/gpio.h \
    $(wildcard include/config/ARCH_HAVE_CUSTOM_GPIO_H) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/arch/arm/include/asm/gpio.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/asm-generic/gpio.h \
    $(wildcard include/config/ARCH_NR_GPIO) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/gpio/driver.h \
    $(wildcard include/config/GPIO_GENERIC) \
    $(wildcard include/config/GPIOLIB_IRQCHIP) \
    $(wildcard include/config/OF_GPIO) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/irqchip/chained_irq.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/pinctrl/pinctrl.h \
    $(wildcard include/config/GENERIC_PINCONF) \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/pinctrl/pinctrl-state.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/pinctrl/devinfo.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/pinctrl/consumer.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/pinctrl/pinconf-generic.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/pinctrl/machine.h \
  arch/arm/include/generated/asm/msi.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/asm-generic/msi.h \
  /usr/src/linux-headers-6.1.0-rpi7-common-rpi/include/linux/gpio/consumer.h \
    $(wildcard include/config/GPIO_SYSFS) \
  /home/ams/workspace/mf8806_tmf8806-linux-driver_main/core_driver/tmf8806_regs.h \
  /home/ams/workspace/mf8806_tmf8806-linux-driver_main/ams_i2c.h \
  /home/ams/workspace/mf8806_tmf8806-linux-driver_main/core_driver/tmf8806.h \
  /home/ams/workspace/mf8806_tmf8806-linux-driver_main/./core_driver/tmf8806_regs.h \

/home/ams/workspace/mf8806_tmf8806-linux-driver_main/./core_driver/tmf8806.o: $(deps_/home/ams/workspace/mf8806_tmf8806-linux-driver_main/./core_driver/tmf8806.o)

$(deps_/home/ams/workspace/mf8806_tmf8806-linux-driver_main/./core_driver/tmf8806.o):
