






typedef __builtin_va_list __gnuc_va_list;
typedef __gnuc_va_list va_list;



struct ftrace_branch_data {
 const char *func;
 const char *file;
 unsigned line;
 union {
  struct {
   unsigned long correct;
   unsigned long incorrect;
  };
  struct {
   unsigned long miss;
   unsigned long hit;
  };
  unsigned long miss_hit[2];
 };
};
struct kernel_symbol
{
 unsigned long value;
 const char *name;
};


extern struct module __this_module;








enum {
 false = 0,
 true = 1
};

























typedef __signed__ char __s8;
typedef unsigned char __u8;

typedef __signed__ short __s16;
typedef unsigned short __u16;

typedef __signed__ int __s32;
typedef unsigned int __u32;


__extension__ typedef __signed__ long long __s64;
__extension__ typedef unsigned long long __u64;




typedef signed char s8;
typedef unsigned char u8;

typedef signed short s16;
typedef unsigned short u16;

typedef signed int s32;
typedef unsigned int u32;

typedef signed long long s64;
typedef unsigned long long u64;
typedef struct {
 unsigned long fds_bits[1024 / (8 * sizeof(long))];
} __kernel_fd_set;


typedef void (*__kernel_sighandler_t)(int);


typedef int __kernel_key_t;
typedef int __kernel_mqd_t;




typedef unsigned short __kernel_old_uid_t;
typedef unsigned short __kernel_old_gid_t;


typedef unsigned long __kernel_old_dev_t;


typedef long __kernel_long_t;
typedef unsigned long __kernel_ulong_t;



typedef __kernel_ulong_t __kernel_ino_t;



typedef unsigned int __kernel_mode_t;



typedef int __kernel_pid_t;



typedef int __kernel_ipc_pid_t;



typedef unsigned int __kernel_uid_t;
typedef unsigned int __kernel_gid_t;



typedef __kernel_long_t __kernel_suseconds_t;



typedef int __kernel_daddr_t;



typedef unsigned int __kernel_uid32_t;
typedef unsigned int __kernel_gid32_t;
typedef __kernel_ulong_t __kernel_size_t;
typedef __kernel_long_t __kernel_ssize_t;
typedef __kernel_long_t __kernel_ptrdiff_t;




typedef struct {
 int val[2];
} __kernel_fsid_t;





typedef __kernel_long_t __kernel_off_t;
typedef long long __kernel_loff_t;
typedef __kernel_long_t __kernel_time_t;
typedef __kernel_long_t __kernel_clock_t;
typedef int __kernel_timer_t;
typedef int __kernel_clockid_t;
typedef char * __kernel_caddr_t;
typedef unsigned short __kernel_uid16_t;
typedef unsigned short __kernel_gid16_t;
typedef __u16 __le16;
typedef __u16 __be16;
typedef __u32 __le32;
typedef __u32 __be32;
typedef __u64 __le64;
typedef __u64 __be64;

typedef __u16 __sum16;
typedef __u32 __wsum;






typedef __u32 __kernel_dev_t;

typedef __kernel_fd_set fd_set;
typedef __kernel_dev_t dev_t;
typedef __kernel_ino_t ino_t;
typedef __kernel_mode_t mode_t;
typedef unsigned short umode_t;
typedef __u32 nlink_t;
typedef __kernel_off_t off_t;
typedef __kernel_pid_t pid_t;
typedef __kernel_daddr_t daddr_t;
typedef __kernel_key_t key_t;
typedef __kernel_suseconds_t suseconds_t;
typedef __kernel_timer_t timer_t;
typedef __kernel_clockid_t clockid_t;
typedef __kernel_mqd_t mqd_t;

typedef _Bool bool;

typedef __kernel_uid32_t uid_t;
typedef __kernel_gid32_t gid_t;
typedef __kernel_uid16_t uid16_t;
typedef __kernel_gid16_t gid16_t;

typedef unsigned long uintptr_t;



typedef __kernel_old_uid_t old_uid_t;
typedef __kernel_old_gid_t old_gid_t;



typedef __kernel_loff_t loff_t;
typedef __kernel_size_t size_t;




typedef __kernel_ssize_t ssize_t;




typedef __kernel_ptrdiff_t ptrdiff_t;




typedef __kernel_time_t time_t;




typedef __kernel_clock_t clock_t;




typedef __kernel_caddr_t caddr_t;



typedef unsigned char u_char;
typedef unsigned short u_short;
typedef unsigned int u_int;
typedef unsigned long u_long;


typedef unsigned char unchar;
typedef unsigned short ushort;
typedef unsigned int uint;
typedef unsigned long ulong;




typedef __u8 u_int8_t;
typedef __s8 int8_t;
typedef __u16 u_int16_t;
typedef __s16 int16_t;
typedef __u32 u_int32_t;
typedef __s32 int32_t;



typedef __u8 uint8_t;
typedef __u16 uint16_t;
typedef __u32 uint32_t;


typedef __u64 uint64_t;
typedef __u64 u_int64_t;
typedef __s64 int64_t;
typedef unsigned long sector_t;
typedef unsigned long blkcnt_t;
typedef u64 dma_addr_t;
typedef unsigned gfp_t;
typedef unsigned fmode_t;
typedef unsigned oom_flags_t;


typedef u64 phys_addr_t;




typedef phys_addr_t resource_size_t;





typedef unsigned long irq_hw_number_t;

typedef struct {
 int counter;
} atomic_t;


typedef struct {
 long counter;
} atomic64_t;


struct list_head {
 struct list_head *next, *prev;
};

struct hlist_head {
 struct hlist_node *first;
};

struct hlist_node {
 struct hlist_node *next, **pprev;
};

struct ustat {
 __kernel_daddr_t f_tfree;
 __kernel_ino_t f_tinode;
 char f_fname[6];
 char f_fpack[6];
};






struct callback_head {
 struct callback_head *next;
 void (*func)(struct callback_head *head);
};

extern unsigned int __sw_hweight8(unsigned int w);
extern unsigned int __sw_hweight16(unsigned int w);
extern unsigned int __sw_hweight32(unsigned int w);
extern unsigned long __sw_hweight64(__u64 w);

















extern const char early_idt_handlers[32][2+2+5];
static inline __attribute__((no_instrument_function)) unsigned long get_limit(unsigned long segment)
{
 unsigned long __limit;
 asm("lsll %1,%0" : "=r" (__limit) : "r" (segment));
 return __limit + 1;
}






extern int devmem_is_allowed(unsigned long pagenr);

extern unsigned long max_low_pfn_mapped;
extern unsigned long max_pfn_mapped;

static inline __attribute__((no_instrument_function)) phys_addr_t get_max_mapped(void)
{
 return (phys_addr_t)max_pfn_mapped << 12;
}

bool pfn_range_is_mapped(unsigned long start_pfn, unsigned long end_pfn);

extern unsigned long init_memory_mapping(unsigned long start,
      unsigned long end);

extern void initmem_init(void);







struct pt_regs {
 unsigned long r15;
 unsigned long r14;
 unsigned long r13;
 unsigned long r12;
 unsigned long bp;
 unsigned long bx;

 unsigned long r11;
 unsigned long r10;
 unsigned long r9;
 unsigned long r8;
 unsigned long ax;
 unsigned long cx;
 unsigned long dx;
 unsigned long si;
 unsigned long di;
 unsigned long orig_ax;


 unsigned long ip;
 unsigned long cs;
 unsigned long flags;
 unsigned long sp;
 unsigned long ss;

};







struct cpuinfo_x86;
struct task_struct;

extern unsigned long profile_pc(struct pt_regs *regs);


extern unsigned long
convert_ip_to_linear(struct task_struct *child, struct pt_regs *regs);
extern void send_sigtrap(struct task_struct *tsk, struct pt_regs *regs,
    int error_code, int si_code);

extern long syscall_trace_enter(struct pt_regs *);
extern void syscall_trace_leave(struct pt_regs *);

static inline __attribute__((no_instrument_function)) unsigned long regs_return_value(struct pt_regs *regs)
{
 return regs->ax;
}
static inline __attribute__((no_instrument_function)) int user_mode(struct pt_regs *regs)
{



 return !!(regs->cs & 3);

}

static inline __attribute__((no_instrument_function)) int user_mode_vm(struct pt_regs *regs)
{




 return user_mode(regs);

}

static inline __attribute__((no_instrument_function)) int v8086_mode(struct pt_regs *regs)
{



 return 0;

}


static inline __attribute__((no_instrument_function)) bool user_64bit_mode(struct pt_regs *regs)
{





 return regs->cs == (6*8+3);




}
static inline __attribute__((no_instrument_function)) unsigned long kernel_stack_pointer(struct pt_regs *regs)
{
 return regs->sp;
}






static inline __attribute__((no_instrument_function)) unsigned long instruction_pointer(struct pt_regs *regs)
{
 return ((regs)->ip);
}
static inline __attribute__((no_instrument_function)) void instruction_pointer_set(struct pt_regs *regs,
                                           unsigned long val)
{
 (((regs)->ip) = (val));
}
static inline __attribute__((no_instrument_function)) unsigned long user_stack_pointer(struct pt_regs *regs)
{
 return ((regs)->sp);
}
static inline __attribute__((no_instrument_function)) void user_stack_pointer_set(struct pt_regs *regs,
                                          unsigned long val)
{
 (((regs)->sp) = (val));
}
static inline __attribute__((no_instrument_function)) unsigned long frame_pointer(struct pt_regs *regs)
{
 return ((regs)->bp);
}
static inline __attribute__((no_instrument_function)) void frame_pointer_set(struct pt_regs *regs,
                                     unsigned long val)
{
 (((regs)->bp) = (val));
}


extern int regs_query_register_offset(const char *name);
extern const char *regs_query_register_name(unsigned int offset);
static inline __attribute__((no_instrument_function)) unsigned long regs_get_register(struct pt_regs *regs,
           unsigned int offset)
{
 if (__builtin_expect(!!(offset > (__builtin_offsetof(struct pt_regs,ss))), 0))
  return 0;
 return *(unsigned long *)((unsigned long)regs + offset);
}
static inline __attribute__((no_instrument_function)) int regs_within_kernel_stack(struct pt_regs *regs,
        unsigned long addr)
{
 return ((addr & ~((((1UL) << 12) << 2) - 1)) ==
  (kernel_stack_pointer(regs) & ~((((1UL) << 12) << 2) - 1)));
}
static inline __attribute__((no_instrument_function)) unsigned long regs_get_kernel_stack_nth(struct pt_regs *regs,
            unsigned int n)
{
 unsigned long *addr = (unsigned long *)kernel_stack_pointer(regs);
 addr += n;
 if (regs_within_kernel_stack(regs, (unsigned long)addr))
  return *addr;
 else
  return 0;
}
struct user_desc;
extern int do_get_thread_area(struct task_struct *p, int idx,
         struct user_desc *info);
extern int do_set_thread_area(struct task_struct *p, int idx,
         struct user_desc *info, int can_allocate);
struct alt_instr {
 s32 instr_offset;
 s32 repl_offset;
 u16 cpuid;
 u8 instrlen;
 u8 replacementlen;
};

extern void alternative_instructions(void);
extern void apply_alternatives(struct alt_instr *start, struct alt_instr *end);

struct module;


extern void alternatives_smp_module_add(struct module *mod, char *name,
     void *locks, void *locks_end,
     void *text, void *text_end);
extern void alternatives_smp_module_del(struct module *mod);
extern void alternatives_enable_smp(void);
extern int alternatives_text_reserved(void *start, void *end);
extern bool skip_smp_alternatives;








extern const char * const x86_cap_flags[11*32];
extern const char * const x86_power_flags[32];





extern const char * const x86_bug_flags[1*32];
extern void warn_pre_alternatives(void);
extern bool __static_cpu_has_safe(u16 bit);






static inline __attribute__((no_instrument_function)) __attribute__((always_inline)) __attribute__((pure)) bool __static_cpu_has(u16 bit)
{
  u8 flag;

  asm volatile("1: movb $0,%0\n"
        "2:\n"
        ".section .altinstructions,\"a\"\n"
        " .long 1b - .\n"
        " .long 3f - .\n"
        " .word %P1\n"
        " .byte 2b - 1b\n"
        " .byte 4f - 3f\n"
        ".previous\n"
        ".section .discard,\"aw\",@progbits\n"
        " .byte 0xff + (4f-3f) - (2b-1b)\n"
        ".previous\n"
        ".section .altinstr_replacement,\"ax\"\n"
        "3: movb $1,%0\n"
        "4:\n"
        ".previous\n"
        : "=qm" (flag) : "i" (bit));
  return flag;


}
static inline __attribute__((no_instrument_function)) __attribute__((always_inline)) __attribute__((pure)) bool _static_cpu_has_safe(u16 bit)
{
  u8 flag;

  asm volatile("1: movb $2,%0\n"
        "2:\n"
        ".section .altinstructions,\"a\"\n"
        " .long 1b - .\n"
        " .long 3f - .\n"
        " .word %P2\n"
        " .byte 2b - 1b\n"
        " .byte 4f - 3f\n"
        ".previous\n"
        ".section .discard,\"aw\",@progbits\n"
        " .byte 0xff + (4f-3f) - (2b-1b)\n"
        ".previous\n"
        ".section .altinstr_replacement,\"ax\"\n"
        "3: movb $0,%0\n"
        "4:\n"
        ".previous\n"
        ".section .altinstructions,\"a\"\n"
        " .long 1b - .\n"
        " .long 5f - .\n"
        " .word %P1\n"
        " .byte 4b - 3b\n"
        " .byte 6f - 5f\n"
        ".previous\n"
        ".section .discard,\"aw\",@progbits\n"
        " .byte 0xff + (6f-5f) - (4b-3b)\n"
        ".previous\n"
        ".section .altinstr_replacement,\"ax\"\n"
        "5: movb $1,%0\n"
        "6:\n"
        ".previous\n"
        : "=qm" (flag)
        : "i" (bit), "i" (( 3*32+21)));
  return (flag == 2 ? __static_cpu_has_safe(bit) : flag);

}
struct paravirt_patch_site;




static inline __attribute__((no_instrument_function)) void apply_paravirt(struct paravirt_patch_site *start,
      struct paravirt_patch_site *end)
{}




extern void *text_poke_early(void *addr, const void *opcode, size_t len);
extern void *text_poke(void *addr, const void *opcode, size_t len);
extern int poke_int3_handler(struct pt_regs *regs);
extern void *text_poke_bp(void *addr, const void *opcode, size_t len, void *handler);




extern const unsigned char * const *ideal_nops;
extern void arch_init_ideal_nops(void);
static inline __attribute__((no_instrument_function)) __attribute__((always_inline)) void rdtsc_barrier(void)
{
 asm volatile ("661:\n\t" ".byte " "0x66,0x66,0x90" "\n" "\n662:\n" ".pushsection .altinstructions,\"a\"\n" " .long 661b - .\n" " .long " "663""1""f - .\n" " .word " "( 3*32+17)" "\n" " .byte " "662b-661b" "\n" " .byte " "664""1""f-""663""1""f" "\n" ".popsection\n" ".pushsection .discard,\"aw\",@progbits\n" " .byte 0xff + (" "664""1""f-""663""1""f" ") - (" "662b-661b" ")\n" ".popsection\n" ".pushsection .altinstr_replacement, \"ax\"\n" "663""1"":\n\t" "mfence" "\n" "664""1" ":\n\t" ".popsection" : : : "memory");
 asm volatile ("661:\n\t" ".byte " "0x66,0x66,0x90" "\n" "\n662:\n" ".pushsection .altinstructions,\"a\"\n" " .long 661b - .\n" " .long " "663""1""f - .\n" " .word " "( 3*32+18)" "\n" " .byte " "662b-661b" "\n" " .byte " "664""1""f-""663""1""f" "\n" ".popsection\n" ".pushsection .discard,\"aw\",@progbits\n" " .byte 0xff + (" "664""1""f-""663""1""f" ") - (" "662b-661b" ")\n" ".popsection\n" ".pushsection .altinstr_replacement, \"ax\"\n" "663""1"":\n\t" "lfence" "\n" "664""1" ":\n\t" ".popsection" : : : "memory");
}
static inline __attribute__((no_instrument_function)) __attribute__((always_inline)) void
set_bit(long nr, volatile unsigned long *addr)
{
 if ((__builtin_constant_p(nr))) {
  asm volatile(".pushsection .smp_locks,\"a\"\n" ".balign 4\n" ".long 671f - .\n" ".popsection\n" "671:" "\n\tlock; " "orb %1,%0"
   : "+m" (*(volatile long *) ((void *)(addr) + ((nr)>>3)))
   : "iq" ((u8)(1 << ((nr) & 7)))
   : "memory");
 } else {
  asm volatile(".pushsection .smp_locks,\"a\"\n" ".balign 4\n" ".long 671f - .\n" ".popsection\n" "671:" "\n\tlock; " "bts %1,%0"
   : "+m" (*(volatile long *) (addr)) : "Ir" (nr) : "memory");
 }
}
static inline __attribute__((no_instrument_function)) void __set_bit(long nr, volatile unsigned long *addr)
{
 asm volatile("bts %1,%0" : "+m" (*(volatile long *) (addr)) : "Ir" (nr) : "memory");
}
static inline __attribute__((no_instrument_function)) __attribute__((always_inline)) void
clear_bit(long nr, volatile unsigned long *addr)
{
 if ((__builtin_constant_p(nr))) {
  asm volatile(".pushsection .smp_locks,\"a\"\n" ".balign 4\n" ".long 671f - .\n" ".popsection\n" "671:" "\n\tlock; " "andb %1,%0"
   : "+m" (*(volatile long *) ((void *)(addr) + ((nr)>>3)))
   : "iq" ((u8)~(1 << ((nr) & 7))));
 } else {
  asm volatile(".pushsection .smp_locks,\"a\"\n" ".balign 4\n" ".long 671f - .\n" ".popsection\n" "671:" "\n\tlock; " "btr %1,%0"
   : "+m" (*(volatile long *) (addr))
   : "Ir" (nr));
 }
}
static inline __attribute__((no_instrument_function)) void clear_bit_unlock(long nr, volatile unsigned long *addr)
{
 __asm__ __volatile__("": : :"memory");
 clear_bit(nr, addr);
}

static inline __attribute__((no_instrument_function)) void __clear_bit(long nr, volatile unsigned long *addr)
{
 asm volatile("btr %1,%0" : "+m" (*(volatile long *) (addr)) : "Ir" (nr));
}
static inline __attribute__((no_instrument_function)) void __clear_bit_unlock(long nr, volatile unsigned long *addr)
{
 __asm__ __volatile__("": : :"memory");
 __clear_bit(nr, addr);
}
static inline __attribute__((no_instrument_function)) void __change_bit(long nr, volatile unsigned long *addr)
{
 asm volatile("btc %1,%0" : "+m" (*(volatile long *) (addr)) : "Ir" (nr));
}
static inline __attribute__((no_instrument_function)) void change_bit(long nr, volatile unsigned long *addr)
{
 if ((__builtin_constant_p(nr))) {
  asm volatile(".pushsection .smp_locks,\"a\"\n" ".balign 4\n" ".long 671f - .\n" ".popsection\n" "671:" "\n\tlock; " "xorb %1,%0"
   : "+m" (*(volatile long *) ((void *)(addr) + ((nr)>>3)))
   : "iq" ((u8)(1 << ((nr) & 7))));
 } else {
  asm volatile(".pushsection .smp_locks,\"a\"\n" ".balign 4\n" ".long 671f - .\n" ".popsection\n" "671:" "\n\tlock; " "btc %1,%0"
   : "+m" (*(volatile long *) (addr))
   : "Ir" (nr));
 }
}
static inline __attribute__((no_instrument_function)) int test_and_set_bit(long nr, volatile unsigned long *addr)
{
 do { char c; asm volatile (".pushsection .smp_locks,\"a\"\n" ".balign 4\n" ".long 671f - .\n" ".popsection\n" "671:" "\n\tlock; " "bts" " %2, " "%0" "; set" "c" " %1" : "+m" (*addr), "=qm" (c) : "Ir" (nr) : "memory"); return c != 0; } while (0);
}
static inline __attribute__((no_instrument_function)) __attribute__((always_inline)) int
test_and_set_bit_lock(long nr, volatile unsigned long *addr)
{
 return test_and_set_bit(nr, addr);
}
static inline __attribute__((no_instrument_function)) int __test_and_set_bit(long nr, volatile unsigned long *addr)
{
 int oldbit;

 asm("bts %2,%1\n\t"
     "sbb %0,%0"
     : "=r" (oldbit), "+m" (*(volatile long *) (addr))
     : "Ir" (nr));
 return oldbit;
}
static inline __attribute__((no_instrument_function)) int test_and_clear_bit(long nr, volatile unsigned long *addr)
{
 do { char c; asm volatile (".pushsection .smp_locks,\"a\"\n" ".balign 4\n" ".long 671f - .\n" ".popsection\n" "671:" "\n\tlock; " "btr" " %2, " "%0" "; set" "c" " %1" : "+m" (*addr), "=qm" (c) : "Ir" (nr) : "memory"); return c != 0; } while (0);
}
static inline __attribute__((no_instrument_function)) int __test_and_clear_bit(long nr, volatile unsigned long *addr)
{
 int oldbit;

 asm volatile("btr %2,%1\n\t"
       "sbb %0,%0"
       : "=r" (oldbit), "+m" (*(volatile long *) (addr))
       : "Ir" (nr));
 return oldbit;
}


static inline __attribute__((no_instrument_function)) int __test_and_change_bit(long nr, volatile unsigned long *addr)
{
 int oldbit;

 asm volatile("btc %2,%1\n\t"
       "sbb %0,%0"
       : "=r" (oldbit), "+m" (*(volatile long *) (addr))
       : "Ir" (nr) : "memory");

 return oldbit;
}
static inline __attribute__((no_instrument_function)) int test_and_change_bit(long nr, volatile unsigned long *addr)
{
 do { char c; asm volatile (".pushsection .smp_locks,\"a\"\n" ".balign 4\n" ".long 671f - .\n" ".popsection\n" "671:" "\n\tlock; " "btc" " %2, " "%0" "; set" "c" " %1" : "+m" (*addr), "=qm" (c) : "Ir" (nr) : "memory"); return c != 0; } while (0);
}

static inline __attribute__((no_instrument_function)) __attribute__((always_inline)) int constant_test_bit(long nr, const volatile unsigned long *addr)
{
 return ((1UL << (nr & (64 -1))) &
  (addr[nr >> 6])) != 0;
}

static inline __attribute__((no_instrument_function)) int variable_test_bit(long nr, volatile const unsigned long *addr)
{
 int oldbit;

 asm volatile("bt %2,%1\n\t"
       "sbb %0,%0"
       : "=r" (oldbit)
       : "m" (*(unsigned long *)addr), "Ir" (nr));

 return oldbit;
}
static inline __attribute__((no_instrument_function)) unsigned long __ffs(unsigned long word)
{
 asm("rep; bsf %1,%0"
  : "=r" (word)
  : "rm" (word));
 return word;
}







static inline __attribute__((no_instrument_function)) unsigned long ffz(unsigned long word)
{
 asm("rep; bsf %1,%0"
  : "=r" (word)
  : "r" (~word));
 return word;
}







static inline __attribute__((no_instrument_function)) unsigned long __fls(unsigned long word)
{
 asm("bsr %1,%0"
     : "=r" (word)
     : "rm" (word));
 return word;
}
static inline __attribute__((no_instrument_function)) int ffs(int x)
{
 int r;
 asm("bsfl %1,%0"
     : "=r" (r)
     : "rm" (x), "0" (-1));
 return r + 1;
}
static inline __attribute__((no_instrument_function)) int fls(int x)
{
 int r;
 asm("bsrl %1,%0"
     : "=r" (r)
     : "rm" (x), "0" (-1));
 return r + 1;
}
static inline __attribute__((no_instrument_function)) __attribute__((always_inline)) int fls64(__u64 x)
{
 int bitpos = -1;





 asm("bsrq %1,%q0"
     : "+r" (bitpos)
     : "rm" (x));
 return bitpos + 1;
}




extern unsigned long find_next_bit(const unsigned long *addr, unsigned long
  size, unsigned long offset);
extern unsigned long find_next_zero_bit(const unsigned long *addr, unsigned
  long size, unsigned long offset);
extern unsigned long find_first_bit(const unsigned long *addr,
        unsigned long size);
extern unsigned long find_first_zero_bit(const unsigned long *addr,
      unsigned long size);

static inline __attribute__((no_instrument_function)) int sched_find_first_bit(const unsigned long *b)
{

 if (b[0])
  return __ffs(b[0]);
 return __ffs(b[1]) + 64;
}

static inline __attribute__((no_instrument_function)) unsigned int __arch_hweight32(unsigned int w)
{
 unsigned int res = 0;

 asm ("661:\n\t" "call __sw_hweight32" "\n662:\n" ".pushsection .altinstructions,\"a\"\n" " .long 661b - .\n" " .long " "663""1""f - .\n" " .word " "( 4*32+23)" "\n" " .byte " "662b-661b" "\n" " .byte " "664""1""f-""663""1""f" "\n" ".popsection\n" ".pushsection .discard,\"aw\",@progbits\n" " .byte 0xff + (" "664""1""f-""663""1""f" ") - (" "662b-661b" ")\n" ".popsection\n" ".pushsection .altinstr_replacement, \"ax\"\n" "663""1"":\n\t" ".byte 0xf3,0x40,0x0f,0xb8,0xc7" "\n" "664""1" ":\n\t" ".popsection"
       : "=""a" (res)
       : "D" (w));

 return res;
}

static inline __attribute__((no_instrument_function)) unsigned int __arch_hweight16(unsigned int w)
{
 return __arch_hweight32(w & 0xffff);
}

static inline __attribute__((no_instrument_function)) unsigned int __arch_hweight8(unsigned int w)
{
 return __arch_hweight32(w & 0xff);
}

static inline __attribute__((no_instrument_function)) unsigned long __arch_hweight64(__u64 w)
{
 unsigned long res = 0;





 asm ("661:\n\t" "call __sw_hweight64" "\n662:\n" ".pushsection .altinstructions,\"a\"\n" " .long 661b - .\n" " .long " "663""1""f - .\n" " .word " "( 4*32+23)" "\n" " .byte " "662b-661b" "\n" " .byte " "664""1""f-""663""1""f" "\n" ".popsection\n" ".pushsection .discard,\"aw\",@progbits\n" " .byte 0xff + (" "664""1""f-""663""1""f" ") - (" "662b-661b" ")\n" ".popsection\n" ".pushsection .altinstr_replacement, \"ax\"\n" "663""1"":\n\t" ".byte 0xf3,0x48,0x0f,0xb8,0xc7" "\n" "664""1" ":\n\t" ".popsection"
       : "=""a" (res)
       : "D" (w));


 return res;
}


























static inline __attribute__((no_instrument_function)) __attribute__((__const__)) __u32 __arch_swab32(__u32 val)
{
 asm("bswapl %0" : "=r" (val) : "0" (val));
 return val;
}


static inline __attribute__((no_instrument_function)) __attribute__((__const__)) __u64 __arch_swab64(__u64 val)
{
 asm("bswapq %0" : "=r" (val) : "0" (val));
 return val;

}
static inline __attribute__((no_instrument_function)) __attribute__((__const__)) __u16 __fswab16(__u16 val)
{





 return ((__u16)( (((__u16)(val) & (__u16)0x00ffU) << 8) | (((__u16)(val) & (__u16)0xff00U) >> 8)));

}

static inline __attribute__((no_instrument_function)) __attribute__((__const__)) __u32 __fswab32(__u32 val)
{

 return __builtin_bswap32(val);





}

static inline __attribute__((no_instrument_function)) __attribute__((__const__)) __u64 __fswab64(__u64 val)
{

 return __builtin_bswap64(val);
}

static inline __attribute__((no_instrument_function)) __attribute__((__const__)) __u32 __fswahw32(__u32 val)
{



 return ((__u32)( (((__u32)(val) & (__u32)0x0000ffffUL) << 16) | (((__u32)(val) & (__u32)0xffff0000UL) >> 16)));

}

static inline __attribute__((no_instrument_function)) __attribute__((__const__)) __u32 __fswahb32(__u32 val)
{



 return ((__u32)( (((__u32)(val) & (__u32)0x00ff00ffUL) << 8) | (((__u32)(val) & (__u32)0xff00ff00UL) >> 8)));

}
static inline __attribute__((no_instrument_function)) __u16 __swab16p(const __u16 *p)
{



 return (__builtin_constant_p((__u16)(*p)) ? ((__u16)( (((__u16)(*p) & (__u16)0x00ffU) << 8) | (((__u16)(*p) & (__u16)0xff00U) >> 8))) : __fswab16(*p));

}





static inline __attribute__((no_instrument_function)) __u32 __swab32p(const __u32 *p)
{



 return (__builtin_constant_p((__u32)(*p)) ? ((__u32)( (((__u32)(*p) & (__u32)0x000000ffUL) << 24) | (((__u32)(*p) & (__u32)0x0000ff00UL) << 8) | (((__u32)(*p) & (__u32)0x00ff0000UL) >> 8) | (((__u32)(*p) & (__u32)0xff000000UL) >> 24))) : __fswab32(*p));

}





static inline __attribute__((no_instrument_function)) __u64 __swab64p(const __u64 *p)
{



 return (__builtin_constant_p((__u64)(*p)) ? ((__u64)( (((__u64)(*p) & (__u64)0x00000000000000ffULL) << 56) | (((__u64)(*p) & (__u64)0x000000000000ff00ULL) << 40) | (((__u64)(*p) & (__u64)0x0000000000ff0000ULL) << 24) | (((__u64)(*p) & (__u64)0x00000000ff000000ULL) << 8) | (((__u64)(*p) & (__u64)0x000000ff00000000ULL) >> 8) | (((__u64)(*p) & (__u64)0x0000ff0000000000ULL) >> 24) | (((__u64)(*p) & (__u64)0x00ff000000000000ULL) >> 40) | (((__u64)(*p) & (__u64)0xff00000000000000ULL) >> 56))) : __fswab64(*p));

}







static inline __attribute__((no_instrument_function)) __u32 __swahw32p(const __u32 *p)
{



 return (__builtin_constant_p((__u32)(*p)) ? ((__u32)( (((__u32)(*p) & (__u32)0x0000ffffUL) << 16) | (((__u32)(*p) & (__u32)0xffff0000UL) >> 16))) : __fswahw32(*p));

}







static inline __attribute__((no_instrument_function)) __u32 __swahb32p(const __u32 *p)
{



 return (__builtin_constant_p((__u32)(*p)) ? ((__u32)( (((__u32)(*p) & (__u32)0x00ff00ffUL) << 8) | (((__u32)(*p) & (__u32)0xff00ff00UL) >> 8))) : __fswahb32(*p));

}





static inline __attribute__((no_instrument_function)) void __swab16s(__u16 *p)
{



 *p = __swab16p(p);

}




static inline __attribute__((no_instrument_function)) void __swab32s(__u32 *p)
{



 *p = __swab32p(p);

}





static inline __attribute__((no_instrument_function)) void __swab64s(__u64 *p)
{



 *p = __swab64p(p);

}







static inline __attribute__((no_instrument_function)) void __swahw32s(__u32 *p)
{



 *p = __swahw32p(p);

}







static inline __attribute__((no_instrument_function)) void __swahb32s(__u32 *p)
{



 *p = __swahb32p(p);

}
static inline __attribute__((no_instrument_function)) __le64 __cpu_to_le64p(const __u64 *p)
{
 return ( __le64)*p;
}
static inline __attribute__((no_instrument_function)) __u64 __le64_to_cpup(const __le64 *p)
{
 return ( __u64)*p;
}
static inline __attribute__((no_instrument_function)) __le32 __cpu_to_le32p(const __u32 *p)
{
 return ( __le32)*p;
}
static inline __attribute__((no_instrument_function)) __u32 __le32_to_cpup(const __le32 *p)
{
 return ( __u32)*p;
}
static inline __attribute__((no_instrument_function)) __le16 __cpu_to_le16p(const __u16 *p)
{
 return ( __le16)*p;
}
static inline __attribute__((no_instrument_function)) __u16 __le16_to_cpup(const __le16 *p)
{
 return ( __u16)*p;
}
static inline __attribute__((no_instrument_function)) __be64 __cpu_to_be64p(const __u64 *p)
{
 return ( __be64)__swab64p(p);
}
static inline __attribute__((no_instrument_function)) __u64 __be64_to_cpup(const __be64 *p)
{
 return __swab64p((__u64 *)p);
}
static inline __attribute__((no_instrument_function)) __be32 __cpu_to_be32p(const __u32 *p)
{
 return ( __be32)__swab32p(p);
}
static inline __attribute__((no_instrument_function)) __u32 __be32_to_cpup(const __be32 *p)
{
 return __swab32p((__u32 *)p);
}
static inline __attribute__((no_instrument_function)) __be16 __cpu_to_be16p(const __u16 *p)
{
 return ( __be16)__swab16p(p);
}
static inline __attribute__((no_instrument_function)) __u16 __be16_to_cpup(const __be16 *p)
{
 return __swab16p((__u16 *)p);
}

static inline __attribute__((no_instrument_function)) void le16_add_cpu(__le16 *var, u16 val)
{
 *var = (( __le16)(__u16)((( __u16)(__le16)(*var)) + val));
}

static inline __attribute__((no_instrument_function)) void le32_add_cpu(__le32 *var, u32 val)
{
 *var = (( __le32)(__u32)((( __u32)(__le32)(*var)) + val));
}

static inline __attribute__((no_instrument_function)) void le64_add_cpu(__le64 *var, u64 val)
{
 *var = (( __le64)(__u64)((( __u64)(__le64)(*var)) + val));
}

static inline __attribute__((no_instrument_function)) void be16_add_cpu(__be16 *var, u16 val)
{
 *var = (( __be16)(__builtin_constant_p((__u16)(((__builtin_constant_p((__u16)(( __u16)(__be16)(*var))) ? ((__u16)( (((__u16)(( __u16)(__be16)(*var)) & (__u16)0x00ffU) << 8) | (((__u16)(( __u16)(__be16)(*var)) & (__u16)0xff00U) >> 8))) : __fswab16(( __u16)(__be16)(*var))) + val))) ? ((__u16)( (((__u16)(((__builtin_constant_p((__u16)(( __u16)(__be16)(*var))) ? ((__u16)( (((__u16)(( __u16)(__be16)(*var)) & (__u16)0x00ffU) << 8) | (((__u16)(( __u16)(__be16)(*var)) & (__u16)0xff00U) >> 8))) : __fswab16(( __u16)(__be16)(*var))) + val)) & (__u16)0x00ffU) << 8) | (((__u16)(((__builtin_constant_p((__u16)(( __u16)(__be16)(*var))) ? ((__u16)( (((__u16)(( __u16)(__be16)(*var)) & (__u16)0x00ffU) << 8) | (((__u16)(( __u16)(__be16)(*var)) & (__u16)0xff00U) >> 8))) : __fswab16(( __u16)(__be16)(*var))) + val)) & (__u16)0xff00U) >> 8))) : __fswab16(((__builtin_constant_p((__u16)(( __u16)(__be16)(*var))) ? ((__u16)( (((__u16)(( __u16)(__be16)(*var)) & (__u16)0x00ffU) << 8) | (((__u16)(( __u16)(__be16)(*var)) & (__u16)0xff00U) >> 8))) : __fswab16(( __u16)(__be16)(*var))) + val))));
}

static inline __attribute__((no_instrument_function)) void be32_add_cpu(__be32 *var, u32 val)
{
 *var = (( __be32)(__builtin_constant_p((__u32)(((__builtin_constant_p((__u32)(( __u32)(__be32)(*var))) ? ((__u32)( (((__u32)(( __u32)(__be32)(*var)) & (__u32)0x000000ffUL) << 24) | (((__u32)(( __u32)(__be32)(*var)) & (__u32)0x0000ff00UL) << 8) | (((__u32)(( __u32)(__be32)(*var)) & (__u32)0x00ff0000UL) >> 8) | (((__u32)(( __u32)(__be32)(*var)) & (__u32)0xff000000UL) >> 24))) : __fswab32(( __u32)(__be32)(*var))) + val))) ? ((__u32)( (((__u32)(((__builtin_constant_p((__u32)(( __u32)(__be32)(*var))) ? ((__u32)( (((__u32)(( __u32)(__be32)(*var)) & (__u32)0x000000ffUL) << 24) | (((__u32)(( __u32)(__be32)(*var)) & (__u32)0x0000ff00UL) << 8) | (((__u32)(( __u32)(__be32)(*var)) & (__u32)0x00ff0000UL) >> 8) | (((__u32)(( __u32)(__be32)(*var)) & (__u32)0xff000000UL) >> 24))) : __fswab32(( __u32)(__be32)(*var))) + val)) & (__u32)0x000000ffUL) << 24) | (((__u32)(((__builtin_constant_p((__u32)(( __u32)(__be32)(*var))) ? ((__u32)( (((__u32)(( __u32)(__be32)(*var)) & (__u32)0x000000ffUL) << 24) | (((__u32)(( __u32)(__be32)(*var)) & (__u32)0x0000ff00UL) << 8) | (((__u32)(( __u32)(__be32)(*var)) & (__u32)0x00ff0000UL) >> 8) | (((__u32)(( __u32)(__be32)(*var)) & (__u32)0xff000000UL) >> 24))) : __fswab32(( __u32)(__be32)(*var))) + val)) & (__u32)0x0000ff00UL) << 8) | (((__u32)(((__builtin_constant_p((__u32)(( __u32)(__be32)(*var))) ? ((__u32)( (((__u32)(( __u32)(__be32)(*var)) & (__u32)0x000000ffUL) << 24) | (((__u32)(( __u32)(__be32)(*var)) & (__u32)0x0000ff00UL) << 8) | (((__u32)(( __u32)(__be32)(*var)) & (__u32)0x00ff0000UL) >> 8) | (((__u32)(( __u32)(__be32)(*var)) & (__u32)0xff000000UL) >> 24))) : __fswab32(( __u32)(__be32)(*var))) + val)) & (__u32)0x00ff0000UL) >> 8) | (((__u32)(((__builtin_constant_p((__u32)(( __u32)(__be32)(*var))) ? ((__u32)( (((__u32)(( __u32)(__be32)(*var)) & (__u32)0x000000ffUL) << 24) | (((__u32)(( __u32)(__be32)(*var)) & (__u32)0x0000ff00UL) << 8) | (((__u32)(( __u32)(__be32)(*var)) & (__u32)0x00ff0000UL) >> 8) | (((__u32)(( __u32)(__be32)(*var)) & (__u32)0xff000000UL) >> 24))) : __fswab32(( __u32)(__be32)(*var))) + val)) & (__u32)0xff000000UL) >> 24))) : __fswab32(((__builtin_constant_p((__u32)(( __u32)(__be32)(*var))) ? ((__u32)( (((__u32)(( __u32)(__be32)(*var)) & (__u32)0x000000ffUL) << 24) | (((__u32)(( __u32)(__be32)(*var)) & (__u32)0x0000ff00UL) << 8) | (((__u32)(( __u32)(__be32)(*var)) & (__u32)0x00ff0000UL) >> 8) | (((__u32)(( __u32)(__be32)(*var)) & (__u32)0xff000000UL) >> 24))) : __fswab32(( __u32)(__be32)(*var))) + val))));
}

static inline __attribute__((no_instrument_function)) void be64_add_cpu(__be64 *var, u64 val)
{
 *var = (( __be64)(__builtin_constant_p((__u64)(((__builtin_constant_p((__u64)(( __u64)(__be64)(*var))) ? ((__u64)( (((__u64)(( __u64)(__be64)(*var)) & (__u64)0x00000000000000ffULL) << 56) | (((__u64)(( __u64)(__be64)(*var)) & (__u64)0x000000000000ff00ULL) << 40) | (((__u64)(( __u64)(__be64)(*var)) & (__u64)0x0000000000ff0000ULL) << 24) | (((__u64)(( __u64)(__be64)(*var)) & (__u64)0x00000000ff000000ULL) << 8) | (((__u64)(( __u64)(__be64)(*var)) & (__u64)0x000000ff00000000ULL) >> 8) | (((__u64)(( __u64)(__be64)(*var)) & (__u64)0x0000ff0000000000ULL) >> 24) | (((__u64)(( __u64)(__be64)(*var)) & (__u64)0x00ff000000000000ULL) >> 40) | (((__u64)(( __u64)(__be64)(*var)) & (__u64)0xff00000000000000ULL) >> 56))) : __fswab64(( __u64)(__be64)(*var))) + val))) ? ((__u64)( (((__u64)(((__builtin_constant_p((__u64)(( __u64)(__be64)(*var))) ? ((__u64)( (((__u64)(( __u64)(__be64)(*var)) & (__u64)0x00000000000000ffULL) << 56) | (((__u64)(( __u64)(__be64)(*var)) & (__u64)0x000000000000ff00ULL) << 40) | (((__u64)(( __u64)(__be64)(*var)) & (__u64)0x0000000000ff0000ULL) << 24) | (((__u64)(( __u64)(__be64)(*var)) & (__u64)0x00000000ff000000ULL) << 8) | (((__u64)(( __u64)(__be64)(*var)) & (__u64)0x000000ff00000000ULL) >> 8) | (((__u64)(( __u64)(__be64)(*var)) & (__u64)0x0000ff0000000000ULL) >> 24) | (((__u64)(( __u64)(__be64)(*var)) & (__u64)0x00ff000000000000ULL) >> 40) | (((__u64)(( __u64)(__be64)(*var)) & (__u64)0xff00000000000000ULL) >> 56))) : __fswab64(( __u64)(__be64)(*var))) + val)) & (__u64)0x00000000000000ffULL) << 56) | (((__u64)(((__builtin_constant_p((__u64)(( __u64)(__be64)(*var))) ? ((__u64)( (((__u64)(( __u64)(__be64)(*var)) & (__u64)0x00000000000000ffULL) << 56) | (((__u64)(( __u64)(__be64)(*var)) & (__u64)0x000000000000ff00ULL) << 40) | (((__u64)(( __u64)(__be64)(*var)) & (__u64)0x0000000000ff0000ULL) << 24) | (((__u64)(( __u64)(__be64)(*var)) & (__u64)0x00000000ff000000ULL) << 8) | (((__u64)(( __u64)(__be64)(*var)) & (__u64)0x000000ff00000000ULL) >> 8) | (((__u64)(( __u64)(__be64)(*var)) & (__u64)0x0000ff0000000000ULL) >> 24) | (((__u64)(( __u64)(__be64)(*var)) & (__u64)0x00ff000000000000ULL) >> 40) | (((__u64)(( __u64)(__be64)(*var)) & (__u64)0xff00000000000000ULL) >> 56))) : __fswab64(( __u64)(__be64)(*var))) + val)) & (__u64)0x000000000000ff00ULL) << 40) | (((__u64)(((__builtin_constant_p((__u64)(( __u64)(__be64)(*var))) ? ((__u64)( (((__u64)(( __u64)(__be64)(*var)) & (__u64)0x00000000000000ffULL) << 56) | (((__u64)(( __u64)(__be64)(*var)) & (__u64)0x000000000000ff00ULL) << 40) | (((__u64)(( __u64)(__be64)(*var)) & (__u64)0x0000000000ff0000ULL) << 24) | (((__u64)(( __u64)(__be64)(*var)) & (__u64)0x00000000ff000000ULL) << 8) | (((__u64)(( __u64)(__be64)(*var)) & (__u64)0x000000ff00000000ULL) >> 8) | (((__u64)(( __u64)(__be64)(*var)) & (__u64)0x0000ff0000000000ULL) >> 24) | (((__u64)(( __u64)(__be64)(*var)) & (__u64)0x00ff000000000000ULL) >> 40) | (((__u64)(( __u64)(__be64)(*var)) & (__u64)0xff00000000000000ULL) >> 56))) : __fswab64(( __u64)(__be64)(*var))) + val)) & (__u64)0x0000000000ff0000ULL) << 24) | (((__u64)(((__builtin_constant_p((__u64)(( __u64)(__be64)(*var))) ? ((__u64)( (((__u64)(( __u64)(__be64)(*var)) & (__u64)0x00000000000000ffULL) << 56) | (((__u64)(( __u64)(__be64)(*var)) & (__u64)0x000000000000ff00ULL) << 40) | (((__u64)(( __u64)(__be64)(*var)) & (__u64)0x0000000000ff0000ULL) << 24) | (((__u64)(( __u64)(__be64)(*var)) & (__u64)0x00000000ff000000ULL) << 8) | (((__u64)(( __u64)(__be64)(*var)) & (__u64)0x000000ff00000000ULL) >> 8) | (((__u64)(( __u64)(__be64)(*var)) & (__u64)0x0000ff0000000000ULL) >> 24) | (((__u64)(( __u64)(__be64)(*var)) & (__u64)0x00ff000000000000ULL) >> 40) | (((__u64)(( __u64)(__be64)(*var)) & (__u64)0xff00000000000000ULL) >> 56))) : __fswab64(( __u64)(__be64)(*var))) + val)) & (__u64)0x00000000ff000000ULL) << 8) | (((__u64)(((__builtin_constant_p((__u64)(( __u64)(__be64)(*var))) ? ((__u64)( (((__u64)(( __u64)(__be64)(*var)) & (__u64)0x00000000000000ffULL) << 56) | (((__u64)(( __u64)(__be64)(*var)) & (__u64)0x000000000000ff00ULL) << 40) | (((__u64)(( __u64)(__be64)(*var)) & (__u64)0x0000000000ff0000ULL) << 24) | (((__u64)(( __u64)(__be64)(*var)) & (__u64)0x00000000ff000000ULL) << 8) | (((__u64)(( __u64)(__be64)(*var)) & (__u64)0x000000ff00000000ULL) >> 8) | (((__u64)(( __u64)(__be64)(*var)) & (__u64)0x0000ff0000000000ULL) >> 24) | (((__u64)(( __u64)(__be64)(*var)) & (__u64)0x00ff000000000000ULL) >> 40) | (((__u64)(( __u64)(__be64)(*var)) & (__u64)0xff00000000000000ULL) >> 56))) : __fswab64(( __u64)(__be64)(*var))) + val)) & (__u64)0x000000ff00000000ULL) >> 8) | (((__u64)(((__builtin_constant_p((__u64)(( __u64)(__be64)(*var))) ? ((__u64)( (((__u64)(( __u64)(__be64)(*var)) & (__u64)0x00000000000000ffULL) << 56) | (((__u64)(( __u64)(__be64)(*var)) & (__u64)0x000000000000ff00ULL) << 40) | (((__u64)(( __u64)(__be64)(*var)) & (__u64)0x0000000000ff0000ULL) << 24) | (((__u64)(( __u64)(__be64)(*var)) & (__u64)0x00000000ff000000ULL) << 8) | (((__u64)(( __u64)(__be64)(*var)) & (__u64)0x000000ff00000000ULL) >> 8) | (((__u64)(( __u64)(__be64)(*var)) & (__u64)0x0000ff0000000000ULL) >> 24) | (((__u64)(( __u64)(__be64)(*var)) & (__u64)0x00ff000000000000ULL) >> 40) | (((__u64)(( __u64)(__be64)(*var)) & (__u64)0xff00000000000000ULL) >> 56))) : __fswab64(( __u64)(__be64)(*var))) + val)) & (__u64)0x0000ff0000000000ULL) >> 24) | (((__u64)(((__builtin_constant_p((__u64)(( __u64)(__be64)(*var))) ? ((__u64)( (((__u64)(( __u64)(__be64)(*var)) & (__u64)0x00000000000000ffULL) << 56) | (((__u64)(( __u64)(__be64)(*var)) & (__u64)0x000000000000ff00ULL) << 40) | (((__u64)(( __u64)(__be64)(*var)) & (__u64)0x0000000000ff0000ULL) << 24) | (((__u64)(( __u64)(__be64)(*var)) & (__u64)0x00000000ff000000ULL) << 8) | (((__u64)(( __u64)(__be64)(*var)) & (__u64)0x000000ff00000000ULL) >> 8) | (((__u64)(( __u64)(__be64)(*var)) & (__u64)0x0000ff0000000000ULL) >> 24) | (((__u64)(( __u64)(__be64)(*var)) & (__u64)0x00ff000000000000ULL) >> 40) | (((__u64)(( __u64)(__be64)(*var)) & (__u64)0xff00000000000000ULL) >> 56))) : __fswab64(( __u64)(__be64)(*var))) + val)) & (__u64)0x00ff000000000000ULL) >> 40) | (((__u64)(((__builtin_constant_p((__u64)(( __u64)(__be64)(*var))) ? ((__u64)( (((__u64)(( __u64)(__be64)(*var)) & (__u64)0x00000000000000ffULL) << 56) | (((__u64)(( __u64)(__be64)(*var)) & (__u64)0x000000000000ff00ULL) << 40) | (((__u64)(( __u64)(__be64)(*var)) & (__u64)0x0000000000ff0000ULL) << 24) | (((__u64)(( __u64)(__be64)(*var)) & (__u64)0x00000000ff000000ULL) << 8) | (((__u64)(( __u64)(__be64)(*var)) & (__u64)0x000000ff00000000ULL) >> 8) | (((__u64)(( __u64)(__be64)(*var)) & (__u64)0x0000ff0000000000ULL) >> 24) | (((__u64)(( __u64)(__be64)(*var)) & (__u64)0x00ff000000000000ULL) >> 40) | (((__u64)(( __u64)(__be64)(*var)) & (__u64)0xff00000000000000ULL) >> 56))) : __fswab64(( __u64)(__be64)(*var))) + val)) & (__u64)0xff00000000000000ULL) >> 56))) : __fswab64(((__builtin_constant_p((__u64)(( __u64)(__be64)(*var))) ? ((__u64)( (((__u64)(( __u64)(__be64)(*var)) & (__u64)0x00000000000000ffULL) << 56) | (((__u64)(( __u64)(__be64)(*var)) & (__u64)0x000000000000ff00ULL) << 40) | (((__u64)(( __u64)(__be64)(*var)) & (__u64)0x0000000000ff0000ULL) << 24) | (((__u64)(( __u64)(__be64)(*var)) & (__u64)0x00000000ff000000ULL) << 8) | (((__u64)(( __u64)(__be64)(*var)) & (__u64)0x000000ff00000000ULL) >> 8) | (((__u64)(( __u64)(__be64)(*var)) & (__u64)0x0000ff0000000000ULL) >> 24) | (((__u64)(( __u64)(__be64)(*var)) & (__u64)0x00ff000000000000ULL) >> 40) | (((__u64)(( __u64)(__be64)(*var)) & (__u64)0xff00000000000000ULL) >> 56))) : __fswab64(( __u64)(__be64)(*var))) + val))));
}





static inline __attribute__((no_instrument_function)) unsigned long find_next_zero_bit_le(const void *addr,
  unsigned long size, unsigned long offset)
{
 return find_next_zero_bit(addr, size, offset);
}

static inline __attribute__((no_instrument_function)) unsigned long find_next_bit_le(const void *addr,
  unsigned long size, unsigned long offset)
{
 return find_next_bit(addr, size, offset);
}

static inline __attribute__((no_instrument_function)) unsigned long find_first_zero_bit_le(const void *addr,
  unsigned long size)
{
 return find_first_zero_bit(addr, size);
}
static inline __attribute__((no_instrument_function)) int test_bit_le(int nr, const void *addr)
{
 return (__builtin_constant_p((nr ^ 0)) ? constant_test_bit((nr ^ 0), (addr)) : variable_test_bit((nr ^ 0), (addr)));
}

static inline __attribute__((no_instrument_function)) void set_bit_le(int nr, void *addr)
{
 set_bit(nr ^ 0, addr);
}

static inline __attribute__((no_instrument_function)) void clear_bit_le(int nr, void *addr)
{
 clear_bit(nr ^ 0, addr);
}

static inline __attribute__((no_instrument_function)) void __set_bit_le(int nr, void *addr)
{
 __set_bit(nr ^ 0, addr);
}

static inline __attribute__((no_instrument_function)) void __clear_bit_le(int nr, void *addr)
{
 __clear_bit(nr ^ 0, addr);
}

static inline __attribute__((no_instrument_function)) int test_and_set_bit_le(int nr, void *addr)
{
 return test_and_set_bit(nr ^ 0, addr);
}

static inline __attribute__((no_instrument_function)) int test_and_clear_bit_le(int nr, void *addr)
{
 return test_and_clear_bit(nr ^ 0, addr);
}

static inline __attribute__((no_instrument_function)) int __test_and_set_bit_le(int nr, void *addr)
{
 return __test_and_set_bit(nr ^ 0, addr);
}

static inline __attribute__((no_instrument_function)) int __test_and_clear_bit_le(int nr, void *addr)
{
 return __test_and_clear_bit(nr ^ 0, addr);
}







static inline __attribute__((no_instrument_function)) void smp_mb__before_clear_bit(void)
{
 extern void __smp_mb__before_atomic(void);
 __smp_mb__before_atomic();
}



static inline __attribute__((no_instrument_function)) void smp_mb__after_clear_bit(void)
{
 extern void __smp_mb__after_atomic(void);
 __smp_mb__after_atomic();
}
static __inline__ __attribute__((no_instrument_function)) int get_bitmask_order(unsigned int count)
{
 int order;

 order = fls(count);
 return order;
}

static __inline__ __attribute__((no_instrument_function)) int get_count_order(unsigned int count)
{
 int order;

 order = fls(count) - 1;
 if (count & (count - 1))
  order++;
 return order;
}

static inline __attribute__((no_instrument_function)) unsigned long hweight_long(unsigned long w)
{
 return sizeof(w) == 4 ? (__builtin_constant_p(w) ? ((((unsigned int) ((!!((w) & (1ULL << 0))) + (!!((w) & (1ULL << 1))) + (!!((w) & (1ULL << 2))) + (!!((w) & (1ULL << 3))) + (!!((w) & (1ULL << 4))) + (!!((w) & (1ULL << 5))) + (!!((w) & (1ULL << 6))) + (!!((w) & (1ULL << 7))))) + ((unsigned int) ((!!(((w) >> 8) & (1ULL << 0))) + (!!(((w) >> 8) & (1ULL << 1))) + (!!(((w) >> 8) & (1ULL << 2))) + (!!(((w) >> 8) & (1ULL << 3))) + (!!(((w) >> 8) & (1ULL << 4))) + (!!(((w) >> 8) & (1ULL << 5))) + (!!(((w) >> 8) & (1ULL << 6))) + (!!(((w) >> 8) & (1ULL << 7)))))) + (((unsigned int) ((!!(((w) >> 16) & (1ULL << 0))) + (!!(((w) >> 16) & (1ULL << 1))) + (!!(((w) >> 16) & (1ULL << 2))) + (!!(((w) >> 16) & (1ULL << 3))) + (!!(((w) >> 16) & (1ULL << 4))) + (!!(((w) >> 16) & (1ULL << 5))) + (!!(((w) >> 16) & (1ULL << 6))) + (!!(((w) >> 16) & (1ULL << 7))))) + ((unsigned int) ((!!((((w) >> 16) >> 8) & (1ULL << 0))) + (!!((((w) >> 16) >> 8) & (1ULL << 1))) + (!!((((w) >> 16) >> 8) & (1ULL << 2))) + (!!((((w) >> 16) >> 8) & (1ULL << 3))) + (!!((((w) >> 16) >> 8) & (1ULL << 4))) + (!!((((w) >> 16) >> 8) & (1ULL << 5))) + (!!((((w) >> 16) >> 8) & (1ULL << 6))) + (!!((((w) >> 16) >> 8) & (1ULL << 7))))))) : __arch_hweight32(w)) : (__builtin_constant_p(w) ? (((((unsigned int) ((!!((w) & (1ULL << 0))) + (!!((w) & (1ULL << 1))) + (!!((w) & (1ULL << 2))) + (!!((w) & (1ULL << 3))) + (!!((w) & (1ULL << 4))) + (!!((w) & (1ULL << 5))) + (!!((w) & (1ULL << 6))) + (!!((w) & (1ULL << 7))))) + ((unsigned int) ((!!(((w) >> 8) & (1ULL << 0))) + (!!(((w) >> 8) & (1ULL << 1))) + (!!(((w) >> 8) & (1ULL << 2))) + (!!(((w) >> 8) & (1ULL << 3))) + (!!(((w) >> 8) & (1ULL << 4))) + (!!(((w) >> 8) & (1ULL << 5))) + (!!(((w) >> 8) & (1ULL << 6))) + (!!(((w) >> 8) & (1ULL << 7)))))) + (((unsigned int) ((!!(((w) >> 16) & (1ULL << 0))) + (!!(((w) >> 16) & (1ULL << 1))) + (!!(((w) >> 16) & (1ULL << 2))) + (!!(((w) >> 16) & (1ULL << 3))) + (!!(((w) >> 16) & (1ULL << 4))) + (!!(((w) >> 16) & (1ULL << 5))) + (!!(((w) >> 16) & (1ULL << 6))) + (!!(((w) >> 16) & (1ULL << 7))))) + ((unsigned int) ((!!((((w) >> 16) >> 8) & (1ULL << 0))) + (!!((((w) >> 16) >> 8) & (1ULL << 1))) + (!!((((w) >> 16) >> 8) & (1ULL << 2))) + (!!((((w) >> 16) >> 8) & (1ULL << 3))) + (!!((((w) >> 16) >> 8) & (1ULL << 4))) + (!!((((w) >> 16) >> 8) & (1ULL << 5))) + (!!((((w) >> 16) >> 8) & (1ULL << 6))) + (!!((((w) >> 16) >> 8) & (1ULL << 7))))))) + ((((unsigned int) ((!!(((w) >> 32) & (1ULL << 0))) + (!!(((w) >> 32) & (1ULL << 1))) + (!!(((w) >> 32) & (1ULL << 2))) + (!!(((w) >> 32) & (1ULL << 3))) + (!!(((w) >> 32) & (1ULL << 4))) + (!!(((w) >> 32) & (1ULL << 5))) + (!!(((w) >> 32) & (1ULL << 6))) + (!!(((w) >> 32) & (1ULL << 7))))) + ((unsigned int) ((!!((((w) >> 32) >> 8) & (1ULL << 0))) + (!!((((w) >> 32) >> 8) & (1ULL << 1))) + (!!((((w) >> 32) >> 8) & (1ULL << 2))) + (!!((((w) >> 32) >> 8) & (1ULL << 3))) + (!!((((w) >> 32) >> 8) & (1ULL << 4))) + (!!((((w) >> 32) >> 8) & (1ULL << 5))) + (!!((((w) >> 32) >> 8) & (1ULL << 6))) + (!!((((w) >> 32) >> 8) & (1ULL << 7)))))) + (((unsigned int) ((!!((((w) >> 32) >> 16) & (1ULL << 0))) + (!!((((w) >> 32) >> 16) & (1ULL << 1))) + (!!((((w) >> 32) >> 16) & (1ULL << 2))) + (!!((((w) >> 32) >> 16) & (1ULL << 3))) + (!!((((w) >> 32) >> 16) & (1ULL << 4))) + (!!((((w) >> 32) >> 16) & (1ULL << 5))) + (!!((((w) >> 32) >> 16) & (1ULL << 6))) + (!!((((w) >> 32) >> 16) & (1ULL << 7))))) + ((unsigned int) ((!!(((((w) >> 32) >> 16) >> 8) & (1ULL << 0))) + (!!(((((w) >> 32) >> 16) >> 8) & (1ULL << 1))) + (!!(((((w) >> 32) >> 16) >> 8) & (1ULL << 2))) + (!!(((((w) >> 32) >> 16) >> 8) & (1ULL << 3))) + (!!(((((w) >> 32) >> 16) >> 8) & (1ULL << 4))) + (!!(((((w) >> 32) >> 16) >> 8) & (1ULL << 5))) + (!!(((((w) >> 32) >> 16) >> 8) & (1ULL << 6))) + (!!(((((w) >> 32) >> 16) >> 8) & (1ULL << 7)))))))) : __arch_hweight64(w));
}






static inline __attribute__((no_instrument_function)) __u64 rol64(__u64 word, unsigned int shift)
{
 return (word << shift) | (word >> (64 - shift));
}






static inline __attribute__((no_instrument_function)) __u64 ror64(__u64 word, unsigned int shift)
{
 return (word >> shift) | (word << (64 - shift));
}






static inline __attribute__((no_instrument_function)) __u32 rol32(__u32 word, unsigned int shift)
{
 return (word << shift) | (word >> (32 - shift));
}






static inline __attribute__((no_instrument_function)) __u32 ror32(__u32 word, unsigned int shift)
{
 return (word >> shift) | (word << (32 - shift));
}






static inline __attribute__((no_instrument_function)) __u16 rol16(__u16 word, unsigned int shift)
{
 return (word << shift) | (word >> (16 - shift));
}






static inline __attribute__((no_instrument_function)) __u16 ror16(__u16 word, unsigned int shift)
{
 return (word >> shift) | (word << (16 - shift));
}






static inline __attribute__((no_instrument_function)) __u8 rol8(__u8 word, unsigned int shift)
{
 return (word << shift) | (word >> (8 - shift));
}






static inline __attribute__((no_instrument_function)) __u8 ror8(__u8 word, unsigned int shift)
{
 return (word >> shift) | (word << (8 - shift));
}






static inline __attribute__((no_instrument_function)) __s32 sign_extend32(__u32 value, int index)
{
 __u8 shift = 31 - index;
 return (__s32)(value << shift) >> shift;
}

static inline __attribute__((no_instrument_function)) unsigned fls_long(unsigned long l)
{
 if (sizeof(l) == 4)
  return fls(l);
 return fls64(l);
}
static inline __attribute__((no_instrument_function)) unsigned long __ffs64(u64 word)
{






 return __ffs((unsigned long)word);
}
extern unsigned long find_last_bit(const unsigned long *addr,
       unsigned long size);
extern __attribute__((const, noreturn))
int ____ilog2_NaN(void);
static inline __attribute__((no_instrument_function)) __attribute__((const))
int __ilog2_u32(u32 n)
{
 return fls(n) - 1;
}



static inline __attribute__((no_instrument_function)) __attribute__((const))
int __ilog2_u64(u64 n)
{
 return fls64(n) - 1;
}







static inline __attribute__((no_instrument_function)) __attribute__((const))
bool is_power_of_2(unsigned long n)
{
 return (n != 0 && ((n & (n - 1)) == 0));
}




static inline __attribute__((no_instrument_function)) __attribute__((const))
unsigned long __roundup_pow_of_two(unsigned long n)
{
 return 1UL << fls_long(n - 1);
}




static inline __attribute__((no_instrument_function)) __attribute__((const))
unsigned long __rounddown_pow_of_two(unsigned long n)
{
 return 1UL << (fls_long(n) - 1);
}




typedef int (*initcall_t)(void);
typedef void (*exitcall_t)(void);

extern initcall_t __con_initcall_start[], __con_initcall_end[];
extern initcall_t __security_initcall_start[], __security_initcall_end[];


typedef void (*ctor_fn_t)(void);


extern int do_one_initcall(initcall_t fn);
extern char __attribute__ ((__section__(".init.data"))) boot_command_line[];
extern char *saved_command_line;
extern unsigned int reset_devices;


void setup_arch(char **);
void prepare_namespace(void);
void __attribute__ ((__section__(".init.text"))) __attribute__((__cold__)) __attribute__((no_instrument_function)) load_default_modules(void);
int __attribute__ ((__section__(".init.text"))) __attribute__((__cold__)) __attribute__((no_instrument_function)) init_rootfs(void);

extern void (*late_time_init)(void);

extern bool initcall_debug;













struct sysinfo {
 __kernel_long_t uptime;
 __kernel_ulong_t loads[3];
 __kernel_ulong_t totalram;
 __kernel_ulong_t freeram;
 __kernel_ulong_t sharedram;
 __kernel_ulong_t bufferram;
 __kernel_ulong_t totalswap;
 __kernel_ulong_t freeswap;
 __u16 procs;
 __u16 pad;
 __kernel_ulong_t totalhigh;
 __kernel_ulong_t freehigh;
 __u32 mem_unit;
 char _f[20-2*sizeof(__kernel_ulong_t)-sizeof(__u32)];
};

extern const char linux_banner[];
extern const char linux_proc_banner[];

extern char *log_buf_addr_get(void);
extern u32 log_buf_len_get(void);

static inline __attribute__((no_instrument_function)) int printk_get_level(const char *buffer)
{
 if (buffer[0] == '\001' && buffer[1]) {
  switch (buffer[1]) {
  case '0' ... '7':
  case 'd':
   return buffer[1];
  }
 }
 return 0;
}

static inline __attribute__((no_instrument_function)) const char *printk_skip_level(const char *buffer)
{
 if (printk_get_level(buffer))
  return buffer + 2;

 return buffer;
}
extern int console_printk[];






static inline __attribute__((no_instrument_function)) void console_silent(void)
{
 (console_printk[0]) = 0;
}

static inline __attribute__((no_instrument_function)) void console_verbose(void)
{
 if ((console_printk[0]))
  (console_printk[0]) = 15;
}

struct va_format {
 const char *fmt;
 va_list *va;
};
static inline __attribute__((no_instrument_function)) __attribute__((format(printf, 1, 2)))
int no_printk(const char *fmt, ...)
{
 return 0;
}


extern __attribute__((format(printf, 1, 2)))
void early_printk(const char *fmt, ...);
void early_vprintk(const char *fmt, va_list ap);






 __attribute__((format(printf, 5, 0)))
int vprintk_emit(int facility, int level,
   const char *dict, size_t dictlen,
   const char *fmt, va_list args);

 __attribute__((format(printf, 1, 0)))
int vprintk(const char *fmt, va_list args);

 __attribute__((format(printf, 5, 6))) __attribute__((__cold__))
int printk_emit(int facility, int level,
  const char *dict, size_t dictlen,
  const char *fmt, ...);

 __attribute__((format(printf, 1, 2))) __attribute__((__cold__))
int printk(const char *fmt, ...);




__attribute__((format(printf, 1, 2))) __attribute__((__cold__)) int printk_deferred(const char *fmt, ...);






extern int __printk_ratelimit(const char *func);

extern bool printk_timed_ratelimit(unsigned long *caller_jiffies,
       unsigned int interval_msec);

extern int printk_delay_msec;
extern int dmesg_restrict;
extern int kptr_restrict;

extern void wake_up_klogd(void);

void log_buf_kexec_setup(void);
void __attribute__ ((__section__(".init.text"))) __attribute__((__cold__)) __attribute__((no_instrument_function)) setup_log_buf(int early);
void dump_stack_set_arch_desc(const char *fmt, ...);
void dump_stack_print_info(const char *log_lvl);
void show_regs_print_info(const char *log_lvl);
extern void dump_stack(void) __attribute__((__cold__));
struct _ddebug {




 const char *modname;
 const char *function;
 const char *filename;
 const char *format;
 unsigned int lineno:18;
 unsigned int flags:8;
} __attribute__((aligned(8)));


int ddebug_add_module(struct _ddebug *tab, unsigned int n,
    const char *modname);

extern char *strndup_user(const char *, long);
extern void *memdup_user(const void *, size_t);







static inline __attribute__((no_instrument_function)) __attribute__((always_inline)) void *__inline_memcpy(void *to, const void *from, size_t n)
{
 unsigned long d0, d1, d2;
 asm volatile("rep ; movsl\n\t"
       "testb $2,%b4\n\t"
       "je 1f\n\t"
       "movsw\n"
       "1:\ttestb $1,%b4\n\t"
       "je 2f\n\t"
       "movsb\n"
       "2:"
       : "=&c" (d0), "=&D" (d1), "=&S" (d2)
       : "0" (n / 4), "q" (n), "1" ((long)to), "2" ((long)from)
       : "memory");
 return to;
}







extern void *memcpy(void *to, const void *from, size_t len);
void *memset(void *s, int c, size_t n);


void *memmove(void *dest, const void *src, size_t count);

int memcmp(const void *cs, const void *ct, size_t count);
size_t strlen(const char *s);
char *strcpy(char *dest, const char *src);
char *strcat(char *dest, const char *src);
int strcmp(const char *cs, const char *ct);


extern char * strcpy(char *,const char *);


extern char * strncpy(char *,const char *, __kernel_size_t);


size_t strlcpy(char *, const char *, size_t);


extern char * strcat(char *, const char *);


extern char * strncat(char *, const char *, __kernel_size_t);


extern size_t strlcat(char *, const char *, __kernel_size_t);


extern int strcmp(const char *,const char *);


extern int strncmp(const char *,const char *,__kernel_size_t);


extern int strnicmp(const char *, const char *, __kernel_size_t);


extern int strcasecmp(const char *s1, const char *s2);


extern int strncasecmp(const char *s1, const char *s2, size_t n);


extern char * strchr(const char *,int);


extern char * strchrnul(const char *,int);


extern char * strnchr(const char *, size_t, int);


extern char * strrchr(const char *,int);

extern char * skip_spaces(const char *);

extern char *strim(char *);

static inline __attribute__((no_instrument_function)) char *strstrip(char *str)
{
 return strim(str);
}


extern char * strstr(const char *, const char *);


extern char * strnstr(const char *, const char *, size_t);


extern __kernel_size_t strlen(const char *);


extern __kernel_size_t strnlen(const char *,__kernel_size_t);


extern char * strpbrk(const char *,const char *);


extern char * strsep(char **,const char *);


extern __kernel_size_t strspn(const char *,const char *);


extern __kernel_size_t strcspn(const char *,const char *);
extern void * memscan(void *,int,__kernel_size_t);


extern int memcmp(const void *,const void *,__kernel_size_t);


extern void * memchr(const void *,int,__kernel_size_t);

void *memchr_inv(const void *s, int c, size_t n);

extern char *kstrdup(const char *s, gfp_t gfp);
extern char *kstrndup(const char *s, size_t len, gfp_t gfp);
extern void *kmemdup(const void *src, size_t len, gfp_t gfp);

extern char **argv_split(gfp_t gfp, const char *str, int *argcp);
extern void argv_free(char **argv);

extern bool sysfs_streq(const char *s1, const char *s2);
extern int strtobool(const char *s, bool *res);


int vbin_printf(u32 *bin_buf, size_t size, const char *fmt, va_list args);
int bstr_printf(char *buf, size_t size, const char *fmt, const u32 *bin_buf);
int bprintf(u32 *bin_buf, size_t size, const char *fmt, ...) __attribute__((format(printf, 3, 4)));


extern ssize_t memory_read_from_buffer(void *to, size_t count, loff_t *ppos,
   const void *from, size_t available);






static inline __attribute__((no_instrument_function)) bool strstarts(const char *str, const char *prefix)
{
 return strncmp(str, prefix, strlen(prefix)) == 0;
}

extern size_t memweight(const void *ptr, size_t bytes);






static inline __attribute__((no_instrument_function)) const char *kbasename(const char *path)
{
 const char *tail = strrchr(path, '/');
 return tail ? tail + 1 : path;
}







static inline __attribute__((no_instrument_function)) int ddebug_remove_module(const char *mod)
{
 return 0;
}

static inline __attribute__((no_instrument_function)) int ddebug_dyndbg_module_param_cb(char *param, char *val,
      const char *modname)
{
 if (strstr(param, "dyndbg")) {

  printk("\001" "4" "dyndbg param is supported only in "
   "CONFIG_DYNAMIC_DEBUG builds\n");
  return 0;
 }
 return -22;
}
extern const struct file_operations kmsg_fops;

enum {
 DUMP_PREFIX_NONE,
 DUMP_PREFIX_ADDRESS,
 DUMP_PREFIX_OFFSET
};
extern void hex_dump_to_buffer(const void *buf, size_t len,
          int rowsize, int groupsize,
          char *linebuf, size_t linebuflen, bool ascii);

extern void print_hex_dump(const char *level, const char *prefix_str,
      int prefix_type, int rowsize, int groupsize,
      const void *buf, size_t len, bool ascii);




extern void print_hex_dump_bytes(const char *prefix_str, int prefix_type,
     const void *buf, size_t len);
struct completion;
struct pt_regs;
struct user;


extern int _cond_resched(void);
  static inline __attribute__((no_instrument_function)) void __might_sleep(const char *file, int line,
       int preempt_offset) { }
static inline __attribute__((no_instrument_function)) u32 reciprocal_scale(u32 val, u32 ep_ro)
{
 return (u32)(((u64) val * ep_ro) >> 32);
}





static inline __attribute__((no_instrument_function)) void might_fault(void) { }


extern struct atomic_notifier_head panic_notifier_list;
extern long (*panic_blink)(int state);
__attribute__((format(printf, 1, 2)))
void panic(const char *fmt, ...)
 __attribute__((noreturn)) __attribute__((__cold__));
extern void oops_enter(void);
extern void oops_exit(void);
void print_oops_end_marker(void);
extern int oops_may_print(void);
void do_exit(long error_code)
 __attribute__((noreturn));
void complete_and_exit(struct completion *, long)
 __attribute__((noreturn));


int _kstrtoul(const char *s, unsigned int base, unsigned long *res);
int _kstrtol(const char *s, unsigned int base, long *res);

int kstrtoull(const char *s, unsigned int base, unsigned long long *res);
int kstrtoll(const char *s, unsigned int base, long long *res);
static inline __attribute__((no_instrument_function)) int kstrtoul(const char *s, unsigned int base, unsigned long *res)
{




 if (sizeof(unsigned long) == sizeof(unsigned long long) &&
     __alignof__(unsigned long) == __alignof__(unsigned long long))
  return kstrtoull(s, base, (unsigned long long *)res);
 else
  return _kstrtoul(s, base, res);
}
static inline __attribute__((no_instrument_function)) int kstrtol(const char *s, unsigned int base, long *res)
{




 if (sizeof(long) == sizeof(long long) &&
     __alignof__(long) == __alignof__(long long))
  return kstrtoll(s, base, (long long *)res);
 else
  return _kstrtol(s, base, res);
}

int kstrtouint(const char *s, unsigned int base, unsigned int *res);
int kstrtoint(const char *s, unsigned int base, int *res);

static inline __attribute__((no_instrument_function)) int kstrtou64(const char *s, unsigned int base, u64 *res)
{
 return kstrtoull(s, base, res);
}

static inline __attribute__((no_instrument_function)) int kstrtos64(const char *s, unsigned int base, s64 *res)
{
 return kstrtoll(s, base, res);
}

static inline __attribute__((no_instrument_function)) int kstrtou32(const char *s, unsigned int base, u32 *res)
{
 return kstrtouint(s, base, res);
}

static inline __attribute__((no_instrument_function)) int kstrtos32(const char *s, unsigned int base, s32 *res)
{
 return kstrtoint(s, base, res);
}

int kstrtou16(const char *s, unsigned int base, u16 *res);
int kstrtos16(const char *s, unsigned int base, s16 *res);
int kstrtou8(const char *s, unsigned int base, u8 *res);
int kstrtos8(const char *s, unsigned int base, s8 *res);

int kstrtoull_from_user(const char *s, size_t count, unsigned int base, unsigned long long *res);
int kstrtoll_from_user(const char *s, size_t count, unsigned int base, long long *res);
int kstrtoul_from_user(const char *s, size_t count, unsigned int base, unsigned long *res);
int kstrtol_from_user(const char *s, size_t count, unsigned int base, long *res);
int kstrtouint_from_user(const char *s, size_t count, unsigned int base, unsigned int *res);
int kstrtoint_from_user(const char *s, size_t count, unsigned int base, int *res);
int kstrtou16_from_user(const char *s, size_t count, unsigned int base, u16 *res);
int kstrtos16_from_user(const char *s, size_t count, unsigned int base, s16 *res);
int kstrtou8_from_user(const char *s, size_t count, unsigned int base, u8 *res);
int kstrtos8_from_user(const char *s, size_t count, unsigned int base, s8 *res);

static inline __attribute__((no_instrument_function)) int kstrtou64_from_user(const char *s, size_t count, unsigned int base, u64 *res)
{
 return kstrtoull_from_user(s, count, base, res);
}

static inline __attribute__((no_instrument_function)) int kstrtos64_from_user(const char *s, size_t count, unsigned int base, s64 *res)
{
 return kstrtoll_from_user(s, count, base, res);
}

static inline __attribute__((no_instrument_function)) int kstrtou32_from_user(const char *s, size_t count, unsigned int base, u32 *res)
{
 return kstrtouint_from_user(s, count, base, res);
}

static inline __attribute__((no_instrument_function)) int kstrtos32_from_user(const char *s, size_t count, unsigned int base, s32 *res)
{
 return kstrtoint_from_user(s, count, base, res);
}



extern unsigned long simple_strtoul(const char *,char **,unsigned int);
extern long simple_strtol(const char *,char **,unsigned int);
extern unsigned long long simple_strtoull(const char *,char **,unsigned int);
extern long long simple_strtoll(const char *,char **,unsigned int);





extern int num_to_str(char *buf, int size, unsigned long long num);



extern __attribute__((format(printf, 2, 3))) int sprintf(char *buf, const char * fmt, ...);
extern __attribute__((format(printf, 2, 0))) int vsprintf(char *buf, const char *, va_list);
extern __attribute__((format(printf, 3, 4)))
int snprintf(char *buf, size_t size, const char *fmt, ...);
extern __attribute__((format(printf, 3, 0)))
int vsnprintf(char *buf, size_t size, const char *fmt, va_list args);
extern __attribute__((format(printf, 3, 4)))
int scnprintf(char *buf, size_t size, const char *fmt, ...);
extern __attribute__((format(printf, 3, 0)))
int vscnprintf(char *buf, size_t size, const char *fmt, va_list args);
extern __attribute__((format(printf, 2, 3)))
char *kasprintf(gfp_t gfp, const char *fmt, ...);
extern char *kvasprintf(gfp_t gfp, const char *fmt, va_list args);

extern __attribute__((format(scanf, 2, 3)))
int sscanf(const char *, const char *, ...);
extern __attribute__((format(scanf, 2, 0)))
int vsscanf(const char *, const char *, va_list);

extern int get_option(char **str, int *pint);
extern char *get_options(const char *str, int nints, int *ints);
extern unsigned long long memparse(const char *ptr, char **retptr);

extern int core_kernel_text(unsigned long addr);
extern int core_kernel_data(unsigned long addr);
extern int __kernel_text_address(unsigned long addr);
extern int kernel_text_address(unsigned long addr);
extern int func_ptr_is_kernel_text(void *ptr);

struct pid;
extern struct pid *session_of_pgrp(struct pid *pgrp);

unsigned long int_sqrt(unsigned long);

extern void bust_spinlocks(int yes);
extern int oops_in_progress;
extern int panic_timeout;
extern int panic_on_oops;
extern int panic_on_unrecovered_nmi;
extern int panic_on_io_nmi;
extern int sysctl_panic_on_stackoverflow;




static inline __attribute__((no_instrument_function)) void set_arch_panic_timeout(int timeout, int arch_default_timeout)
{
 if (panic_timeout == arch_default_timeout)
  panic_timeout = timeout;
}
extern const char *print_tainted(void);
enum lockdep_ok {
 LOCKDEP_STILL_OK,
 LOCKDEP_NOW_UNRELIABLE
};
extern void add_taint(unsigned flag, enum lockdep_ok);
extern int test_taint(unsigned flag);
extern unsigned long get_taint(void);
extern int root_mountflags;

extern bool early_boot_irqs_disabled;


extern enum system_states {
 SYSTEM_BOOTING,
 SYSTEM_RUNNING,
 SYSTEM_HALT,
 SYSTEM_POWER_OFF,
 SYSTEM_RESTART,
} system_state;
extern const char hex_asc[];



static inline __attribute__((no_instrument_function)) char *hex_byte_pack(char *buf, u8 byte)
{
 *buf++ = hex_asc[((byte) & 0xf0) >> 4];
 *buf++ = hex_asc[((byte) & 0x0f)];
 return buf;
}

extern const char hex_asc_upper[];



static inline __attribute__((no_instrument_function)) char *hex_byte_pack_upper(char *buf, u8 byte)
{
 *buf++ = hex_asc_upper[((byte) & 0xf0) >> 4];
 *buf++ = hex_asc_upper[((byte) & 0x0f)];
 return buf;
}

extern int hex_to_bin(char ch);
extern int hex2bin(u8 *dst, const char *src, size_t count);

bool mac_pton(const char *s, u8 *mac);
void tracing_off_permanent(void);




enum ftrace_dump_mode {
 DUMP_NONE,
 DUMP_ALL,
 DUMP_ORIG,
};


void tracing_on(void);
void tracing_off(void);
int tracing_is_on(void);
void tracing_snapshot(void);
void tracing_snapshot_alloc(void);

extern void tracing_start(void);
extern void tracing_stop(void);

static inline __attribute__((no_instrument_function)) __attribute__((format(printf, 1, 2)))
void ____trace_printk_check_format(const char *fmt, ...)
{
}
extern __attribute__((format(printf, 2, 3)))
int __trace_bprintk(unsigned long ip, const char *fmt, ...);

extern __attribute__((format(printf, 2, 3)))
int __trace_printk(unsigned long ip, const char *fmt, ...);
extern int __trace_bputs(unsigned long ip, const char *str);
extern int __trace_puts(unsigned long ip, const char *str, int size);

extern void trace_dump_stack(int skip);
extern int
__ftrace_vbprintk(unsigned long ip, const char *fmt, va_list ap);

extern int
__ftrace_vprintk(unsigned long ip, const char *fmt, va_list ap);

extern void ftrace_dump(enum ftrace_dump_mode oops_dump_mode);

extern unsigned long loops_per_jiffy;








extern void __bad_udelay(void);
extern void __bad_ndelay(void);

extern void __udelay(unsigned long usecs);
extern void __ndelay(unsigned long nsecs);
extern void __const_udelay(unsigned long xloops);
extern void __delay(unsigned long loops);

void use_tsc_delay(void);
extern unsigned long lpj_fine;
void calibrate_delay(void);
void msleep(unsigned int msecs);
unsigned long msleep_interruptible(unsigned int msecs);
void usleep_range(unsigned long min, unsigned long max);

static inline __attribute__((no_instrument_function)) void ssleep(unsigned int seconds)
{
 msleep(seconds * 1000);
}





static inline __attribute__((no_instrument_function)) void INIT_LIST_HEAD(struct list_head *list)
{
 list->next = list;
 list->prev = list;
}
static inline __attribute__((no_instrument_function)) void __list_add(struct list_head *new,
         struct list_head *prev,
         struct list_head *next)
{
 next->prev = new;
 new->next = next;
 new->prev = prev;
 prev->next = new;
}
static inline __attribute__((no_instrument_function)) void list_add(struct list_head *new, struct list_head *head)
{
 __list_add(new, head, head->next);
}
static inline __attribute__((no_instrument_function)) void list_add_tail(struct list_head *new, struct list_head *head)
{
 __list_add(new, head->prev, head);
}
static inline __attribute__((no_instrument_function)) void __list_del(struct list_head * prev, struct list_head * next)
{
 next->prev = prev;
 prev->next = next;
}
static inline __attribute__((no_instrument_function)) void __list_del_entry(struct list_head *entry)
{
 __list_del(entry->prev, entry->next);
}

static inline __attribute__((no_instrument_function)) void list_del(struct list_head *entry)
{
 __list_del(entry->prev, entry->next);
 entry->next = ((void *) 0x00100100 + (0xdead000000000000UL));
 entry->prev = ((void *) 0x00200200 + (0xdead000000000000UL));
}
static inline __attribute__((no_instrument_function)) void list_replace(struct list_head *old,
    struct list_head *new)
{
 new->next = old->next;
 new->next->prev = new;
 new->prev = old->prev;
 new->prev->next = new;
}

static inline __attribute__((no_instrument_function)) void list_replace_init(struct list_head *old,
     struct list_head *new)
{
 list_replace(old, new);
 INIT_LIST_HEAD(old);
}





static inline __attribute__((no_instrument_function)) void list_del_init(struct list_head *entry)
{
 __list_del_entry(entry);
 INIT_LIST_HEAD(entry);
}






static inline __attribute__((no_instrument_function)) void list_move(struct list_head *list, struct list_head *head)
{
 __list_del_entry(list);
 list_add(list, head);
}






static inline __attribute__((no_instrument_function)) void list_move_tail(struct list_head *list,
      struct list_head *head)
{
 __list_del_entry(list);
 list_add_tail(list, head);
}






static inline __attribute__((no_instrument_function)) int list_is_last(const struct list_head *list,
    const struct list_head *head)
{
 return list->next == head;
}





static inline __attribute__((no_instrument_function)) int list_empty(const struct list_head *head)
{
 return head->next == head;
}
static inline __attribute__((no_instrument_function)) int list_empty_careful(const struct list_head *head)
{
 struct list_head *next = head->next;
 return (next == head) && (next == head->prev);
}





static inline __attribute__((no_instrument_function)) void list_rotate_left(struct list_head *head)
{
 struct list_head *first;

 if (!list_empty(head)) {
  first = head->next;
  list_move_tail(first, head);
 }
}





static inline __attribute__((no_instrument_function)) int list_is_singular(const struct list_head *head)
{
 return !list_empty(head) && (head->next == head->prev);
}

static inline __attribute__((no_instrument_function)) void __list_cut_position(struct list_head *list,
  struct list_head *head, struct list_head *entry)
{
 struct list_head *new_first = entry->next;
 list->next = head->next;
 list->next->prev = list;
 list->prev = entry;
 entry->next = list;
 head->next = new_first;
 new_first->prev = head;
}
static inline __attribute__((no_instrument_function)) void list_cut_position(struct list_head *list,
  struct list_head *head, struct list_head *entry)
{
 if (list_empty(head))
  return;
 if (list_is_singular(head) &&
  (head->next != entry && head != entry))
  return;
 if (entry == head)
  INIT_LIST_HEAD(list);
 else
  __list_cut_position(list, head, entry);
}

static inline __attribute__((no_instrument_function)) void __list_splice(const struct list_head *list,
     struct list_head *prev,
     struct list_head *next)
{
 struct list_head *first = list->next;
 struct list_head *last = list->prev;

 first->prev = prev;
 prev->next = first;

 last->next = next;
 next->prev = last;
}






static inline __attribute__((no_instrument_function)) void list_splice(const struct list_head *list,
    struct list_head *head)
{
 if (!list_empty(list))
  __list_splice(list, head, head->next);
}






static inline __attribute__((no_instrument_function)) void list_splice_tail(struct list_head *list,
    struct list_head *head)
{
 if (!list_empty(list))
  __list_splice(list, head->prev, head);
}
static inline __attribute__((no_instrument_function)) void list_splice_init(struct list_head *list,
        struct list_head *head)
{
 if (!list_empty(list)) {
  __list_splice(list, head, head->next);
  INIT_LIST_HEAD(list);
 }
}
static inline __attribute__((no_instrument_function)) void list_splice_tail_init(struct list_head *list,
      struct list_head *head)
{
 if (!list_empty(list)) {
  __list_splice(list, head->prev, head);
  INIT_LIST_HEAD(list);
 }
}
static inline __attribute__((no_instrument_function)) void INIT_HLIST_NODE(struct hlist_node *h)
{
 h->next = ((void *)0);
 h->pprev = ((void *)0);
}

static inline __attribute__((no_instrument_function)) int hlist_unhashed(const struct hlist_node *h)
{
 return !h->pprev;
}

static inline __attribute__((no_instrument_function)) int hlist_empty(const struct hlist_head *h)
{
 return !h->first;
}

static inline __attribute__((no_instrument_function)) void __hlist_del(struct hlist_node *n)
{
 struct hlist_node *next = n->next;
 struct hlist_node **pprev = n->pprev;
 *pprev = next;
 if (next)
  next->pprev = pprev;
}

static inline __attribute__((no_instrument_function)) void hlist_del(struct hlist_node *n)
{
 __hlist_del(n);
 n->next = ((void *) 0x00100100 + (0xdead000000000000UL));
 n->pprev = ((void *) 0x00200200 + (0xdead000000000000UL));
}

static inline __attribute__((no_instrument_function)) void hlist_del_init(struct hlist_node *n)
{
 if (!hlist_unhashed(n)) {
  __hlist_del(n);
  INIT_HLIST_NODE(n);
 }
}

static inline __attribute__((no_instrument_function)) void hlist_add_head(struct hlist_node *n, struct hlist_head *h)
{
 struct hlist_node *first = h->first;
 n->next = first;
 if (first)
  first->pprev = &n->next;
 h->first = n;
 n->pprev = &h->first;
}


static inline __attribute__((no_instrument_function)) void hlist_add_before(struct hlist_node *n,
     struct hlist_node *next)
{
 n->pprev = next->pprev;
 n->next = next;
 next->pprev = &n->next;
 *(n->pprev) = n;
}

static inline __attribute__((no_instrument_function)) void hlist_add_behind(struct hlist_node *n,
        struct hlist_node *prev)
{
 n->next = prev->next;
 prev->next = n;
 n->pprev = &prev->next;

 if (n->next)
  n->next->pprev = &n->next;
}


static inline __attribute__((no_instrument_function)) void hlist_add_fake(struct hlist_node *n)
{
 n->pprev = &n->next;
}





static inline __attribute__((no_instrument_function)) void hlist_move_list(struct hlist_head *old,
       struct hlist_head *new)
{
 new->first = old->first;
 if (new->first)
  new->first->pprev = &new->first;
 old->first = ((void *)0);
}







struct stat {
 __kernel_ulong_t st_dev;
 __kernel_ulong_t st_ino;
 __kernel_ulong_t st_nlink;

 unsigned int st_mode;
 unsigned int st_uid;
 unsigned int st_gid;
 unsigned int __pad0;
 __kernel_ulong_t st_rdev;
 __kernel_long_t st_size;
 __kernel_long_t st_blksize;
 __kernel_long_t st_blocks;

 __kernel_ulong_t st_atime;
 __kernel_ulong_t st_atime_nsec;
 __kernel_ulong_t st_mtime;
 __kernel_ulong_t st_mtime_nsec;
 __kernel_ulong_t st_ctime;
 __kernel_ulong_t st_ctime_nsec;
 __kernel_long_t __unused[3];
};
struct __old_kernel_stat {
 unsigned short st_dev;
 unsigned short st_ino;
 unsigned short st_mode;
 unsigned short st_nlink;
 unsigned short st_uid;
 unsigned short st_gid;
 unsigned short st_rdev;






 unsigned int st_size;
 unsigned int st_atime;
 unsigned int st_mtime;
 unsigned int st_ctime;

};








extern void __bad_percpu_size(void);
static inline __attribute__((no_instrument_function)) __attribute__((always_inline)) int x86_this_cpu_constant_test_bit(unsigned int nr,
                        const unsigned long *addr)
{
 unsigned long *a = (unsigned long *)addr + nr / 64;


 return ((1UL << (nr % 64)) & ({ typeof((*a)) pfo_ret__; switch (sizeof((*a))) { case 1: asm("mov" "b ""%%""gs"":" "%P" "1"",%0" : "=q" (pfo_ret__) : "m"(*a)); break; case 2: asm("mov" "w ""%%""gs"":" "%P" "1"",%0" : "=r" (pfo_ret__) : "m"(*a)); break; case 4: asm("mov" "l ""%%""gs"":" "%P" "1"",%0" : "=r" (pfo_ret__) : "m"(*a)); break; case 8: asm("mov" "q ""%%""gs"":" "%P" "1"",%0" : "=r" (pfo_ret__) : "m"(*a)); break; default: __bad_percpu_size(); } pfo_ret__; })) != 0;



}

static inline __attribute__((no_instrument_function)) int x86_this_cpu_variable_test_bit(int nr,
                        const unsigned long *addr)
{
 int oldbit;

 asm volatile("bt ""%%""gs"":" "%P" "2"",%1\n\t"
   "sbb %0,%0"
   : "=r" (oldbit)
   : "m" (*(unsigned long *)addr), "Ir" (nr));

 return oldbit;
}











extern void __bad_size_call_parameter(void);




static inline __attribute__((no_instrument_function)) void __this_cpu_preempt_check(const char *op) { }
extern unsigned long __per_cpu_offset[256];
extern void setup_per_cpu_areas(void);


extern __attribute__((section(".data..percpu" ""))) __typeof__(unsigned long) this_cpu_off;



struct bug_entry {



 signed int bug_addr_disp;





 signed int file_disp;

 unsigned short line;

 unsigned short flags;
};
extern __attribute__((format(printf, 3, 4)))
void warn_slowpath_fmt(const char *file, const int line,
         const char *fmt, ...);
extern __attribute__((format(printf, 4, 5)))
void warn_slowpath_fmt_taint(const char *file, const int line, unsigned taint,
        const char *fmt, ...);
extern void warn_slowpath_null(const char *file, const int line);


enum bug_trap_type {
 BUG_TRAP_TYPE_NONE = 0,
 BUG_TRAP_TYPE_WARN = 1,
 BUG_TRAP_TYPE_BUG = 2,
};

struct pt_regs;
static inline __attribute__((no_instrument_function)) int is_warning_bug(const struct bug_entry *bug)
{
 return bug->flags & (1 << 0);
}

const struct bug_entry *find_bug(unsigned long bugaddr);

enum bug_trap_type report_bug(unsigned long bug_addr, struct pt_regs *regs);


int is_valid_bugaddr(unsigned long addr);

struct timespec;
struct compat_timespec;




struct restart_block {
 long (*fn)(struct restart_block *);
 union {

  struct {
   u32 *uaddr;
   u32 val;
   u32 flags;
   u32 bitset;
   u64 time;
   u32 *uaddr2;
  } futex;

  struct {
   clockid_t clockid;
   struct timespec *rmtp;

   struct compat_timespec *compat_rmtp;

   u64 expires;
  } nanosleep;

  struct {
   struct pollfd *ufds;
   int nfds;
   int has_timeout;
   unsigned long tv_sec;
   unsigned long tv_nsec;
  } poll;
 };
};

extern long do_no_restart_syscall(struct restart_block *parm);


extern unsigned long max_pfn;
extern unsigned long phys_base;

static inline __attribute__((no_instrument_function)) unsigned long __phys_addr_nodebug(unsigned long x)
{
 unsigned long y = x - (0xffffffff80000000UL);


 x = y + ((x > y) ? phys_base : ((0xffffffff80000000UL) - ((unsigned long)(0xffff880000000000UL))));

 return x;
}
void clear_page(void *page);
void copy_page(void *to, void *from);






struct page;




struct range {
 u64 start;
 u64 end;
};

int add_range(struct range *range, int az, int nr_range,
  u64 start, u64 end);


int add_range_with_merge(struct range *range, int az, int nr_range,
    u64 start, u64 end);

void subtract_range(struct range *range, int az, u64 start, u64 end);

int clean_sort_range(struct range *range, int az);

void sort_range(struct range *range, int nr_range);


static inline __attribute__((no_instrument_function)) resource_size_t cap_resource(u64 val)
{
 if (val > ((resource_size_t)~0))
  return ((resource_size_t)~0);

 return val;
}
extern struct range pfn_mapped[];
extern int nr_pfn_mapped;

static inline __attribute__((no_instrument_function)) void clear_user_page(void *page, unsigned long vaddr,
       struct page *pg)
{
 clear_page(page);
}

static inline __attribute__((no_instrument_function)) void copy_user_page(void *to, void *from, unsigned long vaddr,
      struct page *topage)
{
 copy_page(to, from);
}
extern bool __virt_addr_valid(unsigned long kaddr);




static inline __attribute__((no_instrument_function)) __attribute__((__const__))
int __get_order(unsigned long size)
{
 int order;

 size--;
 size >>= 12;



 order = fls64(size);

 return order;
}
struct task_struct;
struct exec_domain;






struct task_struct;
struct mm_struct;






struct vm86_regs {



 long ebx;
 long ecx;
 long edx;
 long esi;
 long edi;
 long ebp;
 long eax;
 long __null_ds;
 long __null_es;
 long __null_fs;
 long __null_gs;
 long orig_eax;
 long eip;
 unsigned short cs, __csh;
 long eflags;
 long esp;
 unsigned short ss, __ssh;



 unsigned short es, __esh;
 unsigned short ds, __dsh;
 unsigned short fs, __fsh;
 unsigned short gs, __gsh;
};

struct revectored_struct {
 unsigned long __map[8];
};

struct vm86_struct {
 struct vm86_regs regs;
 unsigned long flags;
 unsigned long screen_bitmap;
 unsigned long cpu_type;
 struct revectored_struct int_revectored;
 struct revectored_struct int21_revectored;
};






struct vm86plus_info_struct {
 unsigned long force_return_for_pic:1;
 unsigned long vm86dbg_active:1;
 unsigned long vm86dbg_TFpendig:1;
 unsigned long unused:28;
 unsigned long is_vm86pus:1;
 unsigned char vm86dbg_intxxtab[32];
};
struct vm86plus_struct {
 struct vm86_regs regs;
 unsigned long flags;
 unsigned long screen_bitmap;
 unsigned long cpu_type;
 struct revectored_struct int_revectored;
 struct revectored_struct int21_revectored;
 struct vm86plus_info_struct vm86plus;
};
struct kernel_vm86_regs {



 struct pt_regs pt;



 unsigned short es, __esh;
 unsigned short ds, __dsh;
 unsigned short fs, __fsh;
 unsigned short gs, __gsh;
};

struct kernel_vm86_struct {
 struct kernel_vm86_regs regs;
 unsigned long flags;
 unsigned long screen_bitmap;
 unsigned long cpu_type;
 struct revectored_struct int_revectored;
 struct revectored_struct int21_revectored;
 struct vm86plus_info_struct vm86plus;
 struct pt_regs *regs32;
};
static inline __attribute__((no_instrument_function)) int handle_vm86_trap(struct kernel_vm86_regs *a, long b, int c)
{
 return 0;
}
struct math_emu_info {
 long ___orig_eip;
 union {
  struct pt_regs *regs;
  struct kernel_vm86_regs *vm86;
 };
};





struct _fpx_sw_bytes {
 __u32 magic1;
 __u32 extended_size;


 __u64 xstate_bv;




 __u32 xstate_size;




 __u32 padding[7];
};
struct _fpstate {
 __u16 cwd;
 __u16 swd;
 __u16 twd;

 __u16 fop;
 __u64 rip;
 __u64 rdp;
 __u32 mxcsr;
 __u32 mxcsr_mask;
 __u32 st_space[32];
 __u32 xmm_space[64];
 __u32 reserved2[12];
 union {
  __u32 reserved3[12];
  struct _fpx_sw_bytes sw_reserved;

 };
};
struct _xsave_hdr {
 __u64 xstate_bv;
 __u64 reserved1[2];
 __u64 reserved2[5];
};

struct _ymmh_state {

 __u32 ymmh_space[64];
};







struct _xstate {
 struct _fpstate fpstate;
 struct _xsave_hdr xstate_hdr;
 struct _ymmh_state ymmh;

};
struct sigcontext {
 unsigned long r8;
 unsigned long r9;
 unsigned long r10;
 unsigned long r11;
 unsigned long r12;
 unsigned long r13;
 unsigned long r14;
 unsigned long r15;
 unsigned long di;
 unsigned long si;
 unsigned long bp;
 unsigned long bx;
 unsigned long dx;
 unsigned long ax;
 unsigned long cx;
 unsigned long sp;
 unsigned long ip;
 unsigned long flags;
 unsigned short cs;
 unsigned short gs;
 unsigned short fs;
 unsigned short __pad0;
 unsigned long err;
 unsigned long trapno;
 unsigned long oldmask;
 unsigned long cr2;
 void *fpstate;
 unsigned long reserved1[8];
};







struct task_struct;

extern __attribute__((section(".data..percpu" ""))) __typeof__(struct task_struct *) current_task;

static inline __attribute__((no_instrument_function)) __attribute__((always_inline)) struct task_struct *get_current(void)
{
 return ({ typeof(current_task) pfo_ret__; switch (sizeof(current_task)) { case 1: asm("mov" "b ""%%""gs"":" "%P" "1"",%0" : "=q" (pfo_ret__) : "p" (&(current_task))); break; case 2: asm("mov" "w ""%%""gs"":" "%P" "1"",%0" : "=r" (pfo_ret__) : "p" (&(current_task))); break; case 4: asm("mov" "l ""%%""gs"":" "%P" "1"",%0" : "=r" (pfo_ret__) : "p" (&(current_task))); break; case 8: asm("mov" "q ""%%""gs"":" "%P" "1"",%0" : "=r" (pfo_ret__) : "p" (&(current_task))); break; default: __bad_percpu_size(); } pfo_ret__; });
}












typedef unsigned long pteval_t;
typedef unsigned long pmdval_t;
typedef unsigned long pudval_t;
typedef unsigned long pgdval_t;
typedef unsigned long pgprotval_t;

typedef struct { pteval_t pte; } pte_t;
typedef struct pgprot { pgprotval_t pgprot; } pgprot_t;

typedef struct { pgdval_t pgd; } pgd_t;

static inline __attribute__((no_instrument_function)) pgd_t native_make_pgd(pgdval_t val)
{
 return (pgd_t) { val };
}

static inline __attribute__((no_instrument_function)) pgdval_t native_pgd_val(pgd_t pgd)
{
 return pgd.pgd;
}

static inline __attribute__((no_instrument_function)) pgdval_t pgd_flags(pgd_t pgd)
{
 return native_pgd_val(pgd) & (~((pteval_t)(((signed long)(~(((1UL) << 12)-1))) & ((phys_addr_t)((1ULL << 46) - 1)))));
}


typedef struct { pudval_t pud; } pud_t;

static inline __attribute__((no_instrument_function)) pud_t native_make_pud(pmdval_t val)
{
 return (pud_t) { val };
}

static inline __attribute__((no_instrument_function)) pudval_t native_pud_val(pud_t pud)
{
 return pud.pud;
}
typedef struct { pmdval_t pmd; } pmd_t;

static inline __attribute__((no_instrument_function)) pmd_t native_make_pmd(pmdval_t val)
{
 return (pmd_t) { val };
}

static inline __attribute__((no_instrument_function)) pmdval_t native_pmd_val(pmd_t pmd)
{
 return pmd.pmd;
}
static inline __attribute__((no_instrument_function)) pudval_t pud_flags(pud_t pud)
{
 return native_pud_val(pud) & (~((pteval_t)(((signed long)(~(((1UL) << 12)-1))) & ((phys_addr_t)((1ULL << 46) - 1)))));
}

static inline __attribute__((no_instrument_function)) pmdval_t pmd_flags(pmd_t pmd)
{
 return native_pmd_val(pmd) & (~((pteval_t)(((signed long)(~(((1UL) << 12)-1))) & ((phys_addr_t)((1ULL << 46) - 1)))));
}

static inline __attribute__((no_instrument_function)) pte_t native_make_pte(pteval_t val)
{
 return (pte_t) { .pte = val };
}

static inline __attribute__((no_instrument_function)) pteval_t native_pte_val(pte_t pte)
{
 return pte.pte;
}

static inline __attribute__((no_instrument_function)) pteval_t pte_flags(pte_t pte)
{
 return native_pte_val(pte) & (~((pteval_t)(((signed long)(~(((1UL) << 12)-1))) & ((phys_addr_t)((1ULL << 46) - 1)))));
}





typedef struct page *pgtable_t;

extern pteval_t __supported_pte_mask;
extern void set_nx(void);
extern int nx_enabled;


extern pgprot_t pgprot_writecombine(pgprot_t prot);





struct file;
pgprot_t phys_mem_access_prot(struct file *file, unsigned long pfn,
                              unsigned long size, pgprot_t vma_prot);
int phys_mem_access_prot_allowed(struct file *file, unsigned long pfn,
                              unsigned long size, pgprot_t *vma_prot);


void set_pte_vaddr(unsigned long vaddr, pte_t pte);







struct seq_file;
extern void arch_report_meminfo(struct seq_file *m);

enum pg_level {
 PG_LEVEL_NONE,
 PG_LEVEL_4K,
 PG_LEVEL_2M,
 PG_LEVEL_1G,
 PG_LEVEL_NUM
};


extern void update_page_count(int level, unsigned long pages);
extern pte_t *lookup_address(unsigned long address, unsigned int *level);
extern pte_t *lookup_address_in_pgd(pgd_t *pgd, unsigned long address,
        unsigned int *level);
extern phys_addr_t slow_virt_to_phys(void *__address);
extern int kernel_map_pages_in_pgd(pgd_t *pgd, u64 pfn, unsigned long address,
       unsigned numpages, unsigned long page_flags);
void kernel_unmap_pages_in_pgd(pgd_t *root, unsigned long address,
          unsigned numpages);






















extern unsigned int __invalid_size_argument_for_IOC;







extern int __bitmap_empty(const unsigned long *bitmap, unsigned int nbits);
extern int __bitmap_full(const unsigned long *bitmap, unsigned int nbits);
extern int __bitmap_equal(const unsigned long *bitmap1,
     const unsigned long *bitmap2, unsigned int nbits);
extern void __bitmap_complement(unsigned long *dst, const unsigned long *src,
   unsigned int nbits);
extern void __bitmap_shift_right(unsigned long *dst,
                        const unsigned long *src, int shift, int bits);
extern void __bitmap_shift_left(unsigned long *dst,
                        const unsigned long *src, int shift, int bits);
extern int __bitmap_and(unsigned long *dst, const unsigned long *bitmap1,
   const unsigned long *bitmap2, unsigned int nbits);
extern void __bitmap_or(unsigned long *dst, const unsigned long *bitmap1,
   const unsigned long *bitmap2, unsigned int nbits);
extern void __bitmap_xor(unsigned long *dst, const unsigned long *bitmap1,
   const unsigned long *bitmap2, unsigned int nbits);
extern int __bitmap_andnot(unsigned long *dst, const unsigned long *bitmap1,
   const unsigned long *bitmap2, unsigned int nbits);
extern int __bitmap_intersects(const unsigned long *bitmap1,
   const unsigned long *bitmap2, unsigned int nbits);
extern int __bitmap_subset(const unsigned long *bitmap1,
   const unsigned long *bitmap2, unsigned int nbits);
extern int __bitmap_weight(const unsigned long *bitmap, unsigned int nbits);

extern void bitmap_set(unsigned long *map, unsigned int start, int len);
extern void bitmap_clear(unsigned long *map, unsigned int start, int len);
extern unsigned long bitmap_find_next_zero_area(unsigned long *map,
      unsigned long size,
      unsigned long start,
      unsigned int nr,
      unsigned long align_mask);

extern int bitmap_scnprintf(char *buf, unsigned int len,
   const unsigned long *src, int nbits);
extern int __bitmap_parse(const char *buf, unsigned int buflen, int is_user,
   unsigned long *dst, int nbits);
extern int bitmap_parse_user(const char *ubuf, unsigned int ulen,
   unsigned long *dst, int nbits);
extern int bitmap_scnlistprintf(char *buf, unsigned int len,
   const unsigned long *src, int nbits);
extern int bitmap_parselist(const char *buf, unsigned long *maskp,
   int nmaskbits);
extern int bitmap_parselist_user(const char *ubuf, unsigned int ulen,
   unsigned long *dst, int nbits);
extern void bitmap_remap(unsigned long *dst, const unsigned long *src,
  const unsigned long *old, const unsigned long *new, int bits);
extern int bitmap_bitremap(int oldbit,
  const unsigned long *old, const unsigned long *new, int bits);
extern void bitmap_onto(unsigned long *dst, const unsigned long *orig,
  const unsigned long *relmap, int bits);
extern void bitmap_fold(unsigned long *dst, const unsigned long *orig,
  int sz, int bits);
extern int bitmap_find_free_region(unsigned long *bitmap, unsigned int bits, int order);
extern void bitmap_release_region(unsigned long *bitmap, unsigned int pos, int order);
extern int bitmap_allocate_region(unsigned long *bitmap, unsigned int pos, int order);
extern void bitmap_copy_le(void *dst, const unsigned long *src, int nbits);
extern int bitmap_ord_to_pos(const unsigned long *bitmap, int n, int bits);
static inline __attribute__((no_instrument_function)) void bitmap_zero(unsigned long *dst, int nbits)
{
 if ((__builtin_constant_p(nbits) && (nbits) <= 64))
  *dst = 0UL;
 else {
  int len = (((nbits) + (8 * sizeof(long)) - 1) / (8 * sizeof(long))) * sizeof(unsigned long);
  memset(dst, 0, len);
 }
}

static inline __attribute__((no_instrument_function)) void bitmap_fill(unsigned long *dst, int nbits)
{
 size_t nlongs = (((nbits) + (8 * sizeof(long)) - 1) / (8 * sizeof(long)));
 if (!(__builtin_constant_p(nbits) && (nbits) <= 64)) {
  int len = (nlongs - 1) * sizeof(unsigned long);
  memset(dst, 0xff, len);
 }
 dst[nlongs - 1] = ( ((nbits) % 64) ? (1UL<<((nbits) % 64))-1 : ~0UL );
}

static inline __attribute__((no_instrument_function)) void bitmap_copy(unsigned long *dst, const unsigned long *src,
   int nbits)
{
 if ((__builtin_constant_p(nbits) && (nbits) <= 64))
  *dst = *src;
 else {
  int len = (((nbits) + (8 * sizeof(long)) - 1) / (8 * sizeof(long))) * sizeof(unsigned long);
  memcpy(dst, src, len);
 }
}

static inline __attribute__((no_instrument_function)) int bitmap_and(unsigned long *dst, const unsigned long *src1,
   const unsigned long *src2, unsigned int nbits)
{
 if ((__builtin_constant_p(nbits) && (nbits) <= 64))
  return (*dst = *src1 & *src2 & ( ((nbits) % 64) ? (1UL<<((nbits) % 64))-1 : ~0UL )) != 0;
 return __bitmap_and(dst, src1, src2, nbits);
}

static inline __attribute__((no_instrument_function)) void bitmap_or(unsigned long *dst, const unsigned long *src1,
   const unsigned long *src2, unsigned int nbits)
{
 if ((__builtin_constant_p(nbits) && (nbits) <= 64))
  *dst = *src1 | *src2;
 else
  __bitmap_or(dst, src1, src2, nbits);
}

static inline __attribute__((no_instrument_function)) void bitmap_xor(unsigned long *dst, const unsigned long *src1,
   const unsigned long *src2, unsigned int nbits)
{
 if ((__builtin_constant_p(nbits) && (nbits) <= 64))
  *dst = *src1 ^ *src2;
 else
  __bitmap_xor(dst, src1, src2, nbits);
}

static inline __attribute__((no_instrument_function)) int bitmap_andnot(unsigned long *dst, const unsigned long *src1,
   const unsigned long *src2, unsigned int nbits)
{
 if ((__builtin_constant_p(nbits) && (nbits) <= 64))
  return (*dst = *src1 & ~(*src2) & ( ((nbits) % 64) ? (1UL<<((nbits) % 64))-1 : ~0UL )) != 0;
 return __bitmap_andnot(dst, src1, src2, nbits);
}

static inline __attribute__((no_instrument_function)) void bitmap_complement(unsigned long *dst, const unsigned long *src,
   unsigned int nbits)
{
 if ((__builtin_constant_p(nbits) && (nbits) <= 64))
  *dst = ~(*src);
 else
  __bitmap_complement(dst, src, nbits);
}

static inline __attribute__((no_instrument_function)) int bitmap_equal(const unsigned long *src1,
   const unsigned long *src2, unsigned int nbits)
{
 if ((__builtin_constant_p(nbits) && (nbits) <= 64))
  return ! ((*src1 ^ *src2) & ( ((nbits) % 64) ? (1UL<<((nbits) % 64))-1 : ~0UL ));
 else
  return __bitmap_equal(src1, src2, nbits);
}

static inline __attribute__((no_instrument_function)) int bitmap_intersects(const unsigned long *src1,
   const unsigned long *src2, unsigned int nbits)
{
 if ((__builtin_constant_p(nbits) && (nbits) <= 64))
  return ((*src1 & *src2) & ( ((nbits) % 64) ? (1UL<<((nbits) % 64))-1 : ~0UL )) != 0;
 else
  return __bitmap_intersects(src1, src2, nbits);
}

static inline __attribute__((no_instrument_function)) int bitmap_subset(const unsigned long *src1,
   const unsigned long *src2, unsigned int nbits)
{
 if ((__builtin_constant_p(nbits) && (nbits) <= 64))
  return ! ((*src1 & ~(*src2)) & ( ((nbits) % 64) ? (1UL<<((nbits) % 64))-1 : ~0UL ));
 else
  return __bitmap_subset(src1, src2, nbits);
}

static inline __attribute__((no_instrument_function)) int bitmap_empty(const unsigned long *src, unsigned nbits)
{
 if ((__builtin_constant_p(nbits) && (nbits) <= 64))
  return ! (*src & ( ((nbits) % 64) ? (1UL<<((nbits) % 64))-1 : ~0UL ));
 else
  return __bitmap_empty(src, nbits);
}

static inline __attribute__((no_instrument_function)) int bitmap_full(const unsigned long *src, unsigned int nbits)
{
 if ((__builtin_constant_p(nbits) && (nbits) <= 64))
  return ! (~(*src) & ( ((nbits) % 64) ? (1UL<<((nbits) % 64))-1 : ~0UL ));
 else
  return __bitmap_full(src, nbits);
}

static inline __attribute__((no_instrument_function)) int bitmap_weight(const unsigned long *src, unsigned int nbits)
{
 if ((__builtin_constant_p(nbits) && (nbits) <= 64))
  return hweight_long(*src & ( ((nbits) % 64) ? (1UL<<((nbits) % 64))-1 : ~0UL ));
 return __bitmap_weight(src, nbits);
}

static inline __attribute__((no_instrument_function)) void bitmap_shift_right(unsigned long *dst,
   const unsigned long *src, int n, int nbits)
{
 if ((__builtin_constant_p(nbits) && (nbits) <= 64))
  *dst = (*src & ( ((nbits) % 64) ? (1UL<<((nbits) % 64))-1 : ~0UL )) >> n;
 else
  __bitmap_shift_right(dst, src, n, nbits);
}

static inline __attribute__((no_instrument_function)) void bitmap_shift_left(unsigned long *dst,
   const unsigned long *src, int n, int nbits)
{
 if ((__builtin_constant_p(nbits) && (nbits) <= 64))
  *dst = (*src << n) & ( ((nbits) % 64) ? (1UL<<((nbits) % 64))-1 : ~0UL );
 else
  __bitmap_shift_left(dst, src, n, nbits);
}

static inline __attribute__((no_instrument_function)) int bitmap_parse(const char *buf, unsigned int buflen,
   unsigned long *maskp, int nmaskbits)
{
 return __bitmap_parse(buf, buflen, 0, maskp, nmaskbits);
}


typedef struct cpumask { unsigned long bits[(((256) + (8 * sizeof(long)) - 1) / (8 * sizeof(long)))]; } cpumask_t;
extern int nr_cpu_ids;
extern const struct cpumask *const cpu_possible_mask;
extern const struct cpumask *const cpu_online_mask;
extern const struct cpumask *const cpu_present_mask;
extern const struct cpumask *const cpu_active_mask;
static inline __attribute__((no_instrument_function)) unsigned int cpumask_check(unsigned int cpu)
{



 return cpu;
}
static inline __attribute__((no_instrument_function)) unsigned int cpumask_first(const struct cpumask *srcp)
{
 return find_first_bit(((srcp)->bits), 256);
}
static inline __attribute__((no_instrument_function)) unsigned int cpumask_next(int n, const struct cpumask *srcp)
{

 if (n != -1)
  cpumask_check(n);
 return find_next_bit(((srcp)->bits), 256, n+1);
}
static inline __attribute__((no_instrument_function)) unsigned int cpumask_next_zero(int n, const struct cpumask *srcp)
{

 if (n != -1)
  cpumask_check(n);
 return find_next_zero_bit(((srcp)->bits), 256, n+1);
}

int cpumask_next_and(int n, const struct cpumask *, const struct cpumask *);
int cpumask_any_but(const struct cpumask *mask, unsigned int cpu);
int cpumask_set_cpu_local_first(int i, int numa_node, cpumask_t *dstp);
static inline __attribute__((no_instrument_function)) void cpumask_set_cpu(unsigned int cpu, struct cpumask *dstp)
{
 set_bit(cpumask_check(cpu), ((dstp)->bits));
}






static inline __attribute__((no_instrument_function)) void cpumask_clear_cpu(int cpu, struct cpumask *dstp)
{
 clear_bit(cpumask_check(cpu), ((dstp)->bits));
}
static inline __attribute__((no_instrument_function)) int cpumask_test_and_set_cpu(int cpu, struct cpumask *cpumask)
{
 return test_and_set_bit(cpumask_check(cpu), ((cpumask)->bits));
}
static inline __attribute__((no_instrument_function)) int cpumask_test_and_clear_cpu(int cpu, struct cpumask *cpumask)
{
 return test_and_clear_bit(cpumask_check(cpu), ((cpumask)->bits));
}





static inline __attribute__((no_instrument_function)) void cpumask_setall(struct cpumask *dstp)
{
 bitmap_fill(((dstp)->bits), 256);
}





static inline __attribute__((no_instrument_function)) void cpumask_clear(struct cpumask *dstp)
{
 bitmap_zero(((dstp)->bits), 256);
}
static inline __attribute__((no_instrument_function)) int cpumask_and(struct cpumask *dstp,
          const struct cpumask *src1p,
          const struct cpumask *src2p)
{
 return bitmap_and(((dstp)->bits), ((src1p)->bits),
           ((src2p)->bits), 256);
}







static inline __attribute__((no_instrument_function)) void cpumask_or(struct cpumask *dstp, const struct cpumask *src1p,
         const struct cpumask *src2p)
{
 bitmap_or(((dstp)->bits), ((src1p)->bits),
          ((src2p)->bits), 256);
}







static inline __attribute__((no_instrument_function)) void cpumask_xor(struct cpumask *dstp,
          const struct cpumask *src1p,
          const struct cpumask *src2p)
{
 bitmap_xor(((dstp)->bits), ((src1p)->bits),
           ((src2p)->bits), 256);
}
static inline __attribute__((no_instrument_function)) int cpumask_andnot(struct cpumask *dstp,
      const struct cpumask *src1p,
      const struct cpumask *src2p)
{
 return bitmap_andnot(((dstp)->bits), ((src1p)->bits),
       ((src2p)->bits), 256);
}






static inline __attribute__((no_instrument_function)) void cpumask_complement(struct cpumask *dstp,
          const struct cpumask *srcp)
{
 bitmap_complement(((dstp)->bits), ((srcp)->bits),
           256);
}






static inline __attribute__((no_instrument_function)) bool cpumask_equal(const struct cpumask *src1p,
    const struct cpumask *src2p)
{
 return bitmap_equal(((src1p)->bits), ((src2p)->bits),
       256);
}






static inline __attribute__((no_instrument_function)) bool cpumask_intersects(const struct cpumask *src1p,
         const struct cpumask *src2p)
{
 return bitmap_intersects(((src1p)->bits), ((src2p)->bits),
            256);
}
static inline __attribute__((no_instrument_function)) int cpumask_subset(const struct cpumask *src1p,
     const struct cpumask *src2p)
{
 return bitmap_subset(((src1p)->bits), ((src2p)->bits),
        256);
}





static inline __attribute__((no_instrument_function)) bool cpumask_empty(const struct cpumask *srcp)
{
 return bitmap_empty(((srcp)->bits), 256);
}





static inline __attribute__((no_instrument_function)) bool cpumask_full(const struct cpumask *srcp)
{
 return bitmap_full(((srcp)->bits), 256);
}





static inline __attribute__((no_instrument_function)) unsigned int cpumask_weight(const struct cpumask *srcp)
{
 return bitmap_weight(((srcp)->bits), 256);
}







static inline __attribute__((no_instrument_function)) void cpumask_shift_right(struct cpumask *dstp,
           const struct cpumask *srcp, int n)
{
 bitmap_shift_right(((dstp)->bits), ((srcp)->bits), n,
            256);
}







static inline __attribute__((no_instrument_function)) void cpumask_shift_left(struct cpumask *dstp,
          const struct cpumask *srcp, int n)
{
 bitmap_shift_left(((dstp)->bits), ((srcp)->bits), n,
           256);
}






static inline __attribute__((no_instrument_function)) void cpumask_copy(struct cpumask *dstp,
    const struct cpumask *srcp)
{
 bitmap_copy(((dstp)->bits), ((srcp)->bits), 256);
}
static inline __attribute__((no_instrument_function)) int cpumask_scnprintf(char *buf, int len,
        const struct cpumask *srcp)
{
 return bitmap_scnprintf(buf, len, ((srcp)->bits), 256);
}
static inline __attribute__((no_instrument_function)) int cpumask_parse_user(const char *buf, int len,
         struct cpumask *dstp)
{
 return bitmap_parse_user(buf, len, ((dstp)->bits), 256);
}
static inline __attribute__((no_instrument_function)) int cpumask_parselist_user(const char *buf, int len,
         struct cpumask *dstp)
{
 return bitmap_parselist_user(buf, len, ((dstp)->bits),
       256);
}
static inline __attribute__((no_instrument_function)) int cpulist_scnprintf(char *buf, int len,
        const struct cpumask *srcp)
{
 return bitmap_scnlistprintf(buf, len, ((srcp)->bits),
        256);
}
static inline __attribute__((no_instrument_function)) int cpumask_parse(const char *buf, struct cpumask *dstp)
{
 char *nl = strchr(buf, '\n');
 unsigned int len = nl ? (unsigned int)(nl - buf) : strlen(buf);

 return bitmap_parse(buf, len, ((dstp)->bits), 256);
}
static inline __attribute__((no_instrument_function)) int cpulist_parse(const char *buf, struct cpumask *dstp)
{
 return bitmap_parselist(buf, ((dstp)->bits), 256);
}






static inline __attribute__((no_instrument_function)) size_t cpumask_size(void)
{


 return (((256) + (8 * sizeof(long)) - 1) / (8 * sizeof(long))) * sizeof(long);
}
typedef struct cpumask cpumask_var_t[1];

static inline __attribute__((no_instrument_function)) bool alloc_cpumask_var(cpumask_var_t *mask, gfp_t flags)
{
 return true;
}

static inline __attribute__((no_instrument_function)) bool alloc_cpumask_var_node(cpumask_var_t *mask, gfp_t flags,
       int node)
{
 return true;
}

static inline __attribute__((no_instrument_function)) bool zalloc_cpumask_var(cpumask_var_t *mask, gfp_t flags)
{
 cpumask_clear(*mask);
 return true;
}

static inline __attribute__((no_instrument_function)) bool zalloc_cpumask_var_node(cpumask_var_t *mask, gfp_t flags,
       int node)
{
 cpumask_clear(*mask);
 return true;
}

static inline __attribute__((no_instrument_function)) void alloc_bootmem_cpumask_var(cpumask_var_t *mask)
{
}

static inline __attribute__((no_instrument_function)) void free_cpumask_var(cpumask_var_t mask)
{
}

static inline __attribute__((no_instrument_function)) void free_bootmem_cpumask_var(cpumask_var_t mask)
{
}




extern const unsigned long cpu_all_bits[(((256) + (8 * sizeof(long)) - 1) / (8 * sizeof(long)))];
void set_cpu_possible(unsigned int cpu, bool possible);
void set_cpu_present(unsigned int cpu, bool present);
void set_cpu_online(unsigned int cpu, bool online);
void set_cpu_active(unsigned int cpu, bool active);
void init_cpu_present(const struct cpumask *src);
void init_cpu_possible(const struct cpumask *src);
void init_cpu_online(const struct cpumask *src);
static inline __attribute__((no_instrument_function)) int __check_is_bitmap(const unsigned long *bitmap)
{
 return 1;
}
extern const unsigned long
 cpu_bit_bitmap[64 +1][(((256) + (8 * sizeof(long)) - 1) / (8 * sizeof(long)))];

static inline __attribute__((no_instrument_function)) const struct cpumask *get_cpu_mask(unsigned int cpu)
{
 const unsigned long *p = cpu_bit_bitmap[1 + cpu % 64];
 p -= cpu / 64;
 return ((struct cpumask *)(1 ? (p) : (void *)sizeof(__check_is_bitmap(p))));
}
int __first_cpu(const cpumask_t *srcp);
int __next_cpu(int n, const cpumask_t *srcp);
int __next_cpu_nr(int n, const cpumask_t *srcp);
static inline __attribute__((no_instrument_function)) void __cpu_set(int cpu, volatile cpumask_t *dstp)
{
 set_bit(cpu, dstp->bits);
}


static inline __attribute__((no_instrument_function)) void __cpu_clear(int cpu, volatile cpumask_t *dstp)
{
 clear_bit(cpu, dstp->bits);
}


static inline __attribute__((no_instrument_function)) void __cpus_setall(cpumask_t *dstp, int nbits)
{
 bitmap_fill(dstp->bits, nbits);
}


static inline __attribute__((no_instrument_function)) void __cpus_clear(cpumask_t *dstp, int nbits)
{
 bitmap_zero(dstp->bits, nbits);
}





static inline __attribute__((no_instrument_function)) int __cpu_test_and_set(int cpu, cpumask_t *addr)
{
 return test_and_set_bit(cpu, addr->bits);
}


static inline __attribute__((no_instrument_function)) int __cpus_and(cpumask_t *dstp, const cpumask_t *src1p,
     const cpumask_t *src2p, int nbits)
{
 return bitmap_and(dstp->bits, src1p->bits, src2p->bits, nbits);
}


static inline __attribute__((no_instrument_function)) void __cpus_or(cpumask_t *dstp, const cpumask_t *src1p,
     const cpumask_t *src2p, int nbits)
{
 bitmap_or(dstp->bits, src1p->bits, src2p->bits, nbits);
}


static inline __attribute__((no_instrument_function)) void __cpus_xor(cpumask_t *dstp, const cpumask_t *src1p,
     const cpumask_t *src2p, int nbits)
{
 bitmap_xor(dstp->bits, src1p->bits, src2p->bits, nbits);
}



static inline __attribute__((no_instrument_function)) int __cpus_andnot(cpumask_t *dstp, const cpumask_t *src1p,
     const cpumask_t *src2p, int nbits)
{
 return bitmap_andnot(dstp->bits, src1p->bits, src2p->bits, nbits);
}


static inline __attribute__((no_instrument_function)) int __cpus_equal(const cpumask_t *src1p,
     const cpumask_t *src2p, int nbits)
{
 return bitmap_equal(src1p->bits, src2p->bits, nbits);
}


static inline __attribute__((no_instrument_function)) int __cpus_intersects(const cpumask_t *src1p,
     const cpumask_t *src2p, int nbits)
{
 return bitmap_intersects(src1p->bits, src2p->bits, nbits);
}


static inline __attribute__((no_instrument_function)) int __cpus_subset(const cpumask_t *src1p,
     const cpumask_t *src2p, int nbits)
{
 return bitmap_subset(src1p->bits, src2p->bits, nbits);
}


static inline __attribute__((no_instrument_function)) int __cpus_empty(const cpumask_t *srcp, int nbits)
{
 return bitmap_empty(srcp->bits, nbits);
}


static inline __attribute__((no_instrument_function)) int __cpus_weight(const cpumask_t *srcp, int nbits)
{
 return bitmap_weight(srcp->bits, nbits);
}



static inline __attribute__((no_instrument_function)) void __cpus_shift_left(cpumask_t *dstp,
     const cpumask_t *srcp, int n, int nbits)
{
 bitmap_shift_left(dstp->bits, srcp->bits, n, nbits);
}

extern cpumask_var_t cpu_callin_mask;
extern cpumask_var_t cpu_callout_mask;
extern cpumask_var_t cpu_initialized_mask;
extern cpumask_var_t cpu_sibling_setup_mask;

extern void setup_cpu_local_masks(void);

struct msr {
 union {
  struct {
   u32 l;
   u32 h;
  };
  u64 q;
 };
};

struct msr_info {
 u32 msr_no;
 struct msr reg;
 struct msr *msrs;
 int err;
};

struct msr_regs_info {
 u32 *regs;
 int err;
};

static inline __attribute__((no_instrument_function)) unsigned long long native_read_tscp(unsigned int *aux)
{
 unsigned long low, high;
 asm volatile(".byte 0x0f,0x01,0xf9"
       : "=a" (low), "=d" (high), "=c" (*aux));
 return low | ((u64)high << 32);
}
static inline __attribute__((no_instrument_function)) unsigned long long native_read_msr(unsigned int msr)
{
 unsigned low, high;

 asm volatile("rdmsr" : "=a" (low), "=d" (high) : "c" (msr));
 return ((low) | ((u64)(high) << 32));
}

static inline __attribute__((no_instrument_function)) unsigned long long native_read_msr_safe(unsigned int msr,
            int *err)
{
 unsigned low, high;

 asm volatile("2: rdmsr ; xor %[err],%[err]\n"
       "1:\n\t"
       ".section .fixup,\"ax\"\n\t"
       "3:  mov %[fault],%[err] ; jmp 1b\n\t"
       ".previous\n\t"
       " .pushsection \"__ex_table\",\"a\"\n" " .balign 8\n" " .long (" "2b" ") - .\n" " .long (" "3b" ") - .\n" " .popsection\n"
       : [err] "=r" (*err), "=a" (low), "=d" (high)
       : "c" (msr), [fault] "i" (-5));
 return ((low) | ((u64)(high) << 32));
}

static inline __attribute__((no_instrument_function)) void native_write_msr(unsigned int msr,
        unsigned low, unsigned high)
{
 asm volatile("wrmsr" : : "c" (msr), "a"(low), "d" (high) : "memory");
}


__attribute__((no_instrument_function)) static inline __attribute__((no_instrument_function)) int native_write_msr_safe(unsigned int msr,
     unsigned low, unsigned high)
{
 int err;
 asm volatile("2: wrmsr ; xor %[err],%[err]\n"
       "1:\n\t"
       ".section .fixup,\"ax\"\n\t"
       "3:  mov %[fault],%[err] ; jmp 1b\n\t"
       ".previous\n\t"
       " .pushsection \"__ex_table\",\"a\"\n" " .balign 8\n" " .long (" "2b" ") - .\n" " .long (" "3b" ") - .\n" " .popsection\n"
       : [err] "=a" (err)
       : "c" (msr), "0" (low), "d" (high),
         [fault] "i" (-5)
       : "memory");
 return err;
}

extern unsigned long long native_read_tsc(void);

extern int rdmsr_safe_regs(u32 regs[8]);
extern int wrmsr_safe_regs(u32 regs[8]);

static inline __attribute__((no_instrument_function)) __attribute__((always_inline)) unsigned long long __native_read_tsc(void)
{
 unsigned low, high;

 asm volatile("rdtsc" : "=a" (low), "=d" (high));

 return ((low) | ((u64)(high) << 32));
}

static inline __attribute__((no_instrument_function)) unsigned long long native_read_pmc(int counter)
{
 unsigned low, high;

 asm volatile("rdpmc" : "=a" (low), "=d" (high) : "c" (counter));
 return ((low) | ((u64)(high) << 32));
}
static inline __attribute__((no_instrument_function)) void wrmsr(unsigned msr, unsigned low, unsigned high)
{
 native_write_msr(msr, low, high);
}
static inline __attribute__((no_instrument_function)) int wrmsr_safe(unsigned msr, unsigned low, unsigned high)
{
 return native_write_msr_safe(msr, low, high);
}
static inline __attribute__((no_instrument_function)) int rdmsrl_safe(unsigned msr, unsigned long long *p)
{
 int err;

 *p = native_read_msr_safe(msr, &err);
 return err;
}
struct msr *msrs_alloc(void);
void msrs_free(struct msr *msrs);
int msr_set_bit(u32 msr, u8 bit);
int msr_clear_bit(u32 msr, u8 bit);


int rdmsr_on_cpu(unsigned int cpu, u32 msr_no, u32 *l, u32 *h);
int wrmsr_on_cpu(unsigned int cpu, u32 msr_no, u32 l, u32 h);
int rdmsrl_on_cpu(unsigned int cpu, u32 msr_no, u64 *q);
int wrmsrl_on_cpu(unsigned int cpu, u32 msr_no, u64 q);
void rdmsr_on_cpus(const struct cpumask *mask, u32 msr_no, struct msr *msrs);
void wrmsr_on_cpus(const struct cpumask *mask, u32 msr_no, struct msr *msrs);
int rdmsr_safe_on_cpu(unsigned int cpu, u32 msr_no, u32 *l, u32 *h);
int wrmsr_safe_on_cpu(unsigned int cpu, u32 msr_no, u32 l, u32 h);
int rdmsrl_safe_on_cpu(unsigned int cpu, u32 msr_no, u64 *q);
int wrmsrl_safe_on_cpu(unsigned int cpu, u32 msr_no, u64 q);
int rdmsr_safe_regs_on_cpu(unsigned int cpu, u32 regs[8]);
int wrmsr_safe_regs_on_cpu(unsigned int cpu, u32 regs[8]);
struct desc_struct {
 union {
  struct {
   unsigned int a;
   unsigned int b;
  };
  struct {
   u16 limit0;
   u16 base0;
   unsigned base1: 8, type: 4, s: 1, dpl: 2, p: 1;
   unsigned limit: 4, avl: 1, l: 1, d: 1, g: 1, base2: 8;
  };
 };
} __attribute__((packed));







enum {
 GATE_INTERRUPT = 0xE,
 GATE_TRAP = 0xF,
 GATE_CALL = 0xC,
 GATE_TASK = 0x5,
};


struct gate_struct64 {
 u16 offset_low;
 u16 segment;
 unsigned ist : 3, zero0 : 5, type : 5, dpl : 2, p : 1;
 u16 offset_middle;
 u32 offset_high;
 u32 zero1;
} __attribute__((packed));





enum {
 DESC_TSS = 0x9,
 DESC_LDT = 0x2,
 DESCTYPE_S = 0x10,
};


struct ldttss_desc64 {
 u16 limit0;
 u16 base0;
 unsigned base1 : 8, type : 5, dpl : 2, p : 1;
 unsigned limit1 : 4, zero0 : 3, g : 1, base2 : 8;
 u32 base3;
 u32 zero1;
} __attribute__((packed));


typedef struct gate_struct64 gate_desc;
typedef struct ldttss_desc64 ldt_desc;
typedef struct ldttss_desc64 tss_desc;
struct desc_ptr {
 unsigned short size;
 unsigned long address;
} __attribute__((packed)) ;







static inline __attribute__((no_instrument_function)) void native_clts(void)
{
 asm volatile("clts");
}
extern unsigned long __force_order;

static inline __attribute__((no_instrument_function)) unsigned long native_read_cr0(void)
{
 unsigned long val;
 asm volatile("mov %%cr0,%0\n\t" : "=r" (val), "=m" (__force_order));
 return val;
}

static inline __attribute__((no_instrument_function)) void native_write_cr0(unsigned long val)
{
 asm volatile("mov %0,%%cr0": : "r" (val), "m" (__force_order));
}

static inline __attribute__((no_instrument_function)) unsigned long native_read_cr2(void)
{
 unsigned long val;
 asm volatile("mov %%cr2,%0\n\t" : "=r" (val), "=m" (__force_order));
 return val;
}

static inline __attribute__((no_instrument_function)) void native_write_cr2(unsigned long val)
{
 asm volatile("mov %0,%%cr2": : "r" (val), "m" (__force_order));
}

static inline __attribute__((no_instrument_function)) unsigned long native_read_cr3(void)
{
 unsigned long val;
 asm volatile("mov %%cr3,%0\n\t" : "=r" (val), "=m" (__force_order));
 return val;
}

static inline __attribute__((no_instrument_function)) void native_write_cr3(unsigned long val)
{
 asm volatile("mov %0,%%cr3": : "r" (val), "m" (__force_order));
}

static inline __attribute__((no_instrument_function)) unsigned long native_read_cr4(void)
{
 unsigned long val;
 asm volatile("mov %%cr4,%0\n\t" : "=r" (val), "=m" (__force_order));
 return val;
}

static inline __attribute__((no_instrument_function)) unsigned long native_read_cr4_safe(void)
{
 unsigned long val;
 val = native_read_cr4();

 return val;
}

static inline __attribute__((no_instrument_function)) void native_write_cr4(unsigned long val)
{
 asm volatile("mov %0,%%cr4": : "r" (val), "m" (__force_order));
}


static inline __attribute__((no_instrument_function)) unsigned long native_read_cr8(void)
{
 unsigned long cr8;
 asm volatile("movq %%cr8,%0" : "=r" (cr8));
 return cr8;
}

static inline __attribute__((no_instrument_function)) void native_write_cr8(unsigned long val)
{
 asm volatile("movq %0,%%cr8" :: "r" (val) : "memory");
}


static inline __attribute__((no_instrument_function)) void native_wbinvd(void)
{
 asm volatile("wbinvd": : :"memory");
}

extern void native_load_gs_index(unsigned);





static inline __attribute__((no_instrument_function)) unsigned long read_cr0(void)
{
 return native_read_cr0();
}

static inline __attribute__((no_instrument_function)) void write_cr0(unsigned long x)
{
 native_write_cr0(x);
}

static inline __attribute__((no_instrument_function)) unsigned long read_cr2(void)
{
 return native_read_cr2();
}

static inline __attribute__((no_instrument_function)) void write_cr2(unsigned long x)
{
 native_write_cr2(x);
}

static inline __attribute__((no_instrument_function)) unsigned long read_cr3(void)
{
 return native_read_cr3();
}

static inline __attribute__((no_instrument_function)) void write_cr3(unsigned long x)
{
 native_write_cr3(x);
}

static inline __attribute__((no_instrument_function)) unsigned long read_cr4(void)
{
 return native_read_cr4();
}

static inline __attribute__((no_instrument_function)) unsigned long read_cr4_safe(void)
{
 return native_read_cr4_safe();
}

static inline __attribute__((no_instrument_function)) void write_cr4(unsigned long x)
{
 native_write_cr4(x);
}

static inline __attribute__((no_instrument_function)) void wbinvd(void)
{
 native_wbinvd();
}



static inline __attribute__((no_instrument_function)) unsigned long read_cr8(void)
{
 return native_read_cr8();
}

static inline __attribute__((no_instrument_function)) void write_cr8(unsigned long x)
{
 native_write_cr8(x);
}

static inline __attribute__((no_instrument_function)) void load_gs_index(unsigned selector)
{
 native_load_gs_index(selector);
}




static inline __attribute__((no_instrument_function)) void clts(void)
{
 native_clts();
}





static inline __attribute__((no_instrument_function)) void clflush(volatile void *__p)
{
 asm volatile("clflush %0" : "+m" (*(volatile char *)__p));
}

static inline __attribute__((no_instrument_function)) void clflushopt(volatile void *__p)
{
 asm volatile ("661:\n\t" ".byte " "0x3e" "; clflush %P0" "\n662:\n" ".pushsection .altinstructions,\"a\"\n" " .long 661b - .\n" " .long " "663""1""f - .\n" " .word " "( 9*32+23)" "\n" " .byte " "662b-661b" "\n" " .byte " "664""1""f-""663""1""f" "\n" ".popsection\n" ".pushsection .discard,\"aw\",@progbits\n" " .byte 0xff + (" "664""1""f-""663""1""f" ") - (" "662b-661b" ")\n" ".popsection\n" ".pushsection .altinstr_replacement, \"ax\"\n" "663""1"":\n\t" ".byte 0x66; clflush %P0" "\n" "664""1" ":\n\t" ".popsection" : "+m" (*(volatile char *)__p) : "i" (0))


                                              ;
}




enum {
 UNAME26 = 0x0020000,
 ADDR_NO_RANDOMIZE = 0x0040000,
 FDPIC_FUNCPTRS = 0x0080000,


 MMAP_PAGE_ZERO = 0x0100000,
 ADDR_COMPAT_LAYOUT = 0x0200000,
 READ_IMPLIES_EXEC = 0x0400000,
 ADDR_LIMIT_32BIT = 0x0800000,
 SHORT_INODE = 0x1000000,
 WHOLE_SECONDS = 0x2000000,
 STICKY_TIMEOUTS = 0x4000000,
 ADDR_LIMIT_3GB = 0x8000000,
};
enum {
 PER_LINUX = 0x0000,
 PER_LINUX_32BIT = 0x0000 | ADDR_LIMIT_32BIT,
 PER_LINUX_FDPIC = 0x0000 | FDPIC_FUNCPTRS,
 PER_SVR4 = 0x0001 | STICKY_TIMEOUTS | MMAP_PAGE_ZERO,
 PER_SVR3 = 0x0002 | STICKY_TIMEOUTS | SHORT_INODE,
 PER_SCOSVR3 = 0x0003 | STICKY_TIMEOUTS |
      WHOLE_SECONDS | SHORT_INODE,
 PER_OSR5 = 0x0003 | STICKY_TIMEOUTS | WHOLE_SECONDS,
 PER_WYSEV386 = 0x0004 | STICKY_TIMEOUTS | SHORT_INODE,
 PER_ISCR4 = 0x0005 | STICKY_TIMEOUTS,
 PER_BSD = 0x0006,
 PER_SUNOS = 0x0006 | STICKY_TIMEOUTS,
 PER_XENIX = 0x0007 | STICKY_TIMEOUTS | SHORT_INODE,
 PER_LINUX32 = 0x0008,
 PER_LINUX32_3GB = 0x0008 | ADDR_LIMIT_3GB,
 PER_IRIX32 = 0x0009 | STICKY_TIMEOUTS,
 PER_IRIXN32 = 0x000a | STICKY_TIMEOUTS,
 PER_IRIX64 = 0x000b | STICKY_TIMEOUTS,
 PER_RISCOS = 0x000c,
 PER_SOLARIS = 0x000d | STICKY_TIMEOUTS,
 PER_UW7 = 0x000e | STICKY_TIMEOUTS | MMAP_PAGE_ZERO,
 PER_OSF4 = 0x000f,
 PER_HPUX = 0x0010,
 PER_MASK = 0x00ff,
};






struct exec_domain;
struct pt_regs;

extern int register_exec_domain(struct exec_domain *);
extern int unregister_exec_domain(struct exec_domain *);
extern int __set_personality(unsigned int);
typedef void (*handler_t)(int, struct pt_regs *);

struct exec_domain {
 const char *name;
 handler_t handler;
 unsigned char pers_low;
 unsigned char pers_high;
 unsigned long *signal_map;
 unsigned long *signal_invmap;
 struct map_segment *err_map;
 struct map_segment *socktype_map;
 struct map_segment *sockopt_map;
 struct map_segment *af_map;
 struct module *module;
 struct exec_domain *next;
};







static inline __attribute__((no_instrument_function)) u64 div_u64_rem(u64 dividend, u32 divisor, u32 *remainder)
{
 *remainder = dividend % divisor;
 return dividend / divisor;
}




static inline __attribute__((no_instrument_function)) s64 div_s64_rem(s64 dividend, s32 divisor, s32 *remainder)
{
 *remainder = dividend % divisor;
 return dividend / divisor;
}




static inline __attribute__((no_instrument_function)) u64 div64_u64_rem(u64 dividend, u64 divisor, u64 *remainder)
{
 *remainder = dividend % divisor;
 return dividend / divisor;
}




static inline __attribute__((no_instrument_function)) u64 div64_u64(u64 dividend, u64 divisor)
{
 return dividend / divisor;
}




static inline __attribute__((no_instrument_function)) s64 div64_s64(s64 dividend, s64 divisor)
{
 return dividend / divisor;
}
static inline __attribute__((no_instrument_function)) u64 div_u64(u64 dividend, u32 divisor)
{
 u32 remainder;
 return div_u64_rem(dividend, divisor, &remainder);
}






static inline __attribute__((no_instrument_function)) s64 div_s64(s64 dividend, s32 divisor)
{
 s32 remainder;
 return div_s64_rem(dividend, divisor, &remainder);
}


u32 iter_div_u64_rem(u64 dividend, u32 divisor, u64 *remainder);

static inline __attribute__((no_instrument_function)) __attribute__((always_inline)) u32
__iter_div_u64_rem(u64 dividend, u32 divisor, u64 *remainder)
{
 u32 ret = 0;

 while (dividend >= divisor) {


  asm("" : "+rm"(dividend));

  dividend -= divisor;
  ret++;
 }

 *remainder = dividend;

 return ret;
}




static inline __attribute__((no_instrument_function)) u64 mul_u64_u32_shr(u64 a, u32 mul, unsigned int shift)
{
 return (u64)(((unsigned __int128)a * mul) >> shift);
}






static inline __attribute__((no_instrument_function)) void * ERR_PTR(long error)
{
 return (void *) error;
}

static inline __attribute__((no_instrument_function)) long PTR_ERR( const void *ptr)
{
 return (long) ptr;
}

static inline __attribute__((no_instrument_function)) bool IS_ERR( const void *ptr)
{
 return __builtin_expect(!!(((unsigned long)ptr) >= (unsigned long)-4095), 0);
}

static inline __attribute__((no_instrument_function)) bool IS_ERR_OR_NULL( const void *ptr)
{
 return !ptr || __builtin_expect(!!(((unsigned long)ptr) >= (unsigned long)-4095), 0);
}
static inline __attribute__((no_instrument_function)) void * ERR_CAST( const void *ptr)
{

 return (void *) ptr;
}

static inline __attribute__((no_instrument_function)) int PTR_ERR_OR_ZERO( const void *ptr)
{
 if (IS_ERR(ptr))
  return PTR_ERR(ptr);
 else
  return 0;
}
static inline __attribute__((no_instrument_function)) unsigned long native_save_fl(void)
{
 unsigned long flags;






 asm volatile("# __raw_save_flags\n\t"
       "pushf ; pop %0"
       : "=rm" (flags)
       :
       : "memory");

 return flags;
}

static inline __attribute__((no_instrument_function)) void native_restore_fl(unsigned long flags)
{
 asm volatile("push %0 ; popf"
       :
       :"g" (flags)
       :"memory", "cc");
}

static inline __attribute__((no_instrument_function)) void native_irq_disable(void)
{
 asm volatile("cli": : :"memory");
}

static inline __attribute__((no_instrument_function)) void native_irq_enable(void)
{
 asm volatile("sti": : :"memory");
}

static inline __attribute__((no_instrument_function)) void native_safe_halt(void)
{
 asm volatile("sti; hlt": : :"memory");
}

static inline __attribute__((no_instrument_function)) void native_halt(void)
{
 asm volatile("hlt": : :"memory");
}
static inline __attribute__((no_instrument_function)) __attribute__((no_instrument_function)) unsigned long arch_local_save_flags(void)
{
 return native_save_fl();
}

static inline __attribute__((no_instrument_function)) __attribute__((no_instrument_function)) void arch_local_irq_restore(unsigned long flags)
{
 native_restore_fl(flags);
}

static inline __attribute__((no_instrument_function)) __attribute__((no_instrument_function)) void arch_local_irq_disable(void)
{
 native_irq_disable();
}

static inline __attribute__((no_instrument_function)) __attribute__((no_instrument_function)) void arch_local_irq_enable(void)
{
 native_irq_enable();
}





static inline __attribute__((no_instrument_function)) void arch_safe_halt(void)
{
 native_safe_halt();
}





static inline __attribute__((no_instrument_function)) void halt(void)
{
 native_halt();
}




static inline __attribute__((no_instrument_function)) __attribute__((no_instrument_function)) unsigned long arch_local_irq_save(void)
{
 unsigned long flags = arch_local_save_flags();
 arch_local_irq_disable();
 return flags;
}
static inline __attribute__((no_instrument_function)) int arch_irqs_disabled_flags(unsigned long flags)
{
 return !(flags & ((1UL) << (9)));
}

static inline __attribute__((no_instrument_function)) int arch_irqs_disabled(void)
{
 unsigned long flags = arch_local_save_flags();

 return arch_irqs_disabled_flags(flags);
}
static inline __attribute__((no_instrument_function)) void *current_text_addr(void)
{
 void *pc;

 asm volatile("mov $1f, %0; 1:":"=r" (pc));

 return pc;
}
enum tlb_infos {
 ENTRIES,
 NR_INFO
};

extern u16 __attribute__((__section__(".data..read_mostly"))) tlb_lli_4k[NR_INFO];
extern u16 __attribute__((__section__(".data..read_mostly"))) tlb_lli_2m[NR_INFO];
extern u16 __attribute__((__section__(".data..read_mostly"))) tlb_lli_4m[NR_INFO];
extern u16 __attribute__((__section__(".data..read_mostly"))) tlb_lld_4k[NR_INFO];
extern u16 __attribute__((__section__(".data..read_mostly"))) tlb_lld_2m[NR_INFO];
extern u16 __attribute__((__section__(".data..read_mostly"))) tlb_lld_4m[NR_INFO];
extern u16 __attribute__((__section__(".data..read_mostly"))) tlb_lld_1g[NR_INFO];







struct cpuinfo_x86 {
 __u8 x86;
 __u8 x86_vendor;
 __u8 x86_model;
 __u8 x86_mask;
 int x86_tlbsize;

 __u8 x86_virt_bits;
 __u8 x86_phys_bits;

 __u8 x86_coreid_bits;

 __u32 extended_cpuid_level;

 int cpuid_level;
 __u32 x86_capability[11 + 1];
 char x86_vendor_id[16];
 char x86_model_id[64];

 int x86_cache_size;
 int x86_cache_alignment;
 int x86_power;
 unsigned long loops_per_jiffy;

 u16 x86_max_cores;
 u16 apicid;
 u16 initial_apicid;
 u16 x86_clflush_size;

 u16 booted_cores;

 u16 phys_proc_id;

 u16 cpu_core_id;

 u8 compute_unit_id;

 u16 cpu_index;
 u32 microcode;
} __attribute__((__aligned__((1 << (6)))));
extern struct cpuinfo_x86 boot_cpu_data;
extern struct cpuinfo_x86 new_cpu_data;

extern struct tss_struct doublefault_tss;
extern __u32 cpu_caps_cleared[11];
extern __u32 cpu_caps_set[11];


extern __attribute__((section(".data..percpu" ""))) __typeof__(struct cpuinfo_x86) cpu_info __attribute__((__aligned__((1 << (6)))));






extern const struct seq_operations cpuinfo_op;



extern void cpu_detect(struct cpuinfo_x86 *c);
extern void fpu_detect(struct cpuinfo_x86 *c);

extern void early_cpu_init(void);
extern void identify_boot_cpu(void);
extern void identify_secondary_cpu(struct cpuinfo_x86 *);
extern void print_cpu_info(struct cpuinfo_x86 *);
void print_cpu_msr(struct cpuinfo_x86 *);
extern void init_scattered_cpuid_features(struct cpuinfo_x86 *c);
extern unsigned int init_intel_cacheinfo(struct cpuinfo_x86 *c);
extern void init_amd_cacheinfo(struct cpuinfo_x86 *c);

extern void detect_extended_topology(struct cpuinfo_x86 *c);
extern void detect_ht(struct cpuinfo_x86 *c);




static inline __attribute__((no_instrument_function)) int have_cpuid_p(void)
{
 return 1;
}

static inline __attribute__((no_instrument_function)) void native_cpuid(unsigned int *eax, unsigned int *ebx,
    unsigned int *ecx, unsigned int *edx)
{

 asm volatile("cpuid"
     : "=a" (*eax),
       "=b" (*ebx),
       "=c" (*ecx),
       "=d" (*edx)
     : "0" (*eax), "2" (*ecx)
     : "memory");
}

static inline __attribute__((no_instrument_function)) void load_cr3(pgd_t *pgdir)
{
 write_cr3(__phys_addr_nodebug((unsigned long)(pgdir)));
}
struct x86_hw_tss {
 u32 reserved1;
 u64 sp0;
 u64 sp1;
 u64 sp2;
 u64 reserved2;
 u64 ist[7];
 u32 reserved3;
 u32 reserved4;
 u16 reserved5;
 u16 io_bitmap_base;

} __attribute__((packed)) __attribute__((__aligned__((1 << (6)))));
struct tss_struct {



 struct x86_hw_tss x86_tss;







 unsigned long io_bitmap[((65536/8)/sizeof(long)) + 1];




 unsigned long stack[64];

} __attribute__((__aligned__((1 << (6)))));

extern __attribute__((section(".data..percpu" ""))) __typeof__(struct tss_struct) init_tss __attribute__((__aligned__((1 << (6)))));




struct orig_ist {
 unsigned long ist[7];
};



struct i387_fsave_struct {
 u32 cwd;
 u32 swd;
 u32 twd;
 u32 fip;
 u32 fcs;
 u32 foo;
 u32 fos;


 u32 st_space[20];


 u32 status;
};

struct i387_fxsave_struct {
 u16 cwd;
 u16 swd;
 u16 twd;
 u16 fop;
 union {
  struct {
   u64 rip;
   u64 rdp;
  };
  struct {
   u32 fip;
   u32 fcs;
   u32 foo;
   u32 fos;
  };
 };
 u32 mxcsr;
 u32 mxcsr_mask;


 u32 st_space[32];


 u32 xmm_space[64];

 u32 padding[12];

 union {
  u32 padding1[12];
  u32 sw_reserved[12];
 };

} __attribute__((aligned(16)));

struct i387_soft_struct {
 u32 cwd;
 u32 swd;
 u32 twd;
 u32 fip;
 u32 fcs;
 u32 foo;
 u32 fos;

 u32 st_space[20];
 u8 ftop;
 u8 changed;
 u8 lookahead;
 u8 no_update;
 u8 rm;
 u8 alimit;
 struct math_emu_info *info;
 u32 entry_eip;
};

struct ymmh_struct {

 u32 ymmh_space[64];
};


struct lwp_struct {
 u8 reserved[128];
};

struct bndregs_struct {
 u64 bndregs[8];
} __attribute__((packed));

struct bndcsr_struct {
 u64 cfg_reg_u;
 u64 status_reg;
} __attribute__((packed));

struct xsave_hdr_struct {
 u64 xstate_bv;
 u64 xcomp_bv;
 u64 reserved[6];
} __attribute__((packed));

struct xsave_struct {
 struct i387_fxsave_struct i387;
 struct xsave_hdr_struct xsave_hdr;
 struct ymmh_struct ymmh;
 struct lwp_struct lwp;
 struct bndregs_struct bndregs;
 struct bndcsr_struct bndcsr;

} __attribute__ ((packed, aligned (64)));

union thread_xstate {
 struct i387_fsave_struct fsave;
 struct i387_fxsave_struct fxsave;
 struct i387_soft_struct soft;
 struct xsave_struct xsave;
};

struct fpu {
 unsigned int last_cpu;
 unsigned int has_fpu;
 union thread_xstate *state;
};


extern __attribute__((section(".data..percpu" ""))) __typeof__(struct orig_ist) orig_ist;

union irq_stack_union {
 char irq_stack[(((1UL) << 12) << 2)];





 struct {
  char gs_base[40];
  unsigned long stack_canary;
 };
};

extern __attribute__((section(".data..percpu" "..first"))) __typeof__(union irq_stack_union) irq_stack_union __attribute__((externally_visible));
extern typeof(irq_stack_union) init_per_cpu__irq_stack_union;

extern __attribute__((section(".data..percpu" ""))) __typeof__(char *) irq_stack_ptr;
extern __attribute__((section(".data..percpu" ""))) __typeof__(unsigned int) irq_count;
extern void ignore_sysret(void);
extern unsigned int xstate_size;
extern void free_thread_xstate(struct task_struct *);
extern struct kmem_cache *task_xstate_cachep;

struct perf_event;

struct thread_struct {

 struct desc_struct tls_array[3];
 unsigned long sp0;
 unsigned long sp;



 unsigned long usersp;
 unsigned short es;
 unsigned short ds;
 unsigned short fsindex;
 unsigned short gsindex;





 unsigned long fs;

 unsigned long gs;

 struct perf_event *ptrace_bps[4];

 unsigned long debugreg6;

 unsigned long ptrace_dr7;

 unsigned long cr2;
 unsigned long trap_nr;
 unsigned long error_code;

 struct fpu fpu;
 unsigned long *io_bitmap_ptr;
 unsigned long iopl;

 unsigned io_bitmap_max;
 unsigned char fpu_counter;
};




static inline __attribute__((no_instrument_function)) void native_set_iopl_mask(unsigned mask)
{
}

static inline __attribute__((no_instrument_function)) void
native_load_sp0(struct tss_struct *tss, struct thread_struct *thread)
{
 tss->x86_tss.sp0 = thread->sp0;







}

static inline __attribute__((no_instrument_function)) void native_swapgs(void)
{

 asm volatile("swapgs" ::: "memory");

}







static inline __attribute__((no_instrument_function)) void load_sp0(struct tss_struct *tss,
       struct thread_struct *thread)
{
 native_load_sp0(tss, thread);
}
extern unsigned long mmu_cr4_features;
extern u32 *trampoline_cr4_features;

static inline __attribute__((no_instrument_function)) void set_in_cr4(unsigned long mask)
{
 unsigned long cr4;

 mmu_cr4_features |= mask;
 if (trampoline_cr4_features)
  *trampoline_cr4_features = mmu_cr4_features;
 cr4 = read_cr4();
 cr4 |= mask;
 write_cr4(cr4);
}

static inline __attribute__((no_instrument_function)) void clear_in_cr4(unsigned long mask)
{
 unsigned long cr4;

 mmu_cr4_features &= ~mask;
 if (trampoline_cr4_features)
  *trampoline_cr4_features = mmu_cr4_features;
 cr4 = read_cr4();
 cr4 &= ~mask;
 write_cr4(cr4);
}

typedef struct {
 unsigned long seg;
} mm_segment_t;



extern void release_thread(struct task_struct *);

unsigned long get_wchan(struct task_struct *p);






static inline __attribute__((no_instrument_function)) void cpuid(unsigned int op,
    unsigned int *eax, unsigned int *ebx,
    unsigned int *ecx, unsigned int *edx)
{
 *eax = op;
 *ecx = 0;
 native_cpuid(eax, ebx, ecx, edx);
}


static inline __attribute__((no_instrument_function)) void cpuid_count(unsigned int op, int count,
          unsigned int *eax, unsigned int *ebx,
          unsigned int *ecx, unsigned int *edx)
{
 *eax = op;
 *ecx = count;
 native_cpuid(eax, ebx, ecx, edx);
}




static inline __attribute__((no_instrument_function)) unsigned int cpuid_eax(unsigned int op)
{
 unsigned int eax, ebx, ecx, edx;

 cpuid(op, &eax, &ebx, &ecx, &edx);

 return eax;
}

static inline __attribute__((no_instrument_function)) unsigned int cpuid_ebx(unsigned int op)
{
 unsigned int eax, ebx, ecx, edx;

 cpuid(op, &eax, &ebx, &ecx, &edx);

 return ebx;
}

static inline __attribute__((no_instrument_function)) unsigned int cpuid_ecx(unsigned int op)
{
 unsigned int eax, ebx, ecx, edx;

 cpuid(op, &eax, &ebx, &ecx, &edx);

 return ecx;
}

static inline __attribute__((no_instrument_function)) unsigned int cpuid_edx(unsigned int op)
{
 unsigned int eax, ebx, ecx, edx;

 cpuid(op, &eax, &ebx, &ecx, &edx);

 return edx;
}


static inline __attribute__((no_instrument_function)) void rep_nop(void)
{
 asm volatile("rep; nop" ::: "memory");
}

static inline __attribute__((no_instrument_function)) void cpu_relax(void)
{
 rep_nop();
}




static inline __attribute__((no_instrument_function)) void sync_core(void)
{
 int tmp;
 asm volatile("cpuid"
       : "=a" (tmp)
       : "0" (1)
       : "ebx", "ecx", "edx", "memory");

}

extern void select_idle_routine(const struct cpuinfo_x86 *c);
extern void init_amd_e400_c1e_mask(void);

extern unsigned long boot_option_idle_override;
extern bool amd_e400_c1e_detected;

enum idle_boot_override {IDLE_NO_OVERRIDE=0, IDLE_HALT, IDLE_NOMWAIT,
    IDLE_POLL};

extern void enable_sep_cpu(void);
extern int sysenter_setup(void);

extern void early_trap_init(void);
void early_trap_pf_init(void);


extern struct desc_ptr early_gdt_descr;

extern void cpu_set_gdt(int);
extern void switch_to_new_gdt(int);
extern void load_percpu_segment(int);
extern void cpu_init(void);

static inline __attribute__((no_instrument_function)) unsigned long get_debugctlmsr(void)
{
 unsigned long debugctlmsr = 0;





 ((debugctlmsr) = native_read_msr((0x000001d9)));

 return debugctlmsr;
}

static inline __attribute__((no_instrument_function)) void update_debugctlmsr(unsigned long debugctlmsr)
{




 native_write_msr((0x000001d9), (u32)((u64)(debugctlmsr)), (u32)((u64)(debugctlmsr) >> 32));
}

extern void set_task_blockstep(struct task_struct *task, bool on);





extern unsigned int machine_id;
extern unsigned int machine_submodel_id;
extern unsigned int BIOS_revision;


extern int bootloader_type;
extern int bootloader_version;

extern char ignore_fpu_irq;
static inline __attribute__((no_instrument_function)) void prefetch(const void *x)
{
 asm volatile ("661:\n\t" "prefetcht0 (%1)" "\n662:\n" ".pushsection .altinstructions,\"a\"\n" " .long 661b - .\n" " .long " "663""1""f - .\n" " .word " "( 0*32+25)" "\n" " .byte " "662b-661b" "\n" " .byte " "664""1""f-""663""1""f" "\n" ".popsection\n" ".pushsection .discard,\"aw\",@progbits\n" " .byte 0xff + (" "664""1""f-""663""1""f" ") - (" "662b-661b" ")\n" ".popsection\n" ".pushsection .altinstr_replacement, \"ax\"\n" "663""1"":\n\t" "prefetchnta (%1)" "\n" "664""1" ":\n\t" ".popsection" : : "i" (0), "r" (x))


             ;
}






static inline __attribute__((no_instrument_function)) void prefetchw(const void *x)
{
 asm volatile ("661:\n\t" "prefetcht0 (%1)" "\n662:\n" ".pushsection .altinstructions,\"a\"\n" " .long 661b - .\n" " .long " "663""1""f - .\n" " .word " "( 1*32+31)" "\n" " .byte " "662b-661b" "\n" " .byte " "664""1""f-""663""1""f" "\n" ".popsection\n" ".pushsection .discard,\"aw\",@progbits\n" " .byte 0xff + (" "664""1""f-""663""1""f" ") - (" "662b-661b" ")\n" ".popsection\n" ".pushsection .altinstr_replacement, \"ax\"\n" "663""1"":\n\t" "prefetchw (%1)" "\n" "664""1" ":\n\t" ".popsection" : : "i" (0), "r" (x))


             ;
}

static inline __attribute__((no_instrument_function)) void spin_lock_prefetch(const void *x)
{
 prefetchw(x);
}
extern unsigned long KSTK_ESP(struct task_struct *task);




extern __attribute__((section(".data..percpu" ""))) __typeof__(unsigned long) old_rsp;



extern void start_thread(struct pt_regs *regs, unsigned long new_ip,
            unsigned long new_sp);
extern int get_tsc_mode(unsigned long adr);
extern int set_tsc_mode(unsigned int val);

extern u16 amd_get_nb_id(int cpu);

static inline __attribute__((no_instrument_function)) uint32_t hypervisor_cpuid_base(const char *sig, uint32_t leaves)
{
 uint32_t base, eax, signature[3];

 for (base = 0x40000000; base < 0x40010000; base += 0x100) {
  cpuid(base, &eax, &signature[0], &signature[1], &signature[2]);

  if (!memcmp(sig, signature, 12) &&
      (leaves == 0 || ((eax - base) >= leaves)))
   return base;
 }

 return 0;
}

extern unsigned long arch_align_stack(unsigned long sp);
extern void free_init_pages(char *what, unsigned long begin, unsigned long end);

void default_idle(void);






void stop_this_cpu(void *dummy);
void df_debug(struct pt_regs *regs, long error_code);










extern void __xchg_wrong_size(void)
 __attribute__((error("Bad argument size for xchg")));
extern void __cmpxchg_wrong_size(void)
 __attribute__((error("Bad argument size for cmpxchg")));
extern void __xadd_wrong_size(void)
 __attribute__((error("Bad argument size for xadd")));
extern void __add_wrong_size(void)
 __attribute__((error("Bad argument size for add")));



static inline __attribute__((no_instrument_function)) void set_64bit(volatile u64 *ptr, u64 val)
{
 *ptr = val;
}
static inline __attribute__((no_instrument_function)) int atomic_read(const atomic_t *v)
{
 return (*(volatile int *)&(v)->counter);
}
static inline __attribute__((no_instrument_function)) void atomic_set(atomic_t *v, int i)
{
 v->counter = i;
}
static inline __attribute__((no_instrument_function)) void atomic_add(int i, atomic_t *v)
{
 asm volatile(".pushsection .smp_locks,\"a\"\n" ".balign 4\n" ".long 671f - .\n" ".popsection\n" "671:" "\n\tlock; " "addl %1,%0"
       : "+m" (v->counter)
       : "ir" (i));
}
static inline __attribute__((no_instrument_function)) void atomic_sub(int i, atomic_t *v)
{
 asm volatile(".pushsection .smp_locks,\"a\"\n" ".balign 4\n" ".long 671f - .\n" ".popsection\n" "671:" "\n\tlock; " "subl %1,%0"
       : "+m" (v->counter)
       : "ir" (i));
}
static inline __attribute__((no_instrument_function)) int atomic_sub_and_test(int i, atomic_t *v)
{
 do { char c; asm volatile (".pushsection .smp_locks,\"a\"\n" ".balign 4\n" ".long 671f - .\n" ".popsection\n" "671:" "\n\tlock; " "subl" " %2, " "%0" "; set" "e" " %1" : "+m" (v->counter), "=qm" (c) : "er" (i) : "memory"); return c != 0; } while (0);
}







static inline __attribute__((no_instrument_function)) void atomic_inc(atomic_t *v)
{
 asm volatile(".pushsection .smp_locks,\"a\"\n" ".balign 4\n" ".long 671f - .\n" ".popsection\n" "671:" "\n\tlock; " "incl %0"
       : "+m" (v->counter));
}







static inline __attribute__((no_instrument_function)) void atomic_dec(atomic_t *v)
{
 asm volatile(".pushsection .smp_locks,\"a\"\n" ".balign 4\n" ".long 671f - .\n" ".popsection\n" "671:" "\n\tlock; " "decl %0"
       : "+m" (v->counter));
}
static inline __attribute__((no_instrument_function)) int atomic_dec_and_test(atomic_t *v)
{
 do { char c; asm volatile (".pushsection .smp_locks,\"a\"\n" ".balign 4\n" ".long 671f - .\n" ".popsection\n" "671:" "\n\tlock; " "decl" " " "%0" "; set" "e" " %1" : "+m" (v->counter), "=qm" (c) : : "memory"); return c != 0; } while (0);
}
static inline __attribute__((no_instrument_function)) int atomic_inc_and_test(atomic_t *v)
{
 do { char c; asm volatile (".pushsection .smp_locks,\"a\"\n" ".balign 4\n" ".long 671f - .\n" ".popsection\n" "671:" "\n\tlock; " "incl" " " "%0" "; set" "e" " %1" : "+m" (v->counter), "=qm" (c) : : "memory"); return c != 0; } while (0);
}
static inline __attribute__((no_instrument_function)) int atomic_add_negative(int i, atomic_t *v)
{
 do { char c; asm volatile (".pushsection .smp_locks,\"a\"\n" ".balign 4\n" ".long 671f - .\n" ".popsection\n" "671:" "\n\tlock; " "addl" " %2, " "%0" "; set" "s" " %1" : "+m" (v->counter), "=qm" (c) : "er" (i) : "memory"); return c != 0; } while (0);
}
static inline __attribute__((no_instrument_function)) int atomic_add_return(int i, atomic_t *v)
{
 return i + ({ __typeof__ (*(((&v->counter)))) __ret = (((i))); switch (sizeof(*(((&v->counter))))) { case 1: asm volatile (".pushsection .smp_locks,\"a\"\n" ".balign 4\n" ".long 671f - .\n" ".popsection\n" "671:" "\n\tlock; " "xadd" "b %b0, %1\n" : "+q" (__ret), "+m" (*(((&v->counter)))) : : "memory", "cc"); break; case 2: asm volatile (".pushsection .smp_locks,\"a\"\n" ".balign 4\n" ".long 671f - .\n" ".popsection\n" "671:" "\n\tlock; " "xadd" "w %w0, %1\n" : "+r" (__ret), "+m" (*(((&v->counter)))) : : "memory", "cc"); break; case 4: asm volatile (".pushsection .smp_locks,\"a\"\n" ".balign 4\n" ".long 671f - .\n" ".popsection\n" "671:" "\n\tlock; " "xadd" "l %0, %1\n" : "+r" (__ret), "+m" (*(((&v->counter)))) : : "memory", "cc"); break; case 8: asm volatile (".pushsection .smp_locks,\"a\"\n" ".balign 4\n" ".long 671f - .\n" ".popsection\n" "671:" "\n\tlock; " "xadd" "q %q0, %1\n" : "+r" (__ret), "+m" (*(((&v->counter)))) : : "memory", "cc"); break; default: __xadd_wrong_size(); } __ret; });
}
static inline __attribute__((no_instrument_function)) int atomic_sub_return(int i, atomic_t *v)
{
 return atomic_add_return(-i, v);
}




static inline __attribute__((no_instrument_function)) int atomic_cmpxchg(atomic_t *v, int old, int new)
{
 return ({ __typeof__(*((&v->counter))) __ret; __typeof__(*((&v->counter))) __old = ((old)); __typeof__(*((&v->counter))) __new = ((new)); switch ((sizeof(*(&v->counter)))) { case 1: { volatile u8 *__ptr = (volatile u8 *)((&v->counter)); asm volatile(".pushsection .smp_locks,\"a\"\n" ".balign 4\n" ".long 671f - .\n" ".popsection\n" "671:" "\n\tlock; " "cmpxchgb %2,%1" : "=a" (__ret), "+m" (*__ptr) : "q" (__new), "0" (__old) : "memory"); break; } case 2: { volatile u16 *__ptr = (volatile u16 *)((&v->counter)); asm volatile(".pushsection .smp_locks,\"a\"\n" ".balign 4\n" ".long 671f - .\n" ".popsection\n" "671:" "\n\tlock; " "cmpxchgw %2,%1" : "=a" (__ret), "+m" (*__ptr) : "r" (__new), "0" (__old) : "memory"); break; } case 4: { volatile u32 *__ptr = (volatile u32 *)((&v->counter)); asm volatile(".pushsection .smp_locks,\"a\"\n" ".balign 4\n" ".long 671f - .\n" ".popsection\n" "671:" "\n\tlock; " "cmpxchgl %2,%1" : "=a" (__ret), "+m" (*__ptr) : "r" (__new), "0" (__old) : "memory"); break; } case 8: { volatile u64 *__ptr = (volatile u64 *)((&v->counter)); asm volatile(".pushsection .smp_locks,\"a\"\n" ".balign 4\n" ".long 671f - .\n" ".popsection\n" "671:" "\n\tlock; " "cmpxchgq %2,%1" : "=a" (__ret), "+m" (*__ptr) : "r" (__new), "0" (__old) : "memory"); break; } default: __cmpxchg_wrong_size(); } __ret; });
}

static inline __attribute__((no_instrument_function)) int atomic_xchg(atomic_t *v, int new)
{
 return ({ __typeof__ (*((&v->counter))) __ret = ((new)); switch (sizeof(*((&v->counter)))) { case 1: asm volatile ("" "xchg" "b %b0, %1\n" : "+q" (__ret), "+m" (*((&v->counter))) : : "memory", "cc"); break; case 2: asm volatile ("" "xchg" "w %w0, %1\n" : "+r" (__ret), "+m" (*((&v->counter))) : : "memory", "cc"); break; case 4: asm volatile ("" "xchg" "l %0, %1\n" : "+r" (__ret), "+m" (*((&v->counter))) : : "memory", "cc"); break; case 8: asm volatile ("" "xchg" "q %q0, %1\n" : "+r" (__ret), "+m" (*((&v->counter))) : : "memory", "cc"); break; default: __xchg_wrong_size(); } __ret; });
}
static inline __attribute__((no_instrument_function)) int __atomic_add_unless(atomic_t *v, int a, int u)
{
 int c, old;
 c = atomic_read(v);
 for (;;) {
  if (__builtin_expect(!!(c == (u)), 0))
   break;
  old = atomic_cmpxchg((v), c, c + (a));
  if (__builtin_expect(!!(old == c), 1))
   break;
  c = old;
 }
 return c;
}
static inline __attribute__((no_instrument_function)) short int atomic_inc_short(short int *v)
{
 asm(".pushsection .smp_locks,\"a\"\n" ".balign 4\n" ".long 671f - .\n" ".popsection\n" "671:" "\n\tlock; " "addw $1, %0" : "+m" (*v));
 return *v;
}
static inline __attribute__((no_instrument_function)) void atomic_or_long(unsigned long *v1, unsigned long v2)
{
 asm(".pushsection .smp_locks,\"a\"\n" ".balign 4\n" ".long 671f - .\n" ".popsection\n" "671:" "\n\tlock; " "orq %1, %0" : "+m" (*v1) : "r" (v2));
}
static inline __attribute__((no_instrument_function)) long atomic64_read(const atomic64_t *v)
{
 return (*(volatile long *)&(v)->counter);
}
static inline __attribute__((no_instrument_function)) void atomic64_set(atomic64_t *v, long i)
{
 v->counter = i;
}
static inline __attribute__((no_instrument_function)) void atomic64_add(long i, atomic64_t *v)
{
 asm volatile(".pushsection .smp_locks,\"a\"\n" ".balign 4\n" ".long 671f - .\n" ".popsection\n" "671:" "\n\tlock; " "addq %1,%0"
       : "=m" (v->counter)
       : "er" (i), "m" (v->counter));
}
static inline __attribute__((no_instrument_function)) void atomic64_sub(long i, atomic64_t *v)
{
 asm volatile(".pushsection .smp_locks,\"a\"\n" ".balign 4\n" ".long 671f - .\n" ".popsection\n" "671:" "\n\tlock; " "subq %1,%0"
       : "=m" (v->counter)
       : "er" (i), "m" (v->counter));
}
static inline __attribute__((no_instrument_function)) int atomic64_sub_and_test(long i, atomic64_t *v)
{
 do { char c; asm volatile (".pushsection .smp_locks,\"a\"\n" ".balign 4\n" ".long 671f - .\n" ".popsection\n" "671:" "\n\tlock; " "subq" " %2, " "%0" "; set" "e" " %1" : "+m" (v->counter), "=qm" (c) : "er" (i) : "memory"); return c != 0; } while (0);
}







static inline __attribute__((no_instrument_function)) void atomic64_inc(atomic64_t *v)
{
 asm volatile(".pushsection .smp_locks,\"a\"\n" ".balign 4\n" ".long 671f - .\n" ".popsection\n" "671:" "\n\tlock; " "incq %0"
       : "=m" (v->counter)
       : "m" (v->counter));
}







static inline __attribute__((no_instrument_function)) void atomic64_dec(atomic64_t *v)
{
 asm volatile(".pushsection .smp_locks,\"a\"\n" ".balign 4\n" ".long 671f - .\n" ".popsection\n" "671:" "\n\tlock; " "decq %0"
       : "=m" (v->counter)
       : "m" (v->counter));
}
static inline __attribute__((no_instrument_function)) int atomic64_dec_and_test(atomic64_t *v)
{
 do { char c; asm volatile (".pushsection .smp_locks,\"a\"\n" ".balign 4\n" ".long 671f - .\n" ".popsection\n" "671:" "\n\tlock; " "decq" " " "%0" "; set" "e" " %1" : "+m" (v->counter), "=qm" (c) : : "memory"); return c != 0; } while (0);
}
static inline __attribute__((no_instrument_function)) int atomic64_inc_and_test(atomic64_t *v)
{
 do { char c; asm volatile (".pushsection .smp_locks,\"a\"\n" ".balign 4\n" ".long 671f - .\n" ".popsection\n" "671:" "\n\tlock; " "incq" " " "%0" "; set" "e" " %1" : "+m" (v->counter), "=qm" (c) : : "memory"); return c != 0; } while (0);
}
static inline __attribute__((no_instrument_function)) int atomic64_add_negative(long i, atomic64_t *v)
{
 do { char c; asm volatile (".pushsection .smp_locks,\"a\"\n" ".balign 4\n" ".long 671f - .\n" ".popsection\n" "671:" "\n\tlock; " "addq" " %2, " "%0" "; set" "s" " %1" : "+m" (v->counter), "=qm" (c) : "er" (i) : "memory"); return c != 0; } while (0);
}
static inline __attribute__((no_instrument_function)) long atomic64_add_return(long i, atomic64_t *v)
{
 return i + ({ __typeof__ (*(((&v->counter)))) __ret = (((i))); switch (sizeof(*(((&v->counter))))) { case 1: asm volatile (".pushsection .smp_locks,\"a\"\n" ".balign 4\n" ".long 671f - .\n" ".popsection\n" "671:" "\n\tlock; " "xadd" "b %b0, %1\n" : "+q" (__ret), "+m" (*(((&v->counter)))) : : "memory", "cc"); break; case 2: asm volatile (".pushsection .smp_locks,\"a\"\n" ".balign 4\n" ".long 671f - .\n" ".popsection\n" "671:" "\n\tlock; " "xadd" "w %w0, %1\n" : "+r" (__ret), "+m" (*(((&v->counter)))) : : "memory", "cc"); break; case 4: asm volatile (".pushsection .smp_locks,\"a\"\n" ".balign 4\n" ".long 671f - .\n" ".popsection\n" "671:" "\n\tlock; " "xadd" "l %0, %1\n" : "+r" (__ret), "+m" (*(((&v->counter)))) : : "memory", "cc"); break; case 8: asm volatile (".pushsection .smp_locks,\"a\"\n" ".balign 4\n" ".long 671f - .\n" ".popsection\n" "671:" "\n\tlock; " "xadd" "q %q0, %1\n" : "+r" (__ret), "+m" (*(((&v->counter)))) : : "memory", "cc"); break; default: __xadd_wrong_size(); } __ret; });
}

static inline __attribute__((no_instrument_function)) long atomic64_sub_return(long i, atomic64_t *v)
{
 return atomic64_add_return(-i, v);
}




static inline __attribute__((no_instrument_function)) long atomic64_cmpxchg(atomic64_t *v, long old, long new)
{
 return ({ __typeof__(*((&v->counter))) __ret; __typeof__(*((&v->counter))) __old = ((old)); __typeof__(*((&v->counter))) __new = ((new)); switch ((sizeof(*(&v->counter)))) { case 1: { volatile u8 *__ptr = (volatile u8 *)((&v->counter)); asm volatile(".pushsection .smp_locks,\"a\"\n" ".balign 4\n" ".long 671f - .\n" ".popsection\n" "671:" "\n\tlock; " "cmpxchgb %2,%1" : "=a" (__ret), "+m" (*__ptr) : "q" (__new), "0" (__old) : "memory"); break; } case 2: { volatile u16 *__ptr = (volatile u16 *)((&v->counter)); asm volatile(".pushsection .smp_locks,\"a\"\n" ".balign 4\n" ".long 671f - .\n" ".popsection\n" "671:" "\n\tlock; " "cmpxchgw %2,%1" : "=a" (__ret), "+m" (*__ptr) : "r" (__new), "0" (__old) : "memory"); break; } case 4: { volatile u32 *__ptr = (volatile u32 *)((&v->counter)); asm volatile(".pushsection .smp_locks,\"a\"\n" ".balign 4\n" ".long 671f - .\n" ".popsection\n" "671:" "\n\tlock; " "cmpxchgl %2,%1" : "=a" (__ret), "+m" (*__ptr) : "r" (__new), "0" (__old) : "memory"); break; } case 8: { volatile u64 *__ptr = (volatile u64 *)((&v->counter)); asm volatile(".pushsection .smp_locks,\"a\"\n" ".balign 4\n" ".long 671f - .\n" ".popsection\n" "671:" "\n\tlock; " "cmpxchgq %2,%1" : "=a" (__ret), "+m" (*__ptr) : "r" (__new), "0" (__old) : "memory"); break; } default: __cmpxchg_wrong_size(); } __ret; });
}

static inline __attribute__((no_instrument_function)) long atomic64_xchg(atomic64_t *v, long new)
{
 return ({ __typeof__ (*((&v->counter))) __ret = ((new)); switch (sizeof(*((&v->counter)))) { case 1: asm volatile ("" "xchg" "b %b0, %1\n" : "+q" (__ret), "+m" (*((&v->counter))) : : "memory", "cc"); break; case 2: asm volatile ("" "xchg" "w %w0, %1\n" : "+r" (__ret), "+m" (*((&v->counter))) : : "memory", "cc"); break; case 4: asm volatile ("" "xchg" "l %0, %1\n" : "+r" (__ret), "+m" (*((&v->counter))) : : "memory", "cc"); break; case 8: asm volatile ("" "xchg" "q %q0, %1\n" : "+r" (__ret), "+m" (*((&v->counter))) : : "memory", "cc"); break; default: __xchg_wrong_size(); } __ret; });
}
static inline __attribute__((no_instrument_function)) int atomic64_add_unless(atomic64_t *v, long a, long u)
{
 long c, old;
 c = atomic64_read(v);
 for (;;) {
  if (__builtin_expect(!!(c == (u)), 0))
   break;
  old = atomic64_cmpxchg((v), c, c + (a));
  if (__builtin_expect(!!(old == c), 1))
   break;
  c = old;
 }
 return c != (u);
}
static inline __attribute__((no_instrument_function)) long atomic64_dec_if_positive(atomic64_t *v)
{
 long c, old, dec;
 c = atomic64_read(v);
 for (;;) {
  dec = c - 1;
  if (__builtin_expect(!!(dec < 0), 0))
   break;
  old = atomic64_cmpxchg((v), c, dec);
  if (__builtin_expect(!!(old == c), 1))
   break;
  c = old;
 }
 return dec;
}






static inline __attribute__((no_instrument_function)) void smp_mb__before_atomic_inc(void)
{
 extern void __smp_mb__before_atomic(void);
 __smp_mb__before_atomic();
}



static inline __attribute__((no_instrument_function)) void smp_mb__after_atomic_inc(void)
{
 extern void __smp_mb__after_atomic(void);
 __smp_mb__after_atomic();
}



static inline __attribute__((no_instrument_function)) void smp_mb__before_atomic_dec(void)
{
 extern void __smp_mb__before_atomic(void);
 __smp_mb__before_atomic();
}



static inline __attribute__((no_instrument_function)) void smp_mb__after_atomic_dec(void)
{
 extern void __smp_mb__after_atomic(void);
 __smp_mb__after_atomic();
}
static inline __attribute__((no_instrument_function)) int atomic_add_unless(atomic_t *v, int a, int u)
{
 return __atomic_add_unless(v, a, u) != u;
}
static inline __attribute__((no_instrument_function)) int atomic_inc_not_zero_hint(atomic_t *v, int hint)
{
 int val, c = hint;


 if (!hint)
  return atomic_add_unless((v), 1, 0);

 do {
  val = atomic_cmpxchg(v, c, c + 1);
  if (val == c)
   return 1;
  c = val;
 } while (c);

 return 0;
}



static inline __attribute__((no_instrument_function)) int atomic_inc_unless_negative(atomic_t *p)
{
 int v, v1;
 for (v = 0; v >= 0; v = v1) {
  v1 = atomic_cmpxchg(p, v, v + 1);
  if (__builtin_expect(!!(v1 == v), 1))
   return 1;
 }
 return 0;
}



static inline __attribute__((no_instrument_function)) int atomic_dec_unless_positive(atomic_t *p)
{
 int v, v1;
 for (v = 0; v <= 0; v = v1) {
  v1 = atomic_cmpxchg(p, v, v - 1);
  if (__builtin_expect(!!(v1 == v), 1))
   return 1;
 }
 return 0;
}
static inline __attribute__((no_instrument_function)) int atomic_dec_if_positive(atomic_t *v)
{
 int c, old, dec;
 c = atomic_read(v);
 for (;;) {
  dec = c - 1;
  if (__builtin_expect(!!(dec < 0), 0))
   break;
  old = atomic_cmpxchg((v), c, dec);
  if (__builtin_expect(!!(old == c), 1))
   break;
  c = old;
 }
 return dec;
}



static inline __attribute__((no_instrument_function)) void atomic_or(int i, atomic_t *v)
{
 int old;
 int new;

 do {
  old = atomic_read(v);
  new = old | i;
 } while (atomic_cmpxchg(v, old, new) != old);
}


typedef atomic64_t atomic_long_t;



static inline __attribute__((no_instrument_function)) long atomic_long_read(atomic_long_t *l)
{
 atomic64_t *v = (atomic64_t *)l;

 return (long)atomic64_read(v);
}

static inline __attribute__((no_instrument_function)) void atomic_long_set(atomic_long_t *l, long i)
{
 atomic64_t *v = (atomic64_t *)l;

 atomic64_set(v, i);
}

static inline __attribute__((no_instrument_function)) void atomic_long_inc(atomic_long_t *l)
{
 atomic64_t *v = (atomic64_t *)l;

 atomic64_inc(v);
}

static inline __attribute__((no_instrument_function)) void atomic_long_dec(atomic_long_t *l)
{
 atomic64_t *v = (atomic64_t *)l;

 atomic64_dec(v);
}

static inline __attribute__((no_instrument_function)) void atomic_long_add(long i, atomic_long_t *l)
{
 atomic64_t *v = (atomic64_t *)l;

 atomic64_add(i, v);
}

static inline __attribute__((no_instrument_function)) void atomic_long_sub(long i, atomic_long_t *l)
{
 atomic64_t *v = (atomic64_t *)l;

 atomic64_sub(i, v);
}

static inline __attribute__((no_instrument_function)) int atomic_long_sub_and_test(long i, atomic_long_t *l)
{
 atomic64_t *v = (atomic64_t *)l;

 return atomic64_sub_and_test(i, v);
}

static inline __attribute__((no_instrument_function)) int atomic_long_dec_and_test(atomic_long_t *l)
{
 atomic64_t *v = (atomic64_t *)l;

 return atomic64_dec_and_test(v);
}

static inline __attribute__((no_instrument_function)) int atomic_long_inc_and_test(atomic_long_t *l)
{
 atomic64_t *v = (atomic64_t *)l;

 return atomic64_inc_and_test(v);
}

static inline __attribute__((no_instrument_function)) int atomic_long_add_negative(long i, atomic_long_t *l)
{
 atomic64_t *v = (atomic64_t *)l;

 return atomic64_add_negative(i, v);
}

static inline __attribute__((no_instrument_function)) long atomic_long_add_return(long i, atomic_long_t *l)
{
 atomic64_t *v = (atomic64_t *)l;

 return (long)atomic64_add_return(i, v);
}

static inline __attribute__((no_instrument_function)) long atomic_long_sub_return(long i, atomic_long_t *l)
{
 atomic64_t *v = (atomic64_t *)l;

 return (long)atomic64_sub_return(i, v);
}

static inline __attribute__((no_instrument_function)) long atomic_long_inc_return(atomic_long_t *l)
{
 atomic64_t *v = (atomic64_t *)l;

 return (long)(atomic64_add_return(1, (v)));
}

static inline __attribute__((no_instrument_function)) long atomic_long_dec_return(atomic_long_t *l)
{
 atomic64_t *v = (atomic64_t *)l;

 return (long)(atomic64_sub_return(1, (v)));
}

static inline __attribute__((no_instrument_function)) long atomic_long_add_unless(atomic_long_t *l, long a, long u)
{
 atomic64_t *v = (atomic64_t *)l;

 return (long)atomic64_add_unless(v, a, u);
}

struct thread_info {
 struct task_struct *task;
 struct exec_domain *exec_domain;
 __u32 flags;
 __u32 status;
 __u32 cpu;
 int saved_preempt_count;
 mm_segment_t addr_limit;
 struct restart_block restart_block;
 void *sysenter_return;
 unsigned int sig_on_uaccess_error:1;
 unsigned int uaccess_err:1;
};
extern __attribute__((section(".data..percpu" ""))) __typeof__(unsigned long) kernel_stack;

static inline __attribute__((no_instrument_function)) struct thread_info *current_thread_info(void)
{
 struct thread_info *ti;
 ti = (void *)(({ typeof(kernel_stack) pfo_ret__; switch (sizeof(kernel_stack)) { case 1: asm("mov" "b ""%%""gs"":" "%P" "1"",%0" : "=q" (pfo_ret__) : "p" (&(kernel_stack))); break; case 2: asm("mov" "w ""%%""gs"":" "%P" "1"",%0" : "=r" (pfo_ret__) : "p" (&(kernel_stack))); break; case 4: asm("mov" "l ""%%""gs"":" "%P" "1"",%0" : "=r" (pfo_ret__) : "p" (&(kernel_stack))); break; case 8: asm("mov" "q ""%%""gs"":" "%P" "1"",%0" : "=r" (pfo_ret__) : "p" (&(kernel_stack))); break; default: __bad_percpu_size(); } pfo_ret__; }) +
        (5*(64/8)) - (((1UL) << 12) << 2));
 return ti;
}
static inline __attribute__((no_instrument_function)) void set_restore_sigmask(void)
{
 struct thread_info *ti = current_thread_info();
 ti->status |= 0x0008;
 ({ int __ret_warn_on = !!(!(__builtin_constant_p((2)) ? constant_test_bit((2), ((unsigned long *)&ti->flags)) : variable_test_bit((2), ((unsigned long *)&ti->flags)))); if (__builtin_expect(!!(__ret_warn_on), 0)) warn_slowpath_null("./arch/x86/include/asm/thread_info.h", 204); __builtin_expect(!!(__ret_warn_on), 0); });
}
static inline __attribute__((no_instrument_function)) void clear_restore_sigmask(void)
{
 current_thread_info()->status &= ~0x0008;
}
static inline __attribute__((no_instrument_function)) bool test_restore_sigmask(void)
{
 return current_thread_info()->status & 0x0008;
}
static inline __attribute__((no_instrument_function)) bool test_and_clear_restore_sigmask(void)
{
 struct thread_info *ti = current_thread_info();
 if (!(ti->status & 0x0008))
  return false;
 ti->status &= ~0x0008;
 return true;
}

static inline __attribute__((no_instrument_function)) bool is_ia32_task(void)
{




 if (current_thread_info()->status & 0x0002)
  return true;

 return false;
}



extern void arch_task_cache_init(void);
extern int arch_dup_task_struct(struct task_struct *dst, struct task_struct *src);
extern void arch_release_task_struct(struct task_struct *tsk);
static inline __attribute__((no_instrument_function)) void set_ti_thread_flag(struct thread_info *ti, int flag)
{
 set_bit(flag, (unsigned long *)&ti->flags);
}

static inline __attribute__((no_instrument_function)) void clear_ti_thread_flag(struct thread_info *ti, int flag)
{
 clear_bit(flag, (unsigned long *)&ti->flags);
}

static inline __attribute__((no_instrument_function)) int test_and_set_ti_thread_flag(struct thread_info *ti, int flag)
{
 return test_and_set_bit(flag, (unsigned long *)&ti->flags);
}

static inline __attribute__((no_instrument_function)) int test_and_clear_ti_thread_flag(struct thread_info *ti, int flag)
{
 return test_and_clear_bit(flag, (unsigned long *)&ti->flags);
}

static inline __attribute__((no_instrument_function)) int test_ti_thread_flag(struct thread_info *ti, int flag)
{
 return (__builtin_constant_p((flag)) ? constant_test_bit((flag), ((unsigned long *)&ti->flags)) : variable_test_bit((flag), ((unsigned long *)&ti->flags)));
}

extern __attribute__((section(".data..percpu" ""))) __typeof__(int) __preempt_count;
static inline __attribute__((no_instrument_function)) __attribute__((always_inline)) int preempt_count(void)
{
 return ({ typeof((__preempt_count)) pfo_ret__; switch (sizeof((__preempt_count))) { case 1: asm("mov" "b ""%%""gs"":" "%P" "1"",%0" : "=q" (pfo_ret__) : "m"(__preempt_count)); break; case 2: asm("mov" "w ""%%""gs"":" "%P" "1"",%0" : "=r" (pfo_ret__) : "m"(__preempt_count)); break; case 4: asm("mov" "l ""%%""gs"":" "%P" "1"",%0" : "=r" (pfo_ret__) : "m"(__preempt_count)); break; case 8: asm("mov" "q ""%%""gs"":" "%P" "1"",%0" : "=r" (pfo_ret__) : "m"(__preempt_count)); break; default: __bad_percpu_size(); } pfo_ret__; }) & ~0x80000000;
}

static inline __attribute__((no_instrument_function)) __attribute__((always_inline)) void preempt_count_set(int pc)
{
 do { typedef typeof((__preempt_count)) pto_T__; if (0) { pto_T__ pto_tmp__; pto_tmp__ = (pc); (void)pto_tmp__; } switch (sizeof((__preempt_count))) { case 1: asm("mov" "b %1,""%%""gs"":" "%P" "0" : "+m" ((__preempt_count)) : "qi" ((pto_T__)(pc))); break; case 2: asm("mov" "w %1,""%%""gs"":" "%P" "0" : "+m" ((__preempt_count)) : "ri" ((pto_T__)(pc))); break; case 4: asm("mov" "l %1,""%%""gs"":" "%P" "0" : "+m" ((__preempt_count)) : "ri" ((pto_T__)(pc))); break; case 8: asm("mov" "q %1,""%%""gs"":" "%P" "0" : "+m" ((__preempt_count)) : "re" ((pto_T__)(pc))); break; default: __bad_percpu_size(); } } while (0);
}
static inline __attribute__((no_instrument_function)) __attribute__((always_inline)) void set_preempt_need_resched(void)
{
 do { typedef typeof((__preempt_count)) pto_T__; if (0) { pto_T__ pto_tmp__; pto_tmp__ = (~0x80000000); (void)pto_tmp__; } switch (sizeof((__preempt_count))) { case 1: asm("and" "b %1,""%%""gs"":" "%P" "0" : "+m" ((__preempt_count)) : "qi" ((pto_T__)(~0x80000000))); break; case 2: asm("and" "w %1,""%%""gs"":" "%P" "0" : "+m" ((__preempt_count)) : "ri" ((pto_T__)(~0x80000000))); break; case 4: asm("and" "l %1,""%%""gs"":" "%P" "0" : "+m" ((__preempt_count)) : "ri" ((pto_T__)(~0x80000000))); break; case 8: asm("and" "q %1,""%%""gs"":" "%P" "0" : "+m" ((__preempt_count)) : "re" ((pto_T__)(~0x80000000))); break; default: __bad_percpu_size(); } } while (0);
}

static inline __attribute__((no_instrument_function)) __attribute__((always_inline)) void clear_preempt_need_resched(void)
{
 do { typedef typeof((__preempt_count)) pto_T__; if (0) { pto_T__ pto_tmp__; pto_tmp__ = (0x80000000); (void)pto_tmp__; } switch (sizeof((__preempt_count))) { case 1: asm("or" "b %1,""%%""gs"":" "%P" "0" : "+m" ((__preempt_count)) : "qi" ((pto_T__)(0x80000000))); break; case 2: asm("or" "w %1,""%%""gs"":" "%P" "0" : "+m" ((__preempt_count)) : "ri" ((pto_T__)(0x80000000))); break; case 4: asm("or" "l %1,""%%""gs"":" "%P" "0" : "+m" ((__preempt_count)) : "ri" ((pto_T__)(0x80000000))); break; case 8: asm("or" "q %1,""%%""gs"":" "%P" "0" : "+m" ((__preempt_count)) : "re" ((pto_T__)(0x80000000))); break; default: __bad_percpu_size(); } } while (0);
}

static inline __attribute__((no_instrument_function)) __attribute__((always_inline)) bool test_preempt_need_resched(void)
{
 return !(({ typeof((__preempt_count)) pfo_ret__; switch (sizeof((__preempt_count))) { case 1: asm("mov" "b ""%%""gs"":" "%P" "1"",%0" : "=q" (pfo_ret__) : "m"(__preempt_count)); break; case 2: asm("mov" "w ""%%""gs"":" "%P" "1"",%0" : "=r" (pfo_ret__) : "m"(__preempt_count)); break; case 4: asm("mov" "l ""%%""gs"":" "%P" "1"",%0" : "=r" (pfo_ret__) : "m"(__preempt_count)); break; case 8: asm("mov" "q ""%%""gs"":" "%P" "1"",%0" : "=r" (pfo_ret__) : "m"(__preempt_count)); break; default: __bad_percpu_size(); } pfo_ret__; }) & 0x80000000);
}





static inline __attribute__((no_instrument_function)) __attribute__((always_inline)) void __preempt_count_add(int val)
{
 do { typedef typeof((__preempt_count)) pao_T__; const int pao_ID__ = (__builtin_constant_p(val) && ((val) == 1 || (val) == -1)) ? (int)(val) : 0; if (0) { pao_T__ pao_tmp__; pao_tmp__ = (val); (void)pao_tmp__; } switch (sizeof((__preempt_count))) { case 1: if (pao_ID__ == 1) asm("incb ""%%""gs"":" "%P" "0" : "+m" ((__preempt_count))); else if (pao_ID__ == -1) asm("decb ""%%""gs"":" "%P" "0" : "+m" ((__preempt_count))); else asm("addb %1, ""%%""gs"":" "%P" "0" : "+m" ((__preempt_count)) : "qi" ((pao_T__)(val))); break; case 2: if (pao_ID__ == 1) asm("incw ""%%""gs"":" "%P" "0" : "+m" ((__preempt_count))); else if (pao_ID__ == -1) asm("decw ""%%""gs"":" "%P" "0" : "+m" ((__preempt_count))); else asm("addw %1, ""%%""gs"":" "%P" "0" : "+m" ((__preempt_count)) : "ri" ((pao_T__)(val))); break; case 4: if (pao_ID__ == 1) asm("incl ""%%""gs"":" "%P" "0" : "+m" ((__preempt_count))); else if (pao_ID__ == -1) asm("decl ""%%""gs"":" "%P" "0" : "+m" ((__preempt_count))); else asm("addl %1, ""%%""gs"":" "%P" "0" : "+m" ((__preempt_count)) : "ri" ((pao_T__)(val))); break; case 8: if (pao_ID__ == 1) asm("incq ""%%""gs"":" "%P" "0" : "+m" ((__preempt_count))); else if (pao_ID__ == -1) asm("decq ""%%""gs"":" "%P" "0" : "+m" ((__preempt_count))); else asm("addq %1, ""%%""gs"":" "%P" "0" : "+m" ((__preempt_count)) : "re" ((pao_T__)(val))); break; default: __bad_percpu_size(); } } while (0);
}

static inline __attribute__((no_instrument_function)) __attribute__((always_inline)) void __preempt_count_sub(int val)
{
 do { typedef typeof((__preempt_count)) pao_T__; const int pao_ID__ = (__builtin_constant_p(-val) && ((-val) == 1 || (-val) == -1)) ? (int)(-val) : 0; if (0) { pao_T__ pao_tmp__; pao_tmp__ = (-val); (void)pao_tmp__; } switch (sizeof((__preempt_count))) { case 1: if (pao_ID__ == 1) asm("incb ""%%""gs"":" "%P" "0" : "+m" ((__preempt_count))); else if (pao_ID__ == -1) asm("decb ""%%""gs"":" "%P" "0" : "+m" ((__preempt_count))); else asm("addb %1, ""%%""gs"":" "%P" "0" : "+m" ((__preempt_count)) : "qi" ((pao_T__)(-val))); break; case 2: if (pao_ID__ == 1) asm("incw ""%%""gs"":" "%P" "0" : "+m" ((__preempt_count))); else if (pao_ID__ == -1) asm("decw ""%%""gs"":" "%P" "0" : "+m" ((__preempt_count))); else asm("addw %1, ""%%""gs"":" "%P" "0" : "+m" ((__preempt_count)) : "ri" ((pao_T__)(-val))); break; case 4: if (pao_ID__ == 1) asm("incl ""%%""gs"":" "%P" "0" : "+m" ((__preempt_count))); else if (pao_ID__ == -1) asm("decl ""%%""gs"":" "%P" "0" : "+m" ((__preempt_count))); else asm("addl %1, ""%%""gs"":" "%P" "0" : "+m" ((__preempt_count)) : "ri" ((pao_T__)(-val))); break; case 8: if (pao_ID__ == 1) asm("incq ""%%""gs"":" "%P" "0" : "+m" ((__preempt_count))); else if (pao_ID__ == -1) asm("decq ""%%""gs"":" "%P" "0" : "+m" ((__preempt_count))); else asm("addq %1, ""%%""gs"":" "%P" "0" : "+m" ((__preempt_count)) : "re" ((pao_T__)(-val))); break; default: __bad_percpu_size(); } } while (0);
}






static inline __attribute__((no_instrument_function)) __attribute__((always_inline)) bool __preempt_count_dec_and_test(void)
{
 do { char c; asm volatile ("decl" " " "%%""gs"":" "%P" "0" "; set" "e" " %1" : "+m" (__preempt_count), "=qm" (c) : : "memory"); return c != 0; } while (0);
}




static inline __attribute__((no_instrument_function)) __attribute__((always_inline)) bool should_resched(void)
{
 return __builtin_expect(!!(!({ typeof((__preempt_count)) pfo_ret__; switch (sizeof((__preempt_count))) { case 1: asm("mov" "b ""%%""gs"":" "%P" "1"",%0" : "=q" (pfo_ret__) : "m"(__preempt_count)); break; case 2: asm("mov" "w ""%%""gs"":" "%P" "1"",%0" : "=r" (pfo_ret__) : "m"(__preempt_count)); break; case 4: asm("mov" "l ""%%""gs"":" "%P" "1"",%0" : "=r" (pfo_ret__) : "m"(__preempt_count)); break; case 8: asm("mov" "q ""%%""gs"":" "%P" "1"",%0" : "=r" (pfo_ret__) : "m"(__preempt_count)); break; default: __bad_percpu_size(); } pfo_ret__; })), 0);
}
struct preempt_notifier;
struct preempt_ops {
 void (*sched_in)(struct preempt_notifier *notifier, int cpu);
 void (*sched_out)(struct preempt_notifier *notifier,
     struct task_struct *next);
};
struct preempt_notifier {
 struct hlist_node link;
 struct preempt_ops *ops;
};

void preempt_notifier_register(struct preempt_notifier *notifier);
void preempt_notifier_unregister(struct preempt_notifier *notifier);

static inline __attribute__((no_instrument_function)) void preempt_notifier_init(struct preempt_notifier *notifier,
         struct preempt_ops *ops)
{
 INIT_HLIST_NODE(&notifier->link);
 notifier->ops = ops;
}














static inline __attribute__((no_instrument_function)) __attribute__((always_inline)) void __local_bh_disable_ip(unsigned long ip, unsigned int cnt)
{
 __preempt_count_add(cnt);
 __asm__ __volatile__("": : :"memory");
}


static inline __attribute__((no_instrument_function)) void local_bh_disable(void)
{
 __local_bh_disable_ip(({ __label__ __here; __here: (unsigned long)&&__here; }), (2 * (1UL << (0 + 8))));
}

extern void _local_bh_enable(void);
extern void __local_bh_enable_ip(unsigned long ip, unsigned int cnt);

static inline __attribute__((no_instrument_function)) void local_bh_enable_ip(unsigned long ip)
{
 __local_bh_enable_ip(ip, (2 * (1UL << (0 + 8))));
}

static inline __attribute__((no_instrument_function)) void local_bh_enable(void)
{
 __local_bh_enable_ip(({ __label__ __here; __here: (unsigned long)&&__here; }), (2 * (1UL << (0 + 8))));
}
typedef u16 __ticket_t;
typedef u32 __ticketpair_t;






typedef struct arch_spinlock {
 union {
  __ticketpair_t head_tail;
  struct __raw_tickets {
   __ticket_t head, tail;
  } tickets;
 };
} arch_spinlock_t;













typedef struct qrwlock {
 atomic_t cnts;
 arch_spinlock_t lock;
} arch_rwlock_t;




struct task_struct;
struct lockdep_map;


extern int prove_locking;
extern int lock_stat;
static inline __attribute__((no_instrument_function)) void lockdep_off(void)
{
}

static inline __attribute__((no_instrument_function)) void lockdep_on(void)
{
}
struct lock_class_key { };
static inline __attribute__((no_instrument_function)) void print_irqtrace_events(struct task_struct *curr)
{
}

typedef struct raw_spinlock {
 arch_spinlock_t raw_lock;
} raw_spinlock_t;
typedef struct spinlock {
 union {
  struct raw_spinlock rlock;
 };
} spinlock_t;
typedef struct {
 arch_rwlock_t raw_lock;
} rwlock_t;








extern bool static_key_initialized;
struct static_key {
 atomic_t enabled;
};


enum jump_label_type {
 JUMP_LABEL_DISABLE = 0,
 JUMP_LABEL_ENABLE,
};

struct module;



static inline __attribute__((no_instrument_function)) int static_key_count(struct static_key *key)
{
 return atomic_read(&key->enabled);
}
static inline __attribute__((no_instrument_function)) __attribute__((always_inline)) void jump_label_init(void)
{
 static_key_initialized = true;
}

static inline __attribute__((no_instrument_function)) __attribute__((always_inline)) bool static_key_false(struct static_key *key)
{
 if (__builtin_expect(!!(static_key_count(key) > 0), 0))
  return true;
 return false;
}

static inline __attribute__((no_instrument_function)) __attribute__((always_inline)) bool static_key_true(struct static_key *key)
{
 if (__builtin_expect(!!(static_key_count(key) > 0), 1))
  return true;
 return false;
}

static inline __attribute__((no_instrument_function)) void static_key_slow_inc(struct static_key *key)
{
 ({ int __ret_warn_on = !!(!static_key_initialized); if (__builtin_expect(!!(__ret_warn_on), 0)) warn_slowpath_fmt("include/linux/jump_label.h", 168, "%s used before call to jump_label_init", __func__); __builtin_expect(!!(__ret_warn_on), 0); });
 atomic_inc(&key->enabled);
}

static inline __attribute__((no_instrument_function)) void static_key_slow_dec(struct static_key *key)
{
 ({ int __ret_warn_on = !!(!static_key_initialized); if (__builtin_expect(!!(__ret_warn_on), 0)) warn_slowpath_fmt("include/linux/jump_label.h", 174, "%s used before call to jump_label_init", __func__); __builtin_expect(!!(__ret_warn_on), 0); });
 atomic_dec(&key->enabled);
}

static inline __attribute__((no_instrument_function)) int jump_label_text_reserved(void *start, void *end)
{
 return 0;
}

static inline __attribute__((no_instrument_function)) void jump_label_lock(void) {}
static inline __attribute__((no_instrument_function)) void jump_label_unlock(void) {}

static inline __attribute__((no_instrument_function)) int jump_label_apply_nops(struct module *mod)
{
 return 0;
}
static inline __attribute__((no_instrument_function)) bool static_key_enabled(struct static_key *key)
{
 return static_key_count(key) > 0;
}




extern struct static_key paravirt_ticketlocks_enabled;
static inline __attribute__((no_instrument_function)) __attribute__((always_inline)) bool static_key_false(struct static_key *key);
static inline __attribute__((no_instrument_function)) __attribute__((always_inline)) void __ticket_lock_spinning(arch_spinlock_t *lock,
       __ticket_t ticket)
{
}
static inline __attribute__((no_instrument_function)) void __ticket_unlock_kick(arch_spinlock_t *lock,
       __ticket_t ticket)
{
}



static inline __attribute__((no_instrument_function)) __attribute__((always_inline)) int arch_spin_value_unlocked(arch_spinlock_t lock)
{
 return lock.tickets.head == lock.tickets.tail;
}
static inline __attribute__((no_instrument_function)) __attribute__((always_inline)) void arch_spin_lock(arch_spinlock_t *lock)
{
 register struct __raw_tickets inc = { .tail = ((__ticket_t)1) };

 inc = ({ __typeof__ (*(((&lock->tickets)))) __ret = (((inc))); switch (sizeof(*(((&lock->tickets))))) { case 1: asm volatile (".pushsection .smp_locks,\"a\"\n" ".balign 4\n" ".long 671f - .\n" ".popsection\n" "671:" "\n\tlock; " "xadd" "b %b0, %1\n" : "+q" (__ret), "+m" (*(((&lock->tickets)))) : : "memory", "cc"); break; case 2: asm volatile (".pushsection .smp_locks,\"a\"\n" ".balign 4\n" ".long 671f - .\n" ".popsection\n" "671:" "\n\tlock; " "xadd" "w %w0, %1\n" : "+r" (__ret), "+m" (*(((&lock->tickets)))) : : "memory", "cc"); break; case 4: asm volatile (".pushsection .smp_locks,\"a\"\n" ".balign 4\n" ".long 671f - .\n" ".popsection\n" "671:" "\n\tlock; " "xadd" "l %0, %1\n" : "+r" (__ret), "+m" (*(((&lock->tickets)))) : : "memory", "cc"); break; case 8: asm volatile (".pushsection .smp_locks,\"a\"\n" ".balign 4\n" ".long 671f - .\n" ".popsection\n" "671:" "\n\tlock; " "xadd" "q %q0, %1\n" : "+r" (__ret), "+m" (*(((&lock->tickets)))) : : "memory", "cc"); break; default: __xadd_wrong_size(); } __ret; });
 if (__builtin_expect(!!(inc.head == inc.tail), 1))
  goto out;

 inc.tail &= ~((__ticket_t)0);
 for (;;) {
  unsigned count = (1 << 15);

  do {
   if ((*(volatile typeof(lock->tickets.head) *)&(lock->tickets.head)) == inc.tail)
    goto out;
   cpu_relax();
  } while (--count);
  __ticket_lock_spinning(lock, inc.tail);
 }
out: __asm__ __volatile__("": : :"memory");
}

static inline __attribute__((no_instrument_function)) __attribute__((always_inline)) int arch_spin_trylock(arch_spinlock_t *lock)
{
 arch_spinlock_t old, new;

 old.tickets = (*(volatile typeof(lock->tickets) *)&(lock->tickets));
 if (old.tickets.head != (old.tickets.tail & ~((__ticket_t)0)))
  return 0;

 new.head_tail = old.head_tail + (((__ticket_t)1) << (sizeof(__ticket_t) * 8));


 return ({ __typeof__(*((&lock->head_tail))) __ret; __typeof__(*((&lock->head_tail))) __old = ((old.head_tail)); __typeof__(*((&lock->head_tail))) __new = ((new.head_tail)); switch ((sizeof(*(&lock->head_tail)))) { case 1: { volatile u8 *__ptr = (volatile u8 *)((&lock->head_tail)); asm volatile(".pushsection .smp_locks,\"a\"\n" ".balign 4\n" ".long 671f - .\n" ".popsection\n" "671:" "\n\tlock; " "cmpxchgb %2,%1" : "=a" (__ret), "+m" (*__ptr) : "q" (__new), "0" (__old) : "memory"); break; } case 2: { volatile u16 *__ptr = (volatile u16 *)((&lock->head_tail)); asm volatile(".pushsection .smp_locks,\"a\"\n" ".balign 4\n" ".long 671f - .\n" ".popsection\n" "671:" "\n\tlock; " "cmpxchgw %2,%1" : "=a" (__ret), "+m" (*__ptr) : "r" (__new), "0" (__old) : "memory"); break; } case 4: { volatile u32 *__ptr = (volatile u32 *)((&lock->head_tail)); asm volatile(".pushsection .smp_locks,\"a\"\n" ".balign 4\n" ".long 671f - .\n" ".popsection\n" "671:" "\n\tlock; " "cmpxchgl %2,%1" : "=a" (__ret), "+m" (*__ptr) : "r" (__new), "0" (__old) : "memory"); break; } case 8: { volatile u64 *__ptr = (volatile u64 *)((&lock->head_tail)); asm volatile(".pushsection .smp_locks,\"a\"\n" ".balign 4\n" ".long 671f - .\n" ".popsection\n" "671:" "\n\tlock; " "cmpxchgq %2,%1" : "=a" (__ret), "+m" (*__ptr) : "r" (__new), "0" (__old) : "memory"); break; } default: __cmpxchg_wrong_size(); } __ret; }) == old.head_tail;
}

static inline __attribute__((no_instrument_function)) void __ticket_unlock_slowpath(arch_spinlock_t *lock,
         arch_spinlock_t old)
{
 arch_spinlock_t new;

 ((void)sizeof(char[1 - 2*!!(((__ticket_t)256) != 256)]));


 old.tickets.head += ((__ticket_t)1);


 new.head_tail = old.head_tail & ~(((__ticket_t)0) << (sizeof(__ticket_t) * 8));





 if (new.tickets.head != new.tickets.tail ||
     ({ __typeof__(*((&lock->head_tail))) __ret; __typeof__(*((&lock->head_tail))) __old = ((old.head_tail)); __typeof__(*((&lock->head_tail))) __new = ((new.head_tail)); switch ((sizeof(*(&lock->head_tail)))) { case 1: { volatile u8 *__ptr = (volatile u8 *)((&lock->head_tail)); asm volatile(".pushsection .smp_locks,\"a\"\n" ".balign 4\n" ".long 671f - .\n" ".popsection\n" "671:" "\n\tlock; " "cmpxchgb %2,%1" : "=a" (__ret), "+m" (*__ptr) : "q" (__new), "0" (__old) : "memory"); break; } case 2: { volatile u16 *__ptr = (volatile u16 *)((&lock->head_tail)); asm volatile(".pushsection .smp_locks,\"a\"\n" ".balign 4\n" ".long 671f - .\n" ".popsection\n" "671:" "\n\tlock; " "cmpxchgw %2,%1" : "=a" (__ret), "+m" (*__ptr) : "r" (__new), "0" (__old) : "memory"); break; } case 4: { volatile u32 *__ptr = (volatile u32 *)((&lock->head_tail)); asm volatile(".pushsection .smp_locks,\"a\"\n" ".balign 4\n" ".long 671f - .\n" ".popsection\n" "671:" "\n\tlock; " "cmpxchgl %2,%1" : "=a" (__ret), "+m" (*__ptr) : "r" (__new), "0" (__old) : "memory"); break; } case 8: { volatile u64 *__ptr = (volatile u64 *)((&lock->head_tail)); asm volatile(".pushsection .smp_locks,\"a\"\n" ".balign 4\n" ".long 671f - .\n" ".popsection\n" "671:" "\n\tlock; " "cmpxchgq %2,%1" : "=a" (__ret), "+m" (*__ptr) : "r" (__new), "0" (__old) : "memory"); break; } default: __cmpxchg_wrong_size(); } __ret; })
                    != old.head_tail) {




  __ticket_unlock_kick(lock, old.tickets.head);
 }
}

static inline __attribute__((no_instrument_function)) __attribute__((always_inline)) void arch_spin_unlock(arch_spinlock_t *lock)
{
 if (((__ticket_t)0) &&
     static_key_false(&paravirt_ticketlocks_enabled)) {
  arch_spinlock_t prev;

  prev = *lock;
  ({ __typeof__ (*((&lock->tickets.head))) __ret = ((((__ticket_t)1))); switch (sizeof(*((&lock->tickets.head)))) { case 1: asm volatile (".pushsection .smp_locks,\"a\"\n" ".balign 4\n" ".long 671f - .\n" ".popsection\n" "671:" "\n\tlock; " "addb %b1, %0\n" : "+m" (*((&lock->tickets.head))) : "qi" ((((__ticket_t)1))) : "memory", "cc"); break; case 2: asm volatile (".pushsection .smp_locks,\"a\"\n" ".balign 4\n" ".long 671f - .\n" ".popsection\n" "671:" "\n\tlock; " "addw %w1, %0\n" : "+m" (*((&lock->tickets.head))) : "ri" ((((__ticket_t)1))) : "memory", "cc"); break; case 4: asm volatile (".pushsection .smp_locks,\"a\"\n" ".balign 4\n" ".long 671f - .\n" ".popsection\n" "671:" "\n\tlock; " "addl %1, %0\n" : "+m" (*((&lock->tickets.head))) : "ri" ((((__ticket_t)1))) : "memory", "cc"); break; case 8: asm volatile (".pushsection .smp_locks,\"a\"\n" ".balign 4\n" ".long 671f - .\n" ".popsection\n" "671:" "\n\tlock; " "addq %1, %0\n" : "+m" (*((&lock->tickets.head))) : "ri" ((((__ticket_t)1))) : "memory", "cc"); break; default: __add_wrong_size(); } __ret; });



  if (__builtin_expect(!!(lock->tickets.tail & ((__ticket_t)0)), 0))
   __ticket_unlock_slowpath(lock, prev);
 } else
  ({ __typeof__ (*(&lock->tickets.head)) __ret = (((__ticket_t)1)); switch (sizeof(*(&lock->tickets.head))) { case 1: asm volatile ( "addb %b1, %0\n" : "+m" (*(&lock->tickets.head)) : "qi" (((__ticket_t)1)) : "memory", "cc"); break; case 2: asm volatile ( "addw %w1, %0\n" : "+m" (*(&lock->tickets.head)) : "ri" (((__ticket_t)1)) : "memory", "cc"); break; case 4: asm volatile ( "addl %1, %0\n" : "+m" (*(&lock->tickets.head)) : "ri" (((__ticket_t)1)) : "memory", "cc"); break; case 8: asm volatile ( "addq %1, %0\n" : "+m" (*(&lock->tickets.head)) : "ri" (((__ticket_t)1)) : "memory", "cc"); break; default: __add_wrong_size(); } __ret; });
}

static inline __attribute__((no_instrument_function)) int arch_spin_is_locked(arch_spinlock_t *lock)
{
 struct __raw_tickets tmp = (*(volatile typeof(lock->tickets) *)&(lock->tickets));

 return tmp.tail != tmp.head;
}

static inline __attribute__((no_instrument_function)) int arch_spin_is_contended(arch_spinlock_t *lock)
{
 struct __raw_tickets tmp = (*(volatile typeof(lock->tickets) *)&(lock->tickets));

 return (__ticket_t)(tmp.tail - tmp.head) > ((__ticket_t)1);
}


static inline __attribute__((no_instrument_function)) __attribute__((always_inline)) void arch_spin_lock_flags(arch_spinlock_t *lock,
        unsigned long flags)
{
 arch_spin_lock(lock);
}

static inline __attribute__((no_instrument_function)) void arch_spin_unlock_wait(arch_spinlock_t *lock)
{
 while (arch_spin_is_locked(lock))
  cpu_relax();
}







static inline __attribute__((no_instrument_function)) void queue_write_unlock(struct qrwlock *lock)
{
        __asm__ __volatile__("": : :"memory");
        (*(volatile typeof(*(u8 *)&lock->cnts) *)&(*(u8 *)&lock->cnts)) = 0;
}


extern void queue_read_lock_slowpath(struct qrwlock *lock);
extern void queue_write_lock_slowpath(struct qrwlock *lock);





static inline __attribute__((no_instrument_function)) int queue_read_can_lock(struct qrwlock *lock)
{
 return !(atomic_read(&lock->cnts) & 0xff);
}





static inline __attribute__((no_instrument_function)) int queue_write_can_lock(struct qrwlock *lock)
{
 return !atomic_read(&lock->cnts);
}






static inline __attribute__((no_instrument_function)) int queue_read_trylock(struct qrwlock *lock)
{
 u32 cnts;

 cnts = atomic_read(&lock->cnts);
 if (__builtin_expect(!!(!(cnts & 0xff)), 1)) {
  cnts = (u32)atomic_add_return((1U << 8), &lock->cnts);
  if (__builtin_expect(!!(!(cnts & 0xff)), 1))
   return 1;
  atomic_sub((1U << 8), &lock->cnts);
 }
 return 0;
}






static inline __attribute__((no_instrument_function)) int queue_write_trylock(struct qrwlock *lock)
{
 u32 cnts;

 cnts = atomic_read(&lock->cnts);
 if (__builtin_expect(!!(cnts), 0))
  return 0;

 return __builtin_expect(!!(atomic_cmpxchg(&lock->cnts, cnts, cnts | 0xff) == cnts), 1)
                                          ;
}




static inline __attribute__((no_instrument_function)) void queue_read_lock(struct qrwlock *lock)
{
 u32 cnts;

 cnts = atomic_add_return((1U << 8), &lock->cnts);
 if (__builtin_expect(!!(!(cnts & 0xff)), 1))
  return;


 queue_read_lock_slowpath(lock);
}





static inline __attribute__((no_instrument_function)) void queue_write_lock(struct qrwlock *lock)
{

 if (atomic_cmpxchg(&lock->cnts, 0, 0xff) == 0)
  return;

 queue_write_lock_slowpath(lock);
}





static inline __attribute__((no_instrument_function)) void queue_read_unlock(struct qrwlock *lock)
{



 __asm__ __volatile__("": : :"memory");
 atomic_sub((1U << 8), &lock->cnts);
}
static inline __attribute__((no_instrument_function)) void do_raw_spin_lock(raw_spinlock_t *lock)
{
 (void)0;
 arch_spin_lock(&lock->raw_lock);
}

static inline __attribute__((no_instrument_function)) void
do_raw_spin_lock_flags(raw_spinlock_t *lock, unsigned long *flags)
{
 (void)0;
 arch_spin_lock_flags(&lock->raw_lock, *flags);
}

static inline __attribute__((no_instrument_function)) int do_raw_spin_trylock(raw_spinlock_t *lock)
{
 return arch_spin_trylock(&(lock)->raw_lock);
}

static inline __attribute__((no_instrument_function)) void do_raw_spin_unlock(raw_spinlock_t *lock)
{
 arch_spin_unlock(&lock->raw_lock);
 (void)0;
}





int in_lock_functions(unsigned long addr);



void __attribute__((section(".spinlock.text"))) _raw_spin_lock(raw_spinlock_t *lock) ;
void __attribute__((section(".spinlock.text"))) _raw_spin_lock_nested(raw_spinlock_t *lock, int subclass)
        ;
void __attribute__((section(".spinlock.text")))
_raw_spin_lock_nest_lock(raw_spinlock_t *lock, struct lockdep_map *map)
        ;
void __attribute__((section(".spinlock.text"))) _raw_spin_lock_bh(raw_spinlock_t *lock) ;
void __attribute__((section(".spinlock.text"))) _raw_spin_lock_irq(raw_spinlock_t *lock)
        ;

unsigned long __attribute__((section(".spinlock.text"))) _raw_spin_lock_irqsave(raw_spinlock_t *lock)
        ;
unsigned long __attribute__((section(".spinlock.text")))
_raw_spin_lock_irqsave_nested(raw_spinlock_t *lock, int subclass)
        ;
int __attribute__((section(".spinlock.text"))) _raw_spin_trylock(raw_spinlock_t *lock);
int __attribute__((section(".spinlock.text"))) _raw_spin_trylock_bh(raw_spinlock_t *lock);
void __attribute__((section(".spinlock.text"))) _raw_spin_unlock(raw_spinlock_t *lock) ;
void __attribute__((section(".spinlock.text"))) _raw_spin_unlock_bh(raw_spinlock_t *lock) ;
void __attribute__((section(".spinlock.text"))) _raw_spin_unlock_irq(raw_spinlock_t *lock) ;
void __attribute__((section(".spinlock.text")))
_raw_spin_unlock_irqrestore(raw_spinlock_t *lock, unsigned long flags)
        ;
static inline __attribute__((no_instrument_function)) int __raw_spin_trylock(raw_spinlock_t *lock)
{
 __asm__ __volatile__("": : :"memory");
 if (do_raw_spin_trylock(lock)) {
  do { } while (0);
  return 1;
 }
 __asm__ __volatile__("": : :"memory");
 return 0;
}
static inline __attribute__((no_instrument_function)) unsigned long __raw_spin_lock_irqsave(raw_spinlock_t *lock)
{
 unsigned long flags;

 do { do { ({ unsigned long __dummy; typeof(flags) __dummy2; (void)(&__dummy == &__dummy2); 1; }); flags = arch_local_irq_save(); } while (0); do { } while (0); } while (0);
 __asm__ __volatile__("": : :"memory");
 do { } while (0);
 do_raw_spin_lock_flags(lock, &flags);

 return flags;
}

static inline __attribute__((no_instrument_function)) void __raw_spin_lock_irq(raw_spinlock_t *lock)
{
 do { arch_local_irq_disable(); do { } while (0); } while (0);
 __asm__ __volatile__("": : :"memory");
 do { } while (0);
 do_raw_spin_lock(lock);
}

static inline __attribute__((no_instrument_function)) void __raw_spin_lock_bh(raw_spinlock_t *lock)
{
 __local_bh_disable_ip((unsigned long)__builtin_return_address(0), ((2 * (1UL << (0 + 8))) + 0));
 do { } while (0);
 do_raw_spin_lock(lock);
}

static inline __attribute__((no_instrument_function)) void __raw_spin_lock(raw_spinlock_t *lock)
{
 __asm__ __volatile__("": : :"memory");
 do { } while (0);
 do_raw_spin_lock(lock);
}



static inline __attribute__((no_instrument_function)) void __raw_spin_unlock(raw_spinlock_t *lock)
{
 do { } while (0);
 do_raw_spin_unlock(lock);
 __asm__ __volatile__("": : :"memory");
}

static inline __attribute__((no_instrument_function)) void __raw_spin_unlock_irqrestore(raw_spinlock_t *lock,
         unsigned long flags)
{
 do { } while (0);
 do_raw_spin_unlock(lock);
 do { if (({ ({ unsigned long __dummy; typeof(flags) __dummy2; (void)(&__dummy == &__dummy2); 1; }); arch_irqs_disabled_flags(flags); })) { do { ({ unsigned long __dummy; typeof(flags) __dummy2; (void)(&__dummy == &__dummy2); 1; }); arch_local_irq_restore(flags); } while (0); do { } while (0); } else { do { } while (0); do { ({ unsigned long __dummy; typeof(flags) __dummy2; (void)(&__dummy == &__dummy2); 1; }); arch_local_irq_restore(flags); } while (0); } } while (0);
 __asm__ __volatile__("": : :"memory");
}

static inline __attribute__((no_instrument_function)) void __raw_spin_unlock_irq(raw_spinlock_t *lock)
{
 do { } while (0);
 do_raw_spin_unlock(lock);
 do { do { } while (0); arch_local_irq_enable(); } while (0);
 __asm__ __volatile__("": : :"memory");
}

static inline __attribute__((no_instrument_function)) void __raw_spin_unlock_bh(raw_spinlock_t *lock)
{
 do { } while (0);
 do_raw_spin_unlock(lock);
 __local_bh_enable_ip((unsigned long)__builtin_return_address(0), ((2 * (1UL << (0 + 8))) + 0));
}

static inline __attribute__((no_instrument_function)) int __raw_spin_trylock_bh(raw_spinlock_t *lock)
{
 __local_bh_disable_ip((unsigned long)__builtin_return_address(0), ((2 * (1UL << (0 + 8))) + 0));
 if (do_raw_spin_trylock(lock)) {
  do { } while (0);
  return 1;
 }
 __local_bh_enable_ip((unsigned long)__builtin_return_address(0), ((2 * (1UL << (0 + 8))) + 0));
 return 0;
}

void __attribute__((section(".spinlock.text"))) _raw_read_lock(rwlock_t *lock) ;
void __attribute__((section(".spinlock.text"))) _raw_write_lock(rwlock_t *lock) ;
void __attribute__((section(".spinlock.text"))) _raw_read_lock_bh(rwlock_t *lock) ;
void __attribute__((section(".spinlock.text"))) _raw_write_lock_bh(rwlock_t *lock) ;
void __attribute__((section(".spinlock.text"))) _raw_read_lock_irq(rwlock_t *lock) ;
void __attribute__((section(".spinlock.text"))) _raw_write_lock_irq(rwlock_t *lock) ;
unsigned long __attribute__((section(".spinlock.text"))) _raw_read_lock_irqsave(rwlock_t *lock)
       ;
unsigned long __attribute__((section(".spinlock.text"))) _raw_write_lock_irqsave(rwlock_t *lock)
       ;
int __attribute__((section(".spinlock.text"))) _raw_read_trylock(rwlock_t *lock);
int __attribute__((section(".spinlock.text"))) _raw_write_trylock(rwlock_t *lock);
void __attribute__((section(".spinlock.text"))) _raw_read_unlock(rwlock_t *lock) ;
void __attribute__((section(".spinlock.text"))) _raw_write_unlock(rwlock_t *lock) ;
void __attribute__((section(".spinlock.text"))) _raw_read_unlock_bh(rwlock_t *lock) ;
void __attribute__((section(".spinlock.text"))) _raw_write_unlock_bh(rwlock_t *lock) ;
void __attribute__((section(".spinlock.text"))) _raw_read_unlock_irq(rwlock_t *lock) ;
void __attribute__((section(".spinlock.text"))) _raw_write_unlock_irq(rwlock_t *lock) ;
void __attribute__((section(".spinlock.text")))
_raw_read_unlock_irqrestore(rwlock_t *lock, unsigned long flags)
       ;
void __attribute__((section(".spinlock.text")))
_raw_write_unlock_irqrestore(rwlock_t *lock, unsigned long flags)
       ;
static inline __attribute__((no_instrument_function)) int __raw_read_trylock(rwlock_t *lock)
{
 __asm__ __volatile__("": : :"memory");
 if (queue_read_trylock(&(lock)->raw_lock)) {
  do { } while (0);
  return 1;
 }
 __asm__ __volatile__("": : :"memory");
 return 0;
}

static inline __attribute__((no_instrument_function)) int __raw_write_trylock(rwlock_t *lock)
{
 __asm__ __volatile__("": : :"memory");
 if (queue_write_trylock(&(lock)->raw_lock)) {
  do { } while (0);
  return 1;
 }
 __asm__ __volatile__("": : :"memory");
 return 0;
}
static inline __attribute__((no_instrument_function)) void __raw_read_lock(rwlock_t *lock)
{
 __asm__ __volatile__("": : :"memory");
 do { } while (0);
 do {(void)0; queue_read_lock(&(lock)->raw_lock); } while (0);
}

static inline __attribute__((no_instrument_function)) unsigned long __raw_read_lock_irqsave(rwlock_t *lock)
{
 unsigned long flags;

 do { do { ({ unsigned long __dummy; typeof(flags) __dummy2; (void)(&__dummy == &__dummy2); 1; }); flags = arch_local_irq_save(); } while (0); do { } while (0); } while (0);
 __asm__ __volatile__("": : :"memory");
 do { } while (0);
 do {(void)0; queue_read_lock(&((lock))->raw_lock); } while (0)
                                       ;
 return flags;
}

static inline __attribute__((no_instrument_function)) void __raw_read_lock_irq(rwlock_t *lock)
{
 do { arch_local_irq_disable(); do { } while (0); } while (0);
 __asm__ __volatile__("": : :"memory");
 do { } while (0);
 do {(void)0; queue_read_lock(&(lock)->raw_lock); } while (0);
}

static inline __attribute__((no_instrument_function)) void __raw_read_lock_bh(rwlock_t *lock)
{
 __local_bh_disable_ip((unsigned long)__builtin_return_address(0), ((2 * (1UL << (0 + 8))) + 0));
 do { } while (0);
 do {(void)0; queue_read_lock(&(lock)->raw_lock); } while (0);
}

static inline __attribute__((no_instrument_function)) unsigned long __raw_write_lock_irqsave(rwlock_t *lock)
{
 unsigned long flags;

 do { do { ({ unsigned long __dummy; typeof(flags) __dummy2; (void)(&__dummy == &__dummy2); 1; }); flags = arch_local_irq_save(); } while (0); do { } while (0); } while (0);
 __asm__ __volatile__("": : :"memory");
 do { } while (0);
 do {(void)0; queue_write_lock(&((lock))->raw_lock); } while (0)
                                        ;
 return flags;
}

static inline __attribute__((no_instrument_function)) void __raw_write_lock_irq(rwlock_t *lock)
{
 do { arch_local_irq_disable(); do { } while (0); } while (0);
 __asm__ __volatile__("": : :"memory");
 do { } while (0);
 do {(void)0; queue_write_lock(&(lock)->raw_lock); } while (0);
}

static inline __attribute__((no_instrument_function)) void __raw_write_lock_bh(rwlock_t *lock)
{
 __local_bh_disable_ip((unsigned long)__builtin_return_address(0), ((2 * (1UL << (0 + 8))) + 0));
 do { } while (0);
 do {(void)0; queue_write_lock(&(lock)->raw_lock); } while (0);
}

static inline __attribute__((no_instrument_function)) void __raw_write_lock(rwlock_t *lock)
{
 __asm__ __volatile__("": : :"memory");
 do { } while (0);
 do {(void)0; queue_write_lock(&(lock)->raw_lock); } while (0);
}



static inline __attribute__((no_instrument_function)) void __raw_write_unlock(rwlock_t *lock)
{
 do { } while (0);
 do {queue_write_unlock(&(lock)->raw_lock); (void)0; } while (0);
 __asm__ __volatile__("": : :"memory");
}

static inline __attribute__((no_instrument_function)) void __raw_read_unlock(rwlock_t *lock)
{
 do { } while (0);
 do {queue_read_unlock(&(lock)->raw_lock); (void)0; } while (0);
 __asm__ __volatile__("": : :"memory");
}

static inline __attribute__((no_instrument_function)) void
__raw_read_unlock_irqrestore(rwlock_t *lock, unsigned long flags)
{
 do { } while (0);
 do {queue_read_unlock(&(lock)->raw_lock); (void)0; } while (0);
 do { if (({ ({ unsigned long __dummy; typeof(flags) __dummy2; (void)(&__dummy == &__dummy2); 1; }); arch_irqs_disabled_flags(flags); })) { do { ({ unsigned long __dummy; typeof(flags) __dummy2; (void)(&__dummy == &__dummy2); 1; }); arch_local_irq_restore(flags); } while (0); do { } while (0); } else { do { } while (0); do { ({ unsigned long __dummy; typeof(flags) __dummy2; (void)(&__dummy == &__dummy2); 1; }); arch_local_irq_restore(flags); } while (0); } } while (0);
 __asm__ __volatile__("": : :"memory");
}

static inline __attribute__((no_instrument_function)) void __raw_read_unlock_irq(rwlock_t *lock)
{
 do { } while (0);
 do {queue_read_unlock(&(lock)->raw_lock); (void)0; } while (0);
 do { do { } while (0); arch_local_irq_enable(); } while (0);
 __asm__ __volatile__("": : :"memory");
}

static inline __attribute__((no_instrument_function)) void __raw_read_unlock_bh(rwlock_t *lock)
{
 do { } while (0);
 do {queue_read_unlock(&(lock)->raw_lock); (void)0; } while (0);
 __local_bh_enable_ip((unsigned long)__builtin_return_address(0), ((2 * (1UL << (0 + 8))) + 0));
}

static inline __attribute__((no_instrument_function)) void __raw_write_unlock_irqrestore(rwlock_t *lock,
          unsigned long flags)
{
 do { } while (0);
 do {queue_write_unlock(&(lock)->raw_lock); (void)0; } while (0);
 do { if (({ ({ unsigned long __dummy; typeof(flags) __dummy2; (void)(&__dummy == &__dummy2); 1; }); arch_irqs_disabled_flags(flags); })) { do { ({ unsigned long __dummy; typeof(flags) __dummy2; (void)(&__dummy == &__dummy2); 1; }); arch_local_irq_restore(flags); } while (0); do { } while (0); } else { do { } while (0); do { ({ unsigned long __dummy; typeof(flags) __dummy2; (void)(&__dummy == &__dummy2); 1; }); arch_local_irq_restore(flags); } while (0); } } while (0);
 __asm__ __volatile__("": : :"memory");
}

static inline __attribute__((no_instrument_function)) void __raw_write_unlock_irq(rwlock_t *lock)
{
 do { } while (0);
 do {queue_write_unlock(&(lock)->raw_lock); (void)0; } while (0);
 do { do { } while (0); arch_local_irq_enable(); } while (0);
 __asm__ __volatile__("": : :"memory");
}

static inline __attribute__((no_instrument_function)) void __raw_write_unlock_bh(rwlock_t *lock)
{
 do { } while (0);
 do {queue_write_unlock(&(lock)->raw_lock); (void)0; } while (0);
 __local_bh_enable_ip((unsigned long)__builtin_return_address(0), ((2 * (1UL << (0 + 8))) + 0));
}
static inline __attribute__((no_instrument_function)) raw_spinlock_t *spinlock_check(spinlock_t *lock)
{
 return &lock->rlock;
}







static inline __attribute__((no_instrument_function)) void spin_lock(spinlock_t *lock)
{
 _raw_spin_lock(&lock->rlock);
}

static inline __attribute__((no_instrument_function)) void spin_lock_bh(spinlock_t *lock)
{
 _raw_spin_lock_bh(&lock->rlock);
}

static inline __attribute__((no_instrument_function)) int spin_trylock(spinlock_t *lock)
{
 return (_raw_spin_trylock(&lock->rlock));
}
static inline __attribute__((no_instrument_function)) void spin_lock_irq(spinlock_t *lock)
{
 _raw_spin_lock_irq(&lock->rlock);
}
static inline __attribute__((no_instrument_function)) void spin_unlock(spinlock_t *lock)
{
 __raw_spin_unlock(&lock->rlock);
}

static inline __attribute__((no_instrument_function)) void spin_unlock_bh(spinlock_t *lock)
{
 _raw_spin_unlock_bh(&lock->rlock);
}

static inline __attribute__((no_instrument_function)) void spin_unlock_irq(spinlock_t *lock)
{
 __raw_spin_unlock_irq(&lock->rlock);
}

static inline __attribute__((no_instrument_function)) void spin_unlock_irqrestore(spinlock_t *lock, unsigned long flags)
{
 do { ({ unsigned long __dummy; typeof(flags) __dummy2; (void)(&__dummy == &__dummy2); 1; }); _raw_spin_unlock_irqrestore(&lock->rlock, flags); } while (0);
}

static inline __attribute__((no_instrument_function)) int spin_trylock_bh(spinlock_t *lock)
{
 return (_raw_spin_trylock_bh(&lock->rlock));
}

static inline __attribute__((no_instrument_function)) int spin_trylock_irq(spinlock_t *lock)
{
 return ({ do { arch_local_irq_disable(); do { } while (0); } while (0); (_raw_spin_trylock(&lock->rlock)) ? 1 : ({ do { do { } while (0); arch_local_irq_enable(); } while (0); 0; }); });
}






static inline __attribute__((no_instrument_function)) void spin_unlock_wait(spinlock_t *lock)
{
 arch_spin_unlock_wait(&(&lock->rlock)->raw_lock);
}

static inline __attribute__((no_instrument_function)) int spin_is_locked(spinlock_t *lock)
{
 return arch_spin_is_locked(&(&lock->rlock)->raw_lock);
}

static inline __attribute__((no_instrument_function)) int spin_is_contended(spinlock_t *lock)
{
 return arch_spin_is_contended(&(&lock->rlock)->raw_lock);
}

static inline __attribute__((no_instrument_function)) int spin_can_lock(spinlock_t *lock)
{
 return (!arch_spin_is_locked(&(&lock->rlock)->raw_lock));
}
extern int _atomic_dec_and_lock(atomic_t *atomic, spinlock_t *lock);
typedef struct seqcount {
 unsigned sequence;



} seqcount_t;

static inline __attribute__((no_instrument_function)) void __seqcount_init(seqcount_t *s, const char *name,
       struct lock_class_key *key)
{



 do { (void)(name); (void)(key); } while (0);
 s->sequence = 0;
}
static inline __attribute__((no_instrument_function)) unsigned __read_seqcount_begin(const seqcount_t *s)
{
 unsigned ret;

repeat:
 ret = (*(volatile typeof(s->sequence) *)&(s->sequence));
 if (__builtin_expect(!!(ret & 1), 0)) {
  cpu_relax();
  goto repeat;
 }
 return ret;
}
static inline __attribute__((no_instrument_function)) unsigned raw_read_seqcount(const seqcount_t *s)
{
 unsigned ret = (*(volatile typeof(s->sequence) *)&(s->sequence));
 __asm__ __volatile__("": : :"memory");
 return ret;
}
static inline __attribute__((no_instrument_function)) unsigned raw_read_seqcount_begin(const seqcount_t *s)
{
 unsigned ret = __read_seqcount_begin(s);
 __asm__ __volatile__("": : :"memory");
 return ret;
}
static inline __attribute__((no_instrument_function)) unsigned read_seqcount_begin(const seqcount_t *s)
{
 ;
 return raw_read_seqcount_begin(s);
}
static inline __attribute__((no_instrument_function)) unsigned raw_seqcount_begin(const seqcount_t *s)
{
 unsigned ret = (*(volatile typeof(s->sequence) *)&(s->sequence));
 __asm__ __volatile__("": : :"memory");
 return ret & ~1;
}
static inline __attribute__((no_instrument_function)) int __read_seqcount_retry(const seqcount_t *s, unsigned start)
{
 return __builtin_expect(!!(s->sequence != start), 0);
}
static inline __attribute__((no_instrument_function)) int read_seqcount_retry(const seqcount_t *s, unsigned start)
{
 __asm__ __volatile__("": : :"memory");
 return __read_seqcount_retry(s, start);
}



static inline __attribute__((no_instrument_function)) void raw_write_seqcount_begin(seqcount_t *s)
{
 s->sequence++;
 __asm__ __volatile__("": : :"memory");
}

static inline __attribute__((no_instrument_function)) void raw_write_seqcount_end(seqcount_t *s)
{
 __asm__ __volatile__("": : :"memory");
 s->sequence++;
}





static inline __attribute__((no_instrument_function)) void raw_write_seqcount_latch(seqcount_t *s)
{
       __asm__ __volatile__("": : :"memory");
       s->sequence++;
       __asm__ __volatile__("": : :"memory");
}





static inline __attribute__((no_instrument_function)) void write_seqcount_begin_nested(seqcount_t *s, int subclass)
{
 raw_write_seqcount_begin(s);
 do { } while (0);
}

static inline __attribute__((no_instrument_function)) void write_seqcount_begin(seqcount_t *s)
{
 write_seqcount_begin_nested(s, 0);
}

static inline __attribute__((no_instrument_function)) void write_seqcount_end(seqcount_t *s)
{
 do { } while (0);
 raw_write_seqcount_end(s);
}
static inline __attribute__((no_instrument_function)) void write_seqcount_barrier(seqcount_t *s)
{
 __asm__ __volatile__("": : :"memory");
 s->sequence+=2;
}

typedef struct {
 struct seqcount seqcount;
 spinlock_t lock;
} seqlock_t;
static inline __attribute__((no_instrument_function)) unsigned read_seqbegin(const seqlock_t *sl)
{
 return read_seqcount_begin(&sl->seqcount);
}

static inline __attribute__((no_instrument_function)) unsigned read_seqretry(const seqlock_t *sl, unsigned start)
{
 return read_seqcount_retry(&sl->seqcount, start);
}






static inline __attribute__((no_instrument_function)) void write_seqlock(seqlock_t *sl)
{
 spin_lock(&sl->lock);
 write_seqcount_begin(&sl->seqcount);
}

static inline __attribute__((no_instrument_function)) void write_sequnlock(seqlock_t *sl)
{
 write_seqcount_end(&sl->seqcount);
 spin_unlock(&sl->lock);
}

static inline __attribute__((no_instrument_function)) void write_seqlock_bh(seqlock_t *sl)
{
 spin_lock_bh(&sl->lock);
 write_seqcount_begin(&sl->seqcount);
}

static inline __attribute__((no_instrument_function)) void write_sequnlock_bh(seqlock_t *sl)
{
 write_seqcount_end(&sl->seqcount);
 spin_unlock_bh(&sl->lock);
}

static inline __attribute__((no_instrument_function)) void write_seqlock_irq(seqlock_t *sl)
{
 spin_lock_irq(&sl->lock);
 write_seqcount_begin(&sl->seqcount);
}

static inline __attribute__((no_instrument_function)) void write_sequnlock_irq(seqlock_t *sl)
{
 write_seqcount_end(&sl->seqcount);
 spin_unlock_irq(&sl->lock);
}

static inline __attribute__((no_instrument_function)) unsigned long __write_seqlock_irqsave(seqlock_t *sl)
{
 unsigned long flags;

 do { do { ({ unsigned long __dummy; typeof(flags) __dummy2; (void)(&__dummy == &__dummy2); 1; }); flags = _raw_spin_lock_irqsave(spinlock_check(&sl->lock)); } while (0); } while (0);
 write_seqcount_begin(&sl->seqcount);
 return flags;
}




static inline __attribute__((no_instrument_function)) void
write_sequnlock_irqrestore(seqlock_t *sl, unsigned long flags)
{
 write_seqcount_end(&sl->seqcount);
 spin_unlock_irqrestore(&sl->lock, flags);
}






static inline __attribute__((no_instrument_function)) void read_seqlock_excl(seqlock_t *sl)
{
 spin_lock(&sl->lock);
}

static inline __attribute__((no_instrument_function)) void read_sequnlock_excl(seqlock_t *sl)
{
 spin_unlock(&sl->lock);
}
static inline __attribute__((no_instrument_function)) void read_seqbegin_or_lock(seqlock_t *lock, int *seq)
{
 if (!(*seq & 1))
  *seq = read_seqbegin(lock);
 else
  read_seqlock_excl(lock);
}

static inline __attribute__((no_instrument_function)) int need_seqretry(seqlock_t *lock, int seq)
{
 return !(seq & 1) && read_seqretry(lock, seq);
}

static inline __attribute__((no_instrument_function)) void done_seqretry(seqlock_t *lock, int seq)
{
 if (seq & 1)
  read_sequnlock_excl(lock);
}

static inline __attribute__((no_instrument_function)) void read_seqlock_excl_bh(seqlock_t *sl)
{
 spin_lock_bh(&sl->lock);
}

static inline __attribute__((no_instrument_function)) void read_sequnlock_excl_bh(seqlock_t *sl)
{
 spin_unlock_bh(&sl->lock);
}

static inline __attribute__((no_instrument_function)) void read_seqlock_excl_irq(seqlock_t *sl)
{
 spin_lock_irq(&sl->lock);
}

static inline __attribute__((no_instrument_function)) void read_sequnlock_excl_irq(seqlock_t *sl)
{
 spin_unlock_irq(&sl->lock);
}

static inline __attribute__((no_instrument_function)) unsigned long __read_seqlock_excl_irqsave(seqlock_t *sl)
{
 unsigned long flags;

 do { do { ({ unsigned long __dummy; typeof(flags) __dummy2; (void)(&__dummy == &__dummy2); 1; }); flags = _raw_spin_lock_irqsave(spinlock_check(&sl->lock)); } while (0); } while (0);
 return flags;
}




static inline __attribute__((no_instrument_function)) void
read_sequnlock_excl_irqrestore(seqlock_t *sl, unsigned long flags)
{
 spin_unlock_irqrestore(&sl->lock, flags);
}




struct timespec {
 __kernel_time_t tv_sec;
 long tv_nsec;
};


struct timeval {
 __kernel_time_t tv_sec;
 __kernel_suseconds_t tv_usec;
};

struct timezone {
 int tz_minuteswest;
 int tz_dsttime;
};
struct itimerspec {
 struct timespec it_interval;
 struct timespec it_value;
};

struct itimerval {
 struct timeval it_interval;
 struct timeval it_value;
};

typedef __s64 time64_t;
static inline __attribute__((no_instrument_function)) struct timespec timespec64_to_timespec(const struct timespec ts64)
{
 return ts64;
}

static inline __attribute__((no_instrument_function)) struct timespec timespec_to_timespec64(const struct timespec ts)
{
 return ts;
}

extern struct timezone sys_tz;



static inline __attribute__((no_instrument_function)) int timespec_equal(const struct timespec *a,
                                 const struct timespec *b)
{
 return (a->tv_sec == b->tv_sec) && (a->tv_nsec == b->tv_nsec);
}






static inline __attribute__((no_instrument_function)) int timespec_compare(const struct timespec *lhs, const struct timespec *rhs)
{
 if (lhs->tv_sec < rhs->tv_sec)
  return -1;
 if (lhs->tv_sec > rhs->tv_sec)
  return 1;
 return lhs->tv_nsec - rhs->tv_nsec;
}

static inline __attribute__((no_instrument_function)) int timeval_compare(const struct timeval *lhs, const struct timeval *rhs)
{
 if (lhs->tv_sec < rhs->tv_sec)
  return -1;
 if (lhs->tv_sec > rhs->tv_sec)
  return 1;
 return lhs->tv_usec - rhs->tv_usec;
}

extern unsigned long mktime(const unsigned int year, const unsigned int mon,
       const unsigned int day, const unsigned int hour,
       const unsigned int min, const unsigned int sec);

extern void set_normalized_timespec(struct timespec *ts, time_t sec, s64 nsec);






extern struct timespec timespec_add_safe(const struct timespec lhs,
      const struct timespec rhs);


static inline __attribute__((no_instrument_function)) struct timespec timespec_add(struct timespec lhs,
      struct timespec rhs)
{
 struct timespec ts_delta;
 set_normalized_timespec(&ts_delta, lhs.tv_sec + rhs.tv_sec,
    lhs.tv_nsec + rhs.tv_nsec);
 return ts_delta;
}




static inline __attribute__((no_instrument_function)) struct timespec timespec_sub(struct timespec lhs,
      struct timespec rhs)
{
 struct timespec ts_delta;
 set_normalized_timespec(&ts_delta, lhs.tv_sec - rhs.tv_sec,
    lhs.tv_nsec - rhs.tv_nsec);
 return ts_delta;
}




static inline __attribute__((no_instrument_function)) bool timespec_valid(const struct timespec *ts)
{

 if (ts->tv_sec < 0)
  return false;

 if ((unsigned long)ts->tv_nsec >= 1000000000L)
  return false;
 return true;
}

static inline __attribute__((no_instrument_function)) bool timespec_valid_strict(const struct timespec *ts)
{
 if (!timespec_valid(ts))
  return false;

 if ((unsigned long long)ts->tv_sec >= (((s64)~((u64)1 << 63)) / 1000000000L))
  return false;
 return true;
}

extern struct timespec timespec_trunc(struct timespec t, unsigned gran);
struct itimerval;
extern int do_setitimer(int which, struct itimerval *value,
   struct itimerval *ovalue);
extern int do_getitimer(int which, struct itimerval *value);

extern unsigned int alarm_setitimer(unsigned int seconds);

extern long do_utimes(int dfd, const char *filename, struct timespec *times, int flags);

struct tms;
extern void do_sys_times(struct tms *);





struct tm {




 int tm_sec;

 int tm_min;

 int tm_hour;

 int tm_mday;

 int tm_mon;

 long tm_year;

 int tm_wday;

 int tm_yday;
};

void time_to_tm(time_t totalsecs, int offset, struct tm *result);
static inline __attribute__((no_instrument_function)) s64 timespec_to_ns(const struct timespec *ts)
{
 return ((s64) ts->tv_sec * 1000000000L) + ts->tv_nsec;
}
static inline __attribute__((no_instrument_function)) s64 timeval_to_ns(const struct timeval *tv)
{
 return ((s64) tv->tv_sec * 1000000000L) +
  tv->tv_usec * 1000L;
}







extern struct timespec ns_to_timespec(const s64 nsec);







extern struct timeval ns_to_timeval(const s64 nsec);
static inline __attribute__((no_instrument_function)) __attribute__((always_inline)) void timespec_add_ns(struct timespec *a, u64 ns)
{
 a->tv_sec += __iter_div_u64_rem(a->tv_nsec + ns, 1000000000L, &ns);
 a->tv_nsec = ns;
}
extern int overflowuid;
extern int overflowgid;

extern void __bad_uid(void);
extern void __bad_gid(void);
extern int fs_overflowuid;
extern int fs_overflowgid;

struct user_namespace;
extern struct user_namespace init_user_ns;

typedef struct {
 uid_t val;
} kuid_t;


typedef struct {
 gid_t val;
} kgid_t;




static inline __attribute__((no_instrument_function)) uid_t __kuid_val(kuid_t uid)
{
 return uid.val;
}

static inline __attribute__((no_instrument_function)) gid_t __kgid_val(kgid_t gid)
{
 return gid.val;
}







static inline __attribute__((no_instrument_function)) bool uid_eq(kuid_t left, kuid_t right)
{
 return __kuid_val(left) == __kuid_val(right);
}

static inline __attribute__((no_instrument_function)) bool gid_eq(kgid_t left, kgid_t right)
{
 return __kgid_val(left) == __kgid_val(right);
}

static inline __attribute__((no_instrument_function)) bool uid_gt(kuid_t left, kuid_t right)
{
 return __kuid_val(left) > __kuid_val(right);
}

static inline __attribute__((no_instrument_function)) bool gid_gt(kgid_t left, kgid_t right)
{
 return __kgid_val(left) > __kgid_val(right);
}

static inline __attribute__((no_instrument_function)) bool uid_gte(kuid_t left, kuid_t right)
{
 return __kuid_val(left) >= __kuid_val(right);
}

static inline __attribute__((no_instrument_function)) bool gid_gte(kgid_t left, kgid_t right)
{
 return __kgid_val(left) >= __kgid_val(right);
}

static inline __attribute__((no_instrument_function)) bool uid_lt(kuid_t left, kuid_t right)
{
 return __kuid_val(left) < __kuid_val(right);
}

static inline __attribute__((no_instrument_function)) bool gid_lt(kgid_t left, kgid_t right)
{
 return __kgid_val(left) < __kgid_val(right);
}

static inline __attribute__((no_instrument_function)) bool uid_lte(kuid_t left, kuid_t right)
{
 return __kuid_val(left) <= __kuid_val(right);
}

static inline __attribute__((no_instrument_function)) bool gid_lte(kgid_t left, kgid_t right)
{
 return __kgid_val(left) <= __kgid_val(right);
}

static inline __attribute__((no_instrument_function)) bool uid_valid(kuid_t uid)
{
 return !uid_eq(uid, (kuid_t){ -1 });
}

static inline __attribute__((no_instrument_function)) bool gid_valid(kgid_t gid)
{
 return !gid_eq(gid, (kgid_t){ -1 });
}
static inline __attribute__((no_instrument_function)) kuid_t make_kuid(struct user_namespace *from, uid_t uid)
{
 return (kuid_t){ uid };
}

static inline __attribute__((no_instrument_function)) kgid_t make_kgid(struct user_namespace *from, gid_t gid)
{
 return (kgid_t){ gid };
}

static inline __attribute__((no_instrument_function)) uid_t from_kuid(struct user_namespace *to, kuid_t kuid)
{
 return __kuid_val(kuid);
}

static inline __attribute__((no_instrument_function)) gid_t from_kgid(struct user_namespace *to, kgid_t kgid)
{
 return __kgid_val(kgid);
}

static inline __attribute__((no_instrument_function)) uid_t from_kuid_munged(struct user_namespace *to, kuid_t kuid)
{
 uid_t uid = from_kuid(to, kuid);
 if (uid == (uid_t)-1)
  uid = overflowuid;
 return uid;
}

static inline __attribute__((no_instrument_function)) gid_t from_kgid_munged(struct user_namespace *to, kgid_t kgid)
{
 gid_t gid = from_kgid(to, kgid);
 if (gid == (gid_t)-1)
  gid = overflowgid;
 return gid;
}

static inline __attribute__((no_instrument_function)) bool kuid_has_mapping(struct user_namespace *ns, kuid_t uid)
{
 return true;
}

static inline __attribute__((no_instrument_function)) bool kgid_has_mapping(struct user_namespace *ns, kgid_t gid)
{
 return true;
}

struct kstat {
 u64 ino;
 dev_t dev;
 umode_t mode;
 unsigned int nlink;
 kuid_t uid;
 kgid_t gid;
 dev_t rdev;
 loff_t size;
 struct timespec atime;
 struct timespec mtime;
 struct timespec ctime;
 unsigned long blksize;
 unsigned long long blocks;
};










struct page;

extern void dump_page(struct page *page, const char *reason);
extern void dump_page_badflags(struct page *page, const char *reason,
          unsigned long badflags);

typedef struct __wait_queue wait_queue_t;
typedef int (*wait_queue_func_t)(wait_queue_t *wait, unsigned mode, int flags, void *key);
int default_wake_function(wait_queue_t *wait, unsigned mode, int flags, void *key);

struct __wait_queue {
 unsigned int flags;

 void *private;
 wait_queue_func_t func;
 struct list_head task_list;
};

struct wait_bit_key {
 void *flags;
 int bit_nr;

 unsigned long private;
};

struct wait_bit_queue {
 struct wait_bit_key key;
 wait_queue_t wait;
};

struct __wait_queue_head {
 spinlock_t lock;
 struct list_head task_list;
};
typedef struct __wait_queue_head wait_queue_head_t;

struct task_struct;
extern void __init_waitqueue_head(wait_queue_head_t *q, const char *name, struct lock_class_key *);
static inline __attribute__((no_instrument_function)) void init_waitqueue_entry(wait_queue_t *q, struct task_struct *p)
{
 q->flags = 0;
 q->private = p;
 q->func = default_wake_function;
}

static inline __attribute__((no_instrument_function)) void
init_waitqueue_func_entry(wait_queue_t *q, wait_queue_func_t func)
{
 q->flags = 0;
 q->private = ((void *)0);
 q->func = func;
}

static inline __attribute__((no_instrument_function)) int waitqueue_active(wait_queue_head_t *q)
{
 return !list_empty(&q->task_list);
}

extern void add_wait_queue(wait_queue_head_t *q, wait_queue_t *wait);
extern void add_wait_queue_exclusive(wait_queue_head_t *q, wait_queue_t *wait);
extern void remove_wait_queue(wait_queue_head_t *q, wait_queue_t *wait);

static inline __attribute__((no_instrument_function)) void __add_wait_queue(wait_queue_head_t *head, wait_queue_t *new)
{
 list_add(&new->task_list, &head->task_list);
}




static inline __attribute__((no_instrument_function)) void
__add_wait_queue_exclusive(wait_queue_head_t *q, wait_queue_t *wait)
{
 wait->flags |= 0x01;
 __add_wait_queue(q, wait);
}

static inline __attribute__((no_instrument_function)) void __add_wait_queue_tail(wait_queue_head_t *head,
      wait_queue_t *new)
{
 list_add_tail(&new->task_list, &head->task_list);
}

static inline __attribute__((no_instrument_function)) void
__add_wait_queue_tail_exclusive(wait_queue_head_t *q, wait_queue_t *wait)
{
 wait->flags |= 0x01;
 __add_wait_queue_tail(q, wait);
}

static inline __attribute__((no_instrument_function)) void
__remove_wait_queue(wait_queue_head_t *head, wait_queue_t *old)
{
 list_del(&old->task_list);
}

typedef int wait_bit_action_f(struct wait_bit_key *);
void __wake_up(wait_queue_head_t *q, unsigned int mode, int nr, void *key);
void __wake_up_locked_key(wait_queue_head_t *q, unsigned int mode, void *key);
void __wake_up_sync_key(wait_queue_head_t *q, unsigned int mode, int nr, void *key);
void __wake_up_locked(wait_queue_head_t *q, unsigned int mode, int nr);
void __wake_up_sync(wait_queue_head_t *q, unsigned int mode, int nr);
void __wake_up_bit(wait_queue_head_t *, void *, int);
int __wait_on_bit(wait_queue_head_t *, struct wait_bit_queue *, wait_bit_action_f *, unsigned);
int __wait_on_bit_lock(wait_queue_head_t *, struct wait_bit_queue *, wait_bit_action_f *, unsigned);
void wake_up_bit(void *, int);
void wake_up_atomic_t(atomic_t *);
int out_of_line_wait_on_bit(void *, int, wait_bit_action_f *, unsigned);
int out_of_line_wait_on_bit_lock(void *, int, wait_bit_action_f *, unsigned);
int out_of_line_wait_on_atomic_t(atomic_t *, int (*)(atomic_t *), unsigned);
wait_queue_head_t *bit_waitqueue(void *, int);
void prepare_to_wait(wait_queue_head_t *q, wait_queue_t *wait, int state);
void prepare_to_wait_exclusive(wait_queue_head_t *q, wait_queue_t *wait, int state);
long prepare_to_wait_event(wait_queue_head_t *q, wait_queue_t *wait, int state);
void finish_wait(wait_queue_head_t *q, wait_queue_t *wait);
void abort_exclusive_wait(wait_queue_head_t *q, wait_queue_t *wait, unsigned int mode, void *key);
int autoremove_wake_function(wait_queue_t *wait, unsigned mode, int sync, void *key);
int wake_bit_function(wait_queue_t *wait, unsigned mode, int sync, void *key);
extern int bit_wait(struct wait_bit_key *);
extern int bit_wait_io(struct wait_bit_key *);
static inline __attribute__((no_instrument_function)) int
wait_on_bit(void *word, int bit, unsigned mode)
{
 if (!(__builtin_constant_p((bit)) ? constant_test_bit((bit), (word)) : variable_test_bit((bit), (word))))
  return 0;
 return out_of_line_wait_on_bit(word, bit,
           bit_wait,
           mode);
}
static inline __attribute__((no_instrument_function)) int
wait_on_bit_io(void *word, int bit, unsigned mode)
{
 if (!(__builtin_constant_p((bit)) ? constant_test_bit((bit), (word)) : variable_test_bit((bit), (word))))
  return 0;
 return out_of_line_wait_on_bit(word, bit,
           bit_wait_io,
           mode);
}
static inline __attribute__((no_instrument_function)) int
wait_on_bit_action(void *word, int bit, wait_bit_action_f *action, unsigned mode)
{
 if (!(__builtin_constant_p((bit)) ? constant_test_bit((bit), (word)) : variable_test_bit((bit), (word))))
  return 0;
 return out_of_line_wait_on_bit(word, bit, action, mode);
}
static inline __attribute__((no_instrument_function)) int
wait_on_bit_lock(void *word, int bit, unsigned mode)
{
 if (!test_and_set_bit(bit, word))
  return 0;
 return out_of_line_wait_on_bit_lock(word, bit, bit_wait, mode);
}
static inline __attribute__((no_instrument_function)) int
wait_on_bit_lock_io(void *word, int bit, unsigned mode)
{
 if (!test_and_set_bit(bit, word))
  return 0;
 return out_of_line_wait_on_bit_lock(word, bit, bit_wait_io, mode);
}
static inline __attribute__((no_instrument_function)) int
wait_on_bit_lock_action(void *word, int bit, wait_bit_action_f *action, unsigned mode)
{
 if (!test_and_set_bit(bit, word))
  return 0;
 return out_of_line_wait_on_bit_lock(word, bit, action, mode);
}
static inline __attribute__((no_instrument_function))
int wait_on_atomic_t(atomic_t *val, int (*action)(atomic_t *), unsigned mode)
{
 if (atomic_read(val) == 0)
  return 0;
 return out_of_line_wait_on_atomic_t(val, action, mode);
}





typedef struct { unsigned long bits[((((1 << 6)) + (8 * sizeof(long)) - 1) / (8 * sizeof(long)))]; } nodemask_t;
extern nodemask_t _unused_nodemask_arg_;
static inline __attribute__((no_instrument_function)) __attribute__((always_inline)) void __node_set(int node, volatile nodemask_t *dstp)
{
 set_bit(node, dstp->bits);
}


static inline __attribute__((no_instrument_function)) void __node_clear(int node, volatile nodemask_t *dstp)
{
 clear_bit(node, dstp->bits);
}


static inline __attribute__((no_instrument_function)) void __nodes_setall(nodemask_t *dstp, int nbits)
{
 bitmap_fill(dstp->bits, nbits);
}


static inline __attribute__((no_instrument_function)) void __nodes_clear(nodemask_t *dstp, int nbits)
{
 bitmap_zero(dstp->bits, nbits);
}






static inline __attribute__((no_instrument_function)) int __node_test_and_set(int node, nodemask_t *addr)
{
 return test_and_set_bit(node, addr->bits);
}



static inline __attribute__((no_instrument_function)) void __nodes_and(nodemask_t *dstp, const nodemask_t *src1p,
     const nodemask_t *src2p, int nbits)
{
 bitmap_and(dstp->bits, src1p->bits, src2p->bits, nbits);
}



static inline __attribute__((no_instrument_function)) void __nodes_or(nodemask_t *dstp, const nodemask_t *src1p,
     const nodemask_t *src2p, int nbits)
{
 bitmap_or(dstp->bits, src1p->bits, src2p->bits, nbits);
}



static inline __attribute__((no_instrument_function)) void __nodes_xor(nodemask_t *dstp, const nodemask_t *src1p,
     const nodemask_t *src2p, int nbits)
{
 bitmap_xor(dstp->bits, src1p->bits, src2p->bits, nbits);
}



static inline __attribute__((no_instrument_function)) void __nodes_andnot(nodemask_t *dstp, const nodemask_t *src1p,
     const nodemask_t *src2p, int nbits)
{
 bitmap_andnot(dstp->bits, src1p->bits, src2p->bits, nbits);
}



static inline __attribute__((no_instrument_function)) void __nodes_complement(nodemask_t *dstp,
     const nodemask_t *srcp, int nbits)
{
 bitmap_complement(dstp->bits, srcp->bits, nbits);
}



static inline __attribute__((no_instrument_function)) int __nodes_equal(const nodemask_t *src1p,
     const nodemask_t *src2p, int nbits)
{
 return bitmap_equal(src1p->bits, src2p->bits, nbits);
}



static inline __attribute__((no_instrument_function)) int __nodes_intersects(const nodemask_t *src1p,
     const nodemask_t *src2p, int nbits)
{
 return bitmap_intersects(src1p->bits, src2p->bits, nbits);
}



static inline __attribute__((no_instrument_function)) int __nodes_subset(const nodemask_t *src1p,
     const nodemask_t *src2p, int nbits)
{
 return bitmap_subset(src1p->bits, src2p->bits, nbits);
}


static inline __attribute__((no_instrument_function)) int __nodes_empty(const nodemask_t *srcp, int nbits)
{
 return bitmap_empty(srcp->bits, nbits);
}


static inline __attribute__((no_instrument_function)) int __nodes_full(const nodemask_t *srcp, int nbits)
{
 return bitmap_full(srcp->bits, nbits);
}


static inline __attribute__((no_instrument_function)) int __nodes_weight(const nodemask_t *srcp, int nbits)
{
 return bitmap_weight(srcp->bits, nbits);
}



static inline __attribute__((no_instrument_function)) void __nodes_shift_right(nodemask_t *dstp,
     const nodemask_t *srcp, int n, int nbits)
{
 bitmap_shift_right(dstp->bits, srcp->bits, n, nbits);
}



static inline __attribute__((no_instrument_function)) void __nodes_shift_left(nodemask_t *dstp,
     const nodemask_t *srcp, int n, int nbits)
{
 bitmap_shift_left(dstp->bits, srcp->bits, n, nbits);
}





static inline __attribute__((no_instrument_function)) int __first_node(const nodemask_t *srcp)
{
 return ({ int __min1 = ((1 << 6)); int __min2 = (find_first_bit(srcp->bits, (1 << 6))); __min1 < __min2 ? __min1: __min2; });
}


static inline __attribute__((no_instrument_function)) int __next_node(int n, const nodemask_t *srcp)
{
 return ({ int __min1 = ((1 << 6)); int __min2 = (find_next_bit(srcp->bits, (1 << 6), n+1)); __min1 < __min2 ? __min1: __min2; });
}

static inline __attribute__((no_instrument_function)) void init_nodemask_of_node(nodemask_t *mask, int node)
{
 __nodes_clear(&(*mask), (1 << 6));
 __node_set((node), &(*mask));
}
static inline __attribute__((no_instrument_function)) int __first_unset_node(const nodemask_t *maskp)
{
 return ({ int __min1 = ((1 << 6)); int __min2 = (find_first_zero_bit(maskp->bits, (1 << 6))); __min1 < __min2 ? __min1: __min2; })
                                                  ;
}
static inline __attribute__((no_instrument_function)) int __nodemask_scnprintf(char *buf, int len,
     const nodemask_t *srcp, int nbits)
{
 return bitmap_scnprintf(buf, len, srcp->bits, nbits);
}



static inline __attribute__((no_instrument_function)) int __nodemask_parse_user(const char *buf, int len,
     nodemask_t *dstp, int nbits)
{
 return bitmap_parse_user(buf, len, dstp->bits, nbits);
}



static inline __attribute__((no_instrument_function)) int __nodelist_scnprintf(char *buf, int len,
     const nodemask_t *srcp, int nbits)
{
 return bitmap_scnlistprintf(buf, len, srcp->bits, nbits);
}


static inline __attribute__((no_instrument_function)) int __nodelist_parse(const char *buf, nodemask_t *dstp, int nbits)
{
 return bitmap_parselist(buf, dstp->bits, nbits);
}



static inline __attribute__((no_instrument_function)) int __node_remap(int oldbit,
  const nodemask_t *oldp, const nodemask_t *newp, int nbits)
{
 return bitmap_bitremap(oldbit, oldp->bits, newp->bits, nbits);
}



static inline __attribute__((no_instrument_function)) void __nodes_remap(nodemask_t *dstp, const nodemask_t *srcp,
  const nodemask_t *oldp, const nodemask_t *newp, int nbits)
{
 bitmap_remap(dstp->bits, srcp->bits, oldp->bits, newp->bits, nbits);
}



static inline __attribute__((no_instrument_function)) void __nodes_onto(nodemask_t *dstp, const nodemask_t *origp,
  const nodemask_t *relmapp, int nbits)
{
 bitmap_onto(dstp->bits, origp->bits, relmapp->bits, nbits);
}



static inline __attribute__((no_instrument_function)) void __nodes_fold(nodemask_t *dstp, const nodemask_t *origp,
  int sz, int nbits)
{
 bitmap_fold(dstp->bits, origp->bits, sz, nbits);
}
enum node_states {
 N_POSSIBLE,
 N_ONLINE,
 N_NORMAL_MEMORY,



 N_HIGH_MEMORY = N_NORMAL_MEMORY,




 N_MEMORY = N_HIGH_MEMORY,

 N_CPU,
 NR_NODE_STATES
};






extern nodemask_t node_states[NR_NODE_STATES];


static inline __attribute__((no_instrument_function)) int node_state(int node, enum node_states state)
{
 return (__builtin_constant_p(((node))) ? constant_test_bit(((node)), ((node_states[state]).bits)) : variable_test_bit(((node)), ((node_states[state]).bits)));
}

static inline __attribute__((no_instrument_function)) void node_set_state(int node, enum node_states state)
{
 __node_set(node, &node_states[state]);
}

static inline __attribute__((no_instrument_function)) void node_clear_state(int node, enum node_states state)
{
 __node_clear(node, &node_states[state]);
}

static inline __attribute__((no_instrument_function)) int num_node_state(enum node_states state)
{
 return __nodes_weight(&(node_states[state]), (1 << 6));
}






static inline __attribute__((no_instrument_function)) int next_online_node(int nid)
{
 return __next_node((nid), &(node_states[N_ONLINE]));
}
static inline __attribute__((no_instrument_function)) int next_memory_node(int nid)
{
 return __next_node((nid), &(node_states[N_MEMORY]));
}

extern int nr_node_ids;
extern int nr_online_nodes;

static inline __attribute__((no_instrument_function)) void node_set_online(int nid)
{
 node_set_state(nid, N_ONLINE);
 nr_online_nodes = num_node_state(N_ONLINE);
}

static inline __attribute__((no_instrument_function)) void node_set_offline(int nid)
{
 node_clear_state(nid, N_ONLINE);
 nr_online_nodes = num_node_state(N_ONLINE);
}
extern int node_random(const nodemask_t *maskp);
struct nodemask_scratch {
 nodemask_t mask1;
 nodemask_t mask2;
};
enum pageblock_bits {
 PB_migrate,
 PB_migrate_end = PB_migrate + 3 - 1,

 PB_migrate_skip,





 NR_PAGEBLOCK_BITS
};
struct page;

unsigned long get_pfnblock_flags_mask(struct page *page,
    unsigned long pfn,
    unsigned long end_bitidx,
    unsigned long mask);

void set_pfnblock_flags_mask(struct page *page,
    unsigned long flags,
    unsigned long pfn,
    unsigned long end_bitidx,
    unsigned long mask);




enum {
 MIGRATE_UNMOVABLE,
 MIGRATE_RECLAIMABLE,
 MIGRATE_MOVABLE,
 MIGRATE_PCPTYPES,
 MIGRATE_RESERVE = MIGRATE_PCPTYPES,
 MIGRATE_ISOLATE,

 MIGRATE_TYPES
};
extern int page_group_by_mobility_disabled;
static inline __attribute__((no_instrument_function)) int get_pfnblock_migratetype(struct page *page, unsigned long pfn)
{
 ((void)sizeof(char[1 - 2*!!(PB_migrate_end - PB_migrate != 2)]));
 return get_pfnblock_flags_mask(page, pfn, PB_migrate_end,
     ((1UL << (PB_migrate_end - PB_migrate + 1)) - 1));
}

struct free_area {
 struct list_head free_list[MIGRATE_TYPES];
 unsigned long nr_free;
};

struct pglist_data;
struct zone_padding {
 char x[0];
} __attribute__((__aligned__(1 << (6))));





enum zone_stat_item {

 NR_FREE_PAGES,
 NR_ALLOC_BATCH,
 NR_LRU_BASE,
 NR_INACTIVE_ANON = NR_LRU_BASE,
 NR_ACTIVE_ANON,
 NR_INACTIVE_FILE,
 NR_ACTIVE_FILE,
 NR_UNEVICTABLE,
 NR_MLOCK,
 NR_ANON_PAGES,
 NR_FILE_MAPPED,

 NR_FILE_PAGES,
 NR_FILE_DIRTY,
 NR_WRITEBACK,
 NR_SLAB_RECLAIMABLE,
 NR_SLAB_UNRECLAIMABLE,
 NR_PAGETABLE,
 NR_KERNEL_STACK,

 NR_UNSTABLE_NFS,
 NR_BOUNCE,
 NR_VMSCAN_WRITE,
 NR_VMSCAN_IMMEDIATE,
 NR_WRITEBACK_TEMP,
 NR_ISOLATED_ANON,
 NR_ISOLATED_FILE,
 NR_SHMEM,
 NR_DIRTIED,
 NR_WRITTEN,
 NR_PAGES_SCANNED,

 NUMA_HIT,
 NUMA_MISS,
 NUMA_FOREIGN,
 NUMA_INTERLEAVE_HIT,
 NUMA_LOCAL,
 NUMA_OTHER,

 WORKINGSET_REFAULT,
 WORKINGSET_ACTIVATE,
 WORKINGSET_NODERECLAIM,
 NR_ANON_TRANSPARENT_HUGEPAGES,
 NR_FREE_CMA_PAGES,
 NR_VM_ZONE_STAT_ITEMS };
enum lru_list {
 LRU_INACTIVE_ANON = 0,
 LRU_ACTIVE_ANON = 0 + 1,
 LRU_INACTIVE_FILE = 0 + 2,
 LRU_ACTIVE_FILE = 0 + 2 + 1,
 LRU_UNEVICTABLE,
 NR_LRU_LISTS
};





static inline __attribute__((no_instrument_function)) int is_file_lru(enum lru_list lru)
{
 return (lru == LRU_INACTIVE_FILE || lru == LRU_ACTIVE_FILE);
}

static inline __attribute__((no_instrument_function)) int is_active_lru(enum lru_list lru)
{
 return (lru == LRU_ACTIVE_ANON || lru == LRU_ACTIVE_FILE);
}

static inline __attribute__((no_instrument_function)) int is_unevictable_lru(enum lru_list lru)
{
 return (lru == LRU_UNEVICTABLE);
}

struct zone_reclaim_stat {
 unsigned long recent_rotated[2];
 unsigned long recent_scanned[2];
};

struct lruvec {
 struct list_head lists[NR_LRU_LISTS];
 struct zone_reclaim_stat reclaim_stat;



};
typedef unsigned isolate_mode_t;

enum zone_watermarks {
 WMARK_MIN,
 WMARK_LOW,
 WMARK_HIGH,
 NR_WMARK
};





struct per_cpu_pages {
 int count;
 int high;
 int batch;


 struct list_head lists[MIGRATE_PCPTYPES];
};

struct per_cpu_pageset {
 struct per_cpu_pages pcp;

 s8 expire;


 s8 stat_threshold;
 s8 vm_stat_diff[NR_VM_ZONE_STAT_ITEMS];

};



enum zone_type {
 ZONE_DMA,







 ZONE_DMA32,






 ZONE_NORMAL,
 ZONE_MOVABLE,
 __MAX_NR_ZONES
};



struct zone {



 unsigned long watermark[NR_WMARK];
 long lowmem_reserve[4];


 int node;






 unsigned int inactive_ratio;

 struct pglist_data *zone_pgdat;
 struct per_cpu_pageset *pageset;





 unsigned long dirty_balance_reserve;
 unsigned long min_unmapped_pages;
 unsigned long min_slab_pages;



 unsigned long zone_start_pfn;
 unsigned long managed_pages;
 unsigned long spanned_pages;
 unsigned long present_pages;

 const char *name;





 int nr_migrate_reserve_block;



 seqlock_t span_seqlock;
 wait_queue_head_t *wait_table;
 unsigned long wait_table_hash_nr_entries;
 unsigned long wait_table_bits;

 struct zone_padding _pad1_;


 spinlock_t lock;


 struct free_area free_area[11];


 unsigned long flags;

 struct zone_padding _pad2_;




 spinlock_t lru_lock;
 struct lruvec lruvec;


 atomic_long_t inactive_age;






 unsigned long percpu_drift_mark;



 unsigned long compact_cached_free_pfn;

 unsigned long compact_cached_migrate_pfn[2];
 unsigned int compact_considered;
 unsigned int compact_defer_shift;
 int compact_order_failed;




 bool compact_blockskip_flush;


 struct zone_padding _pad3_;

 atomic_long_t vm_stat[NR_VM_ZONE_STAT_ITEMS];
} __attribute__((__aligned__(1 << (6))));

typedef enum {
 ZONE_RECLAIM_LOCKED,
 ZONE_OOM_LOCKED,
 ZONE_CONGESTED,


 ZONE_TAIL_LRU_DIRTY,



 ZONE_WRITEBACK,


 ZONE_FAIR_DEPLETED,
} zone_flags_t;

static inline __attribute__((no_instrument_function)) void zone_set_flag(struct zone *zone, zone_flags_t flag)
{
 set_bit(flag, &zone->flags);
}

static inline __attribute__((no_instrument_function)) int zone_test_and_set_flag(struct zone *zone, zone_flags_t flag)
{
 return test_and_set_bit(flag, &zone->flags);
}

static inline __attribute__((no_instrument_function)) void zone_clear_flag(struct zone *zone, zone_flags_t flag)
{
 clear_bit(flag, &zone->flags);
}

static inline __attribute__((no_instrument_function)) int zone_is_reclaim_congested(const struct zone *zone)
{
 return (__builtin_constant_p((ZONE_CONGESTED)) ? constant_test_bit((ZONE_CONGESTED), (&zone->flags)) : variable_test_bit((ZONE_CONGESTED), (&zone->flags)));
}

static inline __attribute__((no_instrument_function)) int zone_is_reclaim_dirty(const struct zone *zone)
{
 return (__builtin_constant_p((ZONE_TAIL_LRU_DIRTY)) ? constant_test_bit((ZONE_TAIL_LRU_DIRTY), (&zone->flags)) : variable_test_bit((ZONE_TAIL_LRU_DIRTY), (&zone->flags)));
}

static inline __attribute__((no_instrument_function)) int zone_is_reclaim_writeback(const struct zone *zone)
{
 return (__builtin_constant_p((ZONE_WRITEBACK)) ? constant_test_bit((ZONE_WRITEBACK), (&zone->flags)) : variable_test_bit((ZONE_WRITEBACK), (&zone->flags)));
}

static inline __attribute__((no_instrument_function)) int zone_is_reclaim_locked(const struct zone *zone)
{
 return (__builtin_constant_p((ZONE_RECLAIM_LOCKED)) ? constant_test_bit((ZONE_RECLAIM_LOCKED), (&zone->flags)) : variable_test_bit((ZONE_RECLAIM_LOCKED), (&zone->flags)));
}

static inline __attribute__((no_instrument_function)) int zone_is_fair_depleted(const struct zone *zone)
{
 return (__builtin_constant_p((ZONE_FAIR_DEPLETED)) ? constant_test_bit((ZONE_FAIR_DEPLETED), (&zone->flags)) : variable_test_bit((ZONE_FAIR_DEPLETED), (&zone->flags)));
}

static inline __attribute__((no_instrument_function)) int zone_is_oom_locked(const struct zone *zone)
{
 return (__builtin_constant_p((ZONE_OOM_LOCKED)) ? constant_test_bit((ZONE_OOM_LOCKED), (&zone->flags)) : variable_test_bit((ZONE_OOM_LOCKED), (&zone->flags)));
}

static inline __attribute__((no_instrument_function)) unsigned long zone_end_pfn(const struct zone *zone)
{
 return zone->zone_start_pfn + zone->spanned_pages;
}

static inline __attribute__((no_instrument_function)) bool zone_spans_pfn(const struct zone *zone, unsigned long pfn)
{
 return zone->zone_start_pfn <= pfn && pfn < zone_end_pfn(zone);
}

static inline __attribute__((no_instrument_function)) bool zone_is_initialized(struct zone *zone)
{
 return !!zone->wait_table;
}

static inline __attribute__((no_instrument_function)) bool zone_is_empty(struct zone *zone)
{
 return zone->spanned_pages == 0;
}
struct zonelist_cache {
 unsigned short z_to_n[((1 << 6) * 4)];
 unsigned long fullzones[(((((1 << 6) * 4)) + (8 * sizeof(long)) - 1) / (8 * sizeof(long)))];
 unsigned long last_full_zap;
};
struct zoneref {
 struct zone *zone;
 int zone_idx;
};
struct zonelist {
 struct zonelist_cache *zlcache_ptr;
 struct zoneref _zonerefs[((1 << 6) * 4) + 1];

 struct zonelist_cache zlcache;

};


struct node_active_region {
 unsigned long start_pfn;
 unsigned long end_pfn;
 int nid;
};




extern struct page *mem_map;
struct bootmem_data;
typedef struct pglist_data {
 struct zone node_zones[4];
 struct zonelist node_zonelists[2];
 int nr_zones;
 spinlock_t node_size_lock;

 unsigned long node_start_pfn;
 unsigned long node_present_pages;
 unsigned long node_spanned_pages;

 int node_id;
 wait_queue_head_t kswapd_wait;
 wait_queue_head_t pfmemalloc_wait;
 struct task_struct *kswapd;

 int kswapd_max_order;
 enum zone_type classzone_idx;
} pg_data_t;
static inline __attribute__((no_instrument_function)) unsigned long pgdat_end_pfn(pg_data_t *pgdat)
{
 return pgdat->node_start_pfn + pgdat->node_spanned_pages;
}

static inline __attribute__((no_instrument_function)) bool pgdat_is_empty(pg_data_t *pgdat)
{
 return !pgdat->node_start_pfn && !pgdat->node_spanned_pages;
}





struct optimistic_spin_queue {




 atomic_t tail;
};




static inline __attribute__((no_instrument_function)) void osq_lock_init(struct optimistic_spin_queue *lock)
{
 atomic_set(&lock->tail, (0));
}
struct mutex {

 atomic_t count;
 spinlock_t wait_lock;
 struct list_head wait_list;

 struct task_struct *owner;


 struct optimistic_spin_queue osq;
};





struct mutex_waiter {
 struct list_head list;
 struct task_struct *task;



};
static inline __attribute__((no_instrument_function)) void mutex_destroy(struct mutex *lock) {}
extern void __mutex_init(struct mutex *lock, const char *name,
    struct lock_class_key *key);







static inline __attribute__((no_instrument_function)) int mutex_is_locked(struct mutex *lock)
{
 return atomic_read(&lock->count) != 1;
}
extern void mutex_lock(struct mutex *lock);
extern int mutex_lock_interruptible(struct mutex *lock);
extern int mutex_lock_killable(struct mutex *lock);
extern int mutex_trylock(struct mutex *lock);
extern void mutex_unlock(struct mutex *lock);

extern int atomic_dec_and_mutex_lock(atomic_t *cnt, struct mutex *lock);
struct rw_semaphore;





struct rw_semaphore {
 long count;
 struct list_head wait_list;
 raw_spinlock_t wait_lock;

 struct optimistic_spin_queue osq;




 struct task_struct *owner;




};

extern struct rw_semaphore *rwsem_down_read_failed(struct rw_semaphore *sem);
extern struct rw_semaphore *rwsem_down_write_failed(struct rw_semaphore *sem);
extern struct rw_semaphore *rwsem_wake(struct rw_semaphore *);
extern struct rw_semaphore *rwsem_downgrade_wake(struct rw_semaphore *sem);


static inline __attribute__((no_instrument_function)) void __down_read(struct rw_semaphore *sem)
{
 asm volatile("# beginning down_read\n\t"
       ".pushsection .smp_locks,\"a\"\n" ".balign 4\n" ".long 671f - .\n" ".popsection\n" "671:" "\n\tlock; " " " "incq" " " "(%1)\n\t"

       "  jns        1f\n"
       "  call call_rwsem_down_read_failed\n"
       "1:\n\t"
       "# ending down_read\n\t"
       : "+m" (sem->count)
       : "a" (sem)
       : "memory", "cc");
}




static inline __attribute__((no_instrument_function)) int __down_read_trylock(struct rw_semaphore *sem)
{
 long result, tmp;
 asm volatile("# beginning __down_read_trylock\n\t"
       "  mov          %0,%1\n\t"
       "1:\n\t"
       "  mov          %1,%2\n\t"
       "  add          %3,%2\n\t"
       "  jle	     2f\n\t"
       ".pushsection .smp_locks,\"a\"\n" ".balign 4\n" ".long 671f - .\n" ".popsection\n" "671:" "\n\tlock; " "  cmpxchg  %2,%0\n\t"
       "  jnz	     1b\n\t"
       "2:\n\t"
       "# ending __down_read_trylock\n\t"
       : "+m" (sem->count), "=&a" (result), "=&r" (tmp)
       : "i" (0x00000001L)
       : "memory", "cc");
 return result >= 0 ? 1 : 0;
}




static inline __attribute__((no_instrument_function)) void __down_write_nested(struct rw_semaphore *sem, int subclass)
{
 long tmp;
 asm volatile("# beginning down_write\n\t"
       ".pushsection .smp_locks,\"a\"\n" ".balign 4\n" ".long 671f - .\n" ".popsection\n" "671:" "\n\tlock; " "  xadd      %1,(%2)\n\t"

       "  test " " " "%k1" " " "," " " "%k1" " " "\n\t"

       "  jz        1f\n"
       "  call call_rwsem_down_write_failed\n"
       "1:\n"
       "# ending down_write"
       : "+m" (sem->count), "=d" (tmp)
       : "a" (sem), "1" (((-0xffffffffL -1) + 0x00000001L))
       : "memory", "cc");
}

static inline __attribute__((no_instrument_function)) void __down_write(struct rw_semaphore *sem)
{
 __down_write_nested(sem, 0);
}




static inline __attribute__((no_instrument_function)) int __down_write_trylock(struct rw_semaphore *sem)
{
 long result, tmp;
 asm volatile("# beginning __down_write_trylock\n\t"
       "  mov          %0,%1\n\t"
       "1:\n\t"
       "  test " " " "%k1" " " "," " " "%k1" " " "\n\t"

       "  jnz          2f\n\t"
       "  mov          %1,%2\n\t"
       "  add          %3,%2\n\t"
       ".pushsection .smp_locks,\"a\"\n" ".balign 4\n" ".long 671f - .\n" ".popsection\n" "671:" "\n\tlock; " "  cmpxchg  %2,%0\n\t"
       "  jnz	     1b\n\t"
       "2:\n\t"
       "  sete         %b1\n\t"
       "  movzbl       %b1, %k1\n\t"
       "# ending __down_write_trylock\n\t"
       : "+m" (sem->count), "=&a" (result), "=&r" (tmp)
       : "er" (((-0xffffffffL -1) + 0x00000001L))
       : "memory", "cc");
 return result;
}




static inline __attribute__((no_instrument_function)) void __up_read(struct rw_semaphore *sem)
{
 long tmp;
 asm volatile("# beginning __up_read\n\t"
       ".pushsection .smp_locks,\"a\"\n" ".balign 4\n" ".long 671f - .\n" ".popsection\n" "671:" "\n\tlock; " "  xadd      %1,(%2)\n\t"

       "  jns        1f\n\t"
       "  call call_rwsem_wake\n"
       "1:\n"
       "# ending __up_read\n"
       : "+m" (sem->count), "=d" (tmp)
       : "a" (sem), "1" (-0x00000001L)
       : "memory", "cc");
}




static inline __attribute__((no_instrument_function)) void __up_write(struct rw_semaphore *sem)
{
 long tmp;
 asm volatile("# beginning __up_write\n\t"
       ".pushsection .smp_locks,\"a\"\n" ".balign 4\n" ".long 671f - .\n" ".popsection\n" "671:" "\n\tlock; " "  xadd      %1,(%2)\n\t"

       "  jns        1f\n\t"
       "  call call_rwsem_wake\n"
       "1:\n\t"
       "# ending __up_write\n"
       : "+m" (sem->count), "=d" (tmp)
       : "a" (sem), "1" (-((-0xffffffffL -1) + 0x00000001L))
       : "memory", "cc");
}




static inline __attribute__((no_instrument_function)) void __downgrade_write(struct rw_semaphore *sem)
{
 asm volatile("# beginning __downgrade_write\n\t"
       ".pushsection .smp_locks,\"a\"\n" ".balign 4\n" ".long 671f - .\n" ".popsection\n" "671:" "\n\tlock; " " " "addq" " " "%2,(%1)\n\t"




       "  jns       1f\n\t"
       "  call call_rwsem_downgrade_wake\n"
       "1:\n\t"
       "# ending __downgrade_write\n"
       : "+m" (sem->count)
       : "a" (sem), "er" (-(-0xffffffffL -1))
       : "memory", "cc");
}




static inline __attribute__((no_instrument_function)) void rwsem_atomic_add(long delta, struct rw_semaphore *sem)
{
 asm volatile(".pushsection .smp_locks,\"a\"\n" ".balign 4\n" ".long 671f - .\n" ".popsection\n" "671:" "\n\tlock; " " " "addq" " " "%1,%0"
       : "+m" (sem->count)
       : "er" (delta));
}




static inline __attribute__((no_instrument_function)) long rwsem_atomic_update(long delta, struct rw_semaphore *sem)
{
 return delta + ({ __typeof__ (*(((&sem->count)))) __ret = (((delta))); switch (sizeof(*(((&sem->count))))) { case 1: asm volatile (".pushsection .smp_locks,\"a\"\n" ".balign 4\n" ".long 671f - .\n" ".popsection\n" "671:" "\n\tlock; " "xadd" "b %b0, %1\n" : "+q" (__ret), "+m" (*(((&sem->count)))) : : "memory", "cc"); break; case 2: asm volatile (".pushsection .smp_locks,\"a\"\n" ".balign 4\n" ".long 671f - .\n" ".popsection\n" "671:" "\n\tlock; " "xadd" "w %w0, %1\n" : "+r" (__ret), "+m" (*(((&sem->count)))) : : "memory", "cc"); break; case 4: asm volatile (".pushsection .smp_locks,\"a\"\n" ".balign 4\n" ".long 671f - .\n" ".popsection\n" "671:" "\n\tlock; " "xadd" "l %0, %1\n" : "+r" (__ret), "+m" (*(((&sem->count)))) : : "memory", "cc"); break; case 8: asm volatile (".pushsection .smp_locks,\"a\"\n" ".balign 4\n" ".long 671f - .\n" ".popsection\n" "671:" "\n\tlock; " "xadd" "q %q0, %1\n" : "+r" (__ret), "+m" (*(((&sem->count)))) : : "memory", "cc"); break; default: __xadd_wrong_size(); } __ret; });
}


static inline __attribute__((no_instrument_function)) int rwsem_is_locked(struct rw_semaphore *sem)
{
 return sem->count != 0;
}
extern void __init_rwsem(struct rw_semaphore *sem, const char *name,
    struct lock_class_key *key);
static inline __attribute__((no_instrument_function)) int rwsem_is_contended(struct rw_semaphore *sem)
{
 return !list_empty(&sem->wait_list);
}




extern void down_read(struct rw_semaphore *sem);




extern int down_read_trylock(struct rw_semaphore *sem);




extern void down_write(struct rw_semaphore *sem);




extern int down_write_trylock(struct rw_semaphore *sem);




extern void up_read(struct rw_semaphore *sem);




extern void up_write(struct rw_semaphore *sem);




extern void downgrade_write(struct rw_semaphore *sem);
struct completion {
 unsigned int done;
 wait_queue_head_t wait;
};
static inline __attribute__((no_instrument_function)) void init_completion(struct completion *x)
{
 x->done = 0;
 do { static struct lock_class_key __key; __init_waitqueue_head((&x->wait), "&x->wait", &__key); } while (0);
}
static inline __attribute__((no_instrument_function)) void reinit_completion(struct completion *x)
{
 x->done = 0;
}

extern void wait_for_completion(struct completion *);
extern void wait_for_completion_io(struct completion *);
extern int wait_for_completion_interruptible(struct completion *x);
extern int wait_for_completion_killable(struct completion *x);
extern unsigned long wait_for_completion_timeout(struct completion *x,
         unsigned long timeout);
extern unsigned long wait_for_completion_io_timeout(struct completion *x,
          unsigned long timeout);
extern long wait_for_completion_interruptible_timeout(
 struct completion *x, unsigned long timeout);
extern long wait_for_completion_killable_timeout(
 struct completion *x, unsigned long timeout);
extern bool try_wait_for_completion(struct completion *x);
extern bool completion_done(struct completion *x);

extern void complete(struct completion *);
extern void complete_all(struct completion *);






enum debug_obj_state {
 ODEBUG_STATE_NONE,
 ODEBUG_STATE_INIT,
 ODEBUG_STATE_INACTIVE,
 ODEBUG_STATE_ACTIVE,
 ODEBUG_STATE_DESTROYED,
 ODEBUG_STATE_NOTAVAILABLE,
 ODEBUG_STATE_MAX,
};

struct debug_obj_descr;
struct debug_obj {
 struct hlist_node node;
 enum debug_obj_state state;
 unsigned int astate;
 void *object;
 struct debug_obj_descr *descr;
};
struct debug_obj_descr {
 const char *name;
 void *(*debug_hint) (void *addr);
 int (*fixup_init) (void *addr, enum debug_obj_state state);
 int (*fixup_activate) (void *addr, enum debug_obj_state state);
 int (*fixup_destroy) (void *addr, enum debug_obj_state state);
 int (*fixup_free) (void *addr, enum debug_obj_state state);
 int (*fixup_assert_init)(void *addr, enum debug_obj_state state);
};
static inline __attribute__((no_instrument_function)) void
debug_object_init (void *addr, struct debug_obj_descr *descr) { }
static inline __attribute__((no_instrument_function)) void
debug_object_init_on_stack(void *addr, struct debug_obj_descr *descr) { }
static inline __attribute__((no_instrument_function)) int
debug_object_activate (void *addr, struct debug_obj_descr *descr) { return 0; }
static inline __attribute__((no_instrument_function)) void
debug_object_deactivate(void *addr, struct debug_obj_descr *descr) { }
static inline __attribute__((no_instrument_function)) void
debug_object_destroy (void *addr, struct debug_obj_descr *descr) { }
static inline __attribute__((no_instrument_function)) void
debug_object_free (void *addr, struct debug_obj_descr *descr) { }
static inline __attribute__((no_instrument_function)) void
debug_object_assert_init(void *addr, struct debug_obj_descr *descr) { }

static inline __attribute__((no_instrument_function)) void debug_objects_early_init(void) { }
static inline __attribute__((no_instrument_function)) void debug_objects_mem_init(void) { }





static inline __attribute__((no_instrument_function)) void
debug_check_no_obj_freed(const void *address, unsigned long size) { }




extern int rcu_expedited;




enum rcutorture_type {
 RCU_FLAVOR,
 RCU_BH_FLAVOR,
 RCU_SCHED_FLAVOR,
 SRCU_FLAVOR,
 INVALID_RCU_FLAVOR
};


void rcutorture_get_gp_data(enum rcutorture_type test_type, int *flags,
       unsigned long *gpnum, unsigned long *completed);
void rcutorture_record_test_transition(void);
void rcutorture_record_progress(unsigned long vernum);
void do_trace_rcu_torture_read(const char *rcutorturename,
          struct callback_head *rhp,
          unsigned long secs,
          unsigned long c_old,
          unsigned long c);
void call_rcu_bh(struct callback_head *head,
   void (*func)(struct callback_head *head));
void call_rcu_sched(struct callback_head *head,
      void (*func)(struct callback_head *rcu));

void synchronize_sched(void);
static inline __attribute__((no_instrument_function)) void __rcu_read_lock(void)
{
 __asm__ __volatile__("": : :"memory");
}

static inline __attribute__((no_instrument_function)) void __rcu_read_unlock(void)
{
 __asm__ __volatile__("": : :"memory");
}

static inline __attribute__((no_instrument_function)) void synchronize_rcu(void)
{
 synchronize_sched();
}

static inline __attribute__((no_instrument_function)) int rcu_preempt_depth(void)
{
 return 0;
}




void rcu_init(void);
void rcu_sched_qs(int cpu);
void rcu_bh_qs(int cpu);
void rcu_check_callbacks(int cpu, int user);
struct notifier_block;
void rcu_idle_enter(void);
void rcu_idle_exit(void);
void rcu_irq_enter(void);
void rcu_irq_exit(void);


void rcu_sysrq_start(void);
void rcu_sysrq_end(void);
static inline __attribute__((no_instrument_function)) void rcu_user_enter(void) { }
static inline __attribute__((no_instrument_function)) void rcu_user_exit(void) { }
static inline __attribute__((no_instrument_function)) void rcu_user_hooks_switch(struct task_struct *prev,
      struct task_struct *next) { }
bool __rcu_is_watching(void);







typedef void call_rcu_func_t(struct callback_head *head,
        void (*func)(struct callback_head *head));
void wait_rcu_gp(call_rcu_func_t crf);


void rcu_note_context_switch(int cpu);

int rcu_needs_cpu(int cpu, unsigned long *delta_jiffies);

void rcu_cpu_stall_reset(void);






static inline __attribute__((no_instrument_function)) void rcu_virt_note_context_switch(int cpu)
{
 rcu_note_context_switch(cpu);
}

void synchronize_rcu_bh(void);
void synchronize_sched_expedited(void);
void synchronize_rcu_expedited(void);

void kfree_call_rcu(struct callback_head *head, void (*func)(struct callback_head *rcu));
static inline __attribute__((no_instrument_function)) void synchronize_rcu_bh_expedited(void)
{
 synchronize_sched_expedited();
}

void rcu_barrier(void);
void rcu_barrier_bh(void);
void rcu_barrier_sched(void);
unsigned long get_state_synchronize_rcu(void);
void cond_synchronize_rcu(unsigned long oldstate);

extern unsigned long rcutorture_testseq;
extern unsigned long rcutorture_vernum;
long rcu_batches_completed(void);
long rcu_batches_completed_bh(void);
long rcu_batches_completed_sched(void);
void show_rcu_gp_kthreads(void);

void rcu_force_quiescent_state(void);
void rcu_bh_force_quiescent_state(void);
void rcu_sched_force_quiescent_state(void);

void exit_rcu(void);

void rcu_scheduler_starting(void);
extern int rcu_scheduler_active __attribute__((__section__(".data..read_mostly")));

bool rcu_is_watching(void);
static inline __attribute__((no_instrument_function)) void init_rcu_head(struct callback_head *head)
{
}

static inline __attribute__((no_instrument_function)) void destroy_rcu_head(struct callback_head *head)
{
}

static inline __attribute__((no_instrument_function)) void init_rcu_head_on_stack(struct callback_head *head)
{
}

static inline __attribute__((no_instrument_function)) void destroy_rcu_head_on_stack(struct callback_head *head)
{
}





static inline __attribute__((no_instrument_function)) bool rcu_lockdep_current_cpu_online(void)
{
 return 1;
}
static inline __attribute__((no_instrument_function)) int rcu_read_lock_held(void)
{
 return 1;
}

static inline __attribute__((no_instrument_function)) int rcu_read_lock_bh_held(void)
{
 return 1;
}







static inline __attribute__((no_instrument_function)) int rcu_read_lock_sched_held(void)
{
 return 1;
}
static inline __attribute__((no_instrument_function)) void rcu_read_lock(void)
{
 __rcu_read_lock();
 (void)0;
 do { } while (0);
 do { } while (0)
                                                  ;
}
static inline __attribute__((no_instrument_function)) void rcu_read_unlock(void)
{
 do { } while (0)
                                                    ;
 do { } while (0);
 (void)0;
 __rcu_read_unlock();
}
static inline __attribute__((no_instrument_function)) void rcu_read_lock_bh(void)
{
 local_bh_disable();
 (void)0;
 do { } while (0);
 do { } while (0)
                                                     ;
}






static inline __attribute__((no_instrument_function)) void rcu_read_unlock_bh(void)
{
 do { } while (0)
                                                       ;
 do { } while (0);
 (void)0;
 local_bh_enable();
}
static inline __attribute__((no_instrument_function)) void rcu_read_lock_sched(void)
{
 __asm__ __volatile__("": : :"memory");
 (void)0;
 do { } while (0);
 do { } while (0)
                                                        ;
}


static inline __attribute__((no_instrument_function)) __attribute__((no_instrument_function)) void rcu_read_lock_sched_notrace(void)
{
 __asm__ __volatile__("": : :"memory");
 (void)0;
}






static inline __attribute__((no_instrument_function)) void rcu_read_unlock_sched(void)
{
 do { } while (0)
                                                          ;
 do { } while (0);
 (void)0;
 __asm__ __volatile__("": : :"memory");
}


static inline __attribute__((no_instrument_function)) __attribute__((no_instrument_function)) void rcu_read_unlock_sched_notrace(void)
{
 (void)0;
 __asm__ __volatile__("": : :"memory");
}
static inline __attribute__((no_instrument_function)) bool rcu_is_nocb_cpu(int cpu) { return false; }
static inline __attribute__((no_instrument_function)) bool rcu_sys_is_idle(void)
{
 return false;
}

static inline __attribute__((no_instrument_function)) void rcu_sysidle_force_exit(void)
{
}


















struct timex {
 unsigned int modes;
 __kernel_long_t offset;
 __kernel_long_t freq;
 __kernel_long_t maxerror;
 __kernel_long_t esterror;
 int status;
 __kernel_long_t constant;
 __kernel_long_t precision;
 __kernel_long_t tolerance;


 struct timeval time;
 __kernel_long_t tick;

 __kernel_long_t ppsfreq;
 __kernel_long_t jitter;
 int shift;
 __kernel_long_t stabil;
 __kernel_long_t jitcnt;
 __kernel_long_t calcnt;
 __kernel_long_t errcnt;
 __kernel_long_t stbcnt;

 int tai;

 int :32; int :32; int :32; int :32;
 int :32; int :32; int :32; int :32;
 int :32; int :32; int :32;
};

















typedef unsigned long long cycles_t;

extern unsigned int cpu_khz;
extern unsigned int tsc_khz;

extern void disable_TSC(void);

static inline __attribute__((no_instrument_function)) cycles_t get_cycles(void)
{
 unsigned long long ret = 0;





 ((ret) = __native_read_tsc());

 return ret;
}

static inline __attribute__((no_instrument_function)) __attribute__((always_inline)) cycles_t vget_cycles(void)
{
 return (cycles_t)__native_read_tsc();
}

extern void tsc_init(void);
extern void mark_tsc_unstable(char *reason);
extern int unsynchronized_tsc(void);
extern int check_tsc_unstable(void);
extern int check_tsc_disabled(void);
extern unsigned long native_calibrate_tsc(void);

extern int tsc_clocksource_reliable;





extern void check_tsc_sync_source(int cpu);
extern void check_tsc_sync_target(void);

extern int notsc_setup(char *);
extern void tsc_save_sched_clock_state(void);
extern void tsc_restore_sched_clock_state(void);


unsigned long try_msr_calibrate_tsc(void);
extern unsigned long tick_usec;
extern unsigned long tick_nsec;
extern int do_adjtimex(struct timex *);
extern void hardpps(const struct timespec *, const struct timespec *);

int read_current_timer(unsigned long *timer_val);
void ntp_notify_cmos_timer(void);
extern int register_refined_jiffies(long clock_tick_rate);
extern u64 __attribute__((section(".data"))) jiffies_64;
extern unsigned long volatile __attribute__((section(".data"))) jiffies;




static inline __attribute__((no_instrument_function)) u64 get_jiffies_64(void)
{
 return (u64)jiffies;
}
extern unsigned long preset_lpj;
extern unsigned int jiffies_to_msecs(const unsigned long j);
extern unsigned int jiffies_to_usecs(const unsigned long j);

static inline __attribute__((no_instrument_function)) u64 jiffies_to_nsecs(const unsigned long j)
{
 return (u64)jiffies_to_usecs(j) * 1000L;
}

extern unsigned long msecs_to_jiffies(const unsigned int m);
extern unsigned long usecs_to_jiffies(const unsigned int u);
extern unsigned long timespec_to_jiffies(const struct timespec *value);
extern void jiffies_to_timespec(const unsigned long jiffies,
    struct timespec *value);
extern unsigned long timeval_to_jiffies(const struct timeval *value);
extern void jiffies_to_timeval(const unsigned long jiffies,
          struct timeval *value);

extern clock_t jiffies_to_clock_t(unsigned long x);
static inline __attribute__((no_instrument_function)) clock_t jiffies_delta_to_clock_t(long delta)
{
 return jiffies_to_clock_t(({ typeof(0L) _max1 = (0L); typeof(delta) _max2 = (delta); (void) (&_max1 == &_max2); _max1 > _max2 ? _max1 : _max2; }));
}

extern unsigned long clock_t_to_jiffies(unsigned long x);
extern u64 jiffies_64_to_clock_t(u64 x);
extern u64 nsec_to_clock_t(u64 x);
extern u64 nsecs_to_jiffies64(u64 n);
extern unsigned long nsecs_to_jiffies(u64 n);
union ktime {
 s64 tv64;
};

typedef union ktime ktime_t;
static inline __attribute__((no_instrument_function)) ktime_t ktime_set(const s64 secs, const unsigned long nsecs)
{
 if (__builtin_expect(!!(secs >= (((s64)~((u64)1 << 63)) / 1000000000L)), 0))
  return (ktime_t){ .tv64 = ((s64)~((u64)1 << 63)) };

 return (ktime_t) { .tv64 = secs * 1000000000L + (s64)nsecs };
}
static inline __attribute__((no_instrument_function)) ktime_t timespec_to_ktime(struct timespec ts)
{
 return ktime_set(ts.tv_sec, ts.tv_nsec);
}


static inline __attribute__((no_instrument_function)) ktime_t timespec64_to_ktime(struct timespec ts)
{
 return ktime_set(ts.tv_sec, ts.tv_nsec);
}


static inline __attribute__((no_instrument_function)) ktime_t timeval_to_ktime(struct timeval tv)
{
 return ktime_set(tv.tv_sec, tv.tv_usec * 1000L);
}
static inline __attribute__((no_instrument_function)) int ktime_equal(const ktime_t cmp1, const ktime_t cmp2)
{
 return cmp1.tv64 == cmp2.tv64;
}
static inline __attribute__((no_instrument_function)) int ktime_compare(const ktime_t cmp1, const ktime_t cmp2)
{
 if (cmp1.tv64 < cmp2.tv64)
  return -1;
 if (cmp1.tv64 > cmp2.tv64)
  return 1;
 return 0;
}
static inline __attribute__((no_instrument_function)) bool ktime_after(const ktime_t cmp1, const ktime_t cmp2)
{
 return ktime_compare(cmp1, cmp2) > 0;
}
static inline __attribute__((no_instrument_function)) bool ktime_before(const ktime_t cmp1, const ktime_t cmp2)
{
 return ktime_compare(cmp1, cmp2) < 0;
}







static inline __attribute__((no_instrument_function)) s64 ktime_to_us(const ktime_t kt)
{
 return (u64)((kt).tv64 / (1000L));
}

static inline __attribute__((no_instrument_function)) s64 ktime_to_ms(const ktime_t kt)
{
 return (u64)((kt).tv64 / (1000000L));
}

static inline __attribute__((no_instrument_function)) s64 ktime_us_delta(const ktime_t later, const ktime_t earlier)
{
       return ktime_to_us(({ (ktime_t){ .tv64 = (later).tv64 - (earlier).tv64 }; }));
}

static inline __attribute__((no_instrument_function)) ktime_t ktime_add_us(const ktime_t kt, const u64 usec)
{
 return ({ (ktime_t){ .tv64 = (kt).tv64 + (usec * 1000L) }; });
}

static inline __attribute__((no_instrument_function)) ktime_t ktime_add_ms(const ktime_t kt, const u64 msec)
{
 return ({ (ktime_t){ .tv64 = (kt).tv64 + (msec * 1000000L) }; });
}

static inline __attribute__((no_instrument_function)) ktime_t ktime_sub_us(const ktime_t kt, const u64 usec)
{
 return ({ (ktime_t){ .tv64 = (kt).tv64 - (usec * 1000L) }; });
}

extern ktime_t ktime_add_safe(const ktime_t lhs, const ktime_t rhs);
static inline __attribute__((no_instrument_function)) bool ktime_to_timespec_cond(const ktime_t kt,
             struct timespec *ts)
{
 if (kt.tv64) {
  *ts = ns_to_timespec((kt).tv64);
  return true;
 } else {
  return false;
 }
}
static inline __attribute__((no_instrument_function)) bool ktime_to_timespec64_cond(const ktime_t kt,
             struct timespec *ts)
{
 if (kt.tv64) {
  *ts = ns_to_timespec((kt).tv64);
  return true;
 } else {
  return false;
 }
}
static inline __attribute__((no_instrument_function)) ktime_t ns_to_ktime(u64 ns)
{
 static const ktime_t ktime_zero = { .tv64 = 0 };

 return ({ (ktime_t){ .tv64 = (ktime_zero).tv64 + (ns) }; });
}

static inline __attribute__((no_instrument_function)) ktime_t ms_to_ktime(u64 ms)
{
 static const ktime_t ktime_zero = { .tv64 = 0 };

 return ktime_add_ms(ktime_zero, ms);
}






void timekeeping_init(void);
extern int timekeeping_suspended;




extern void do_gettimeofday(struct timeval *tv);
extern int do_settimeofday(const struct timespec *tv);
extern int do_sys_settimeofday(const struct timespec *tv,
          const struct timezone *tz);




unsigned long get_seconds(void);
struct timespec current_kernel_time(void);

struct timespec __current_kernel_time(void);




struct timespec get_monotonic_coarse(void);
extern void getrawmonotonic(struct timespec *ts);
extern void ktime_get_ts64(struct timespec *ts);

extern int __getnstimeofday64(struct timespec *tv);
extern void getnstimeofday64(struct timespec *tv);


static inline __attribute__((no_instrument_function)) int __getnstimeofday(struct timespec *ts)
{
 return __getnstimeofday64(ts);
}

static inline __attribute__((no_instrument_function)) void getnstimeofday(struct timespec *ts)
{
 getnstimeofday64(ts);
}

static inline __attribute__((no_instrument_function)) void ktime_get_ts(struct timespec *ts)
{
 ktime_get_ts64(ts);
}

static inline __attribute__((no_instrument_function)) void ktime_get_real_ts(struct timespec *ts)
{
 getnstimeofday64(ts);
}
extern void getboottime(struct timespec *ts);
enum tk_offsets {
 TK_OFFS_REAL,
 TK_OFFS_BOOT,
 TK_OFFS_TAI,
 TK_OFFS_MAX,
};

extern ktime_t ktime_get(void);
extern ktime_t ktime_get_with_offset(enum tk_offsets offs);
extern ktime_t ktime_mono_to_any(ktime_t tmono, enum tk_offsets offs);
extern ktime_t ktime_get_raw(void);




static inline __attribute__((no_instrument_function)) ktime_t ktime_get_real(void)
{
 return ktime_get_with_offset(TK_OFFS_REAL);
}







static inline __attribute__((no_instrument_function)) ktime_t ktime_get_boottime(void)
{
 return ktime_get_with_offset(TK_OFFS_BOOT);
}




static inline __attribute__((no_instrument_function)) ktime_t ktime_get_clocktai(void)
{
 return ktime_get_with_offset(TK_OFFS_TAI);
}




static inline __attribute__((no_instrument_function)) ktime_t ktime_mono_to_real(ktime_t mono)
{
 return ktime_mono_to_any(mono, TK_OFFS_REAL);
}

static inline __attribute__((no_instrument_function)) u64 ktime_get_ns(void)
{
 return ((ktime_get()).tv64);
}

static inline __attribute__((no_instrument_function)) u64 ktime_get_real_ns(void)
{
 return ((ktime_get_real()).tv64);
}

static inline __attribute__((no_instrument_function)) u64 ktime_get_boot_ns(void)
{
 return ((ktime_get_boottime()).tv64);
}

static inline __attribute__((no_instrument_function)) u64 ktime_get_raw_ns(void)
{
 return ((ktime_get_raw()).tv64);
}

extern u64 ktime_get_mono_fast_ns(void);




static inline __attribute__((no_instrument_function)) void get_monotonic_boottime(struct timespec *ts)
{
 *ts = ns_to_timespec((ktime_get_boottime()).tv64);
}

static inline __attribute__((no_instrument_function)) void timekeeping_clocktai(struct timespec *ts)
{
 *ts = ns_to_timespec((ktime_get_clocktai()).tv64);
}




extern void timekeeping_inject_sleeptime(struct timespec *delta);




extern void getnstime_raw_and_real(struct timespec *ts_raw,
       struct timespec *ts_real);




extern bool persistent_clock_exist;
extern int persistent_clock_is_local;

static inline __attribute__((no_instrument_function)) bool has_persistent_clock(void)
{
 return persistent_clock_exist;
}

extern void read_persistent_clock(struct timespec *ts);
extern void read_boot_clock(struct timespec *ts);
extern int update_persistent_clock(struct timespec now);




struct tvec_base;

struct timer_list {




 struct list_head entry;
 unsigned long expires;
 struct tvec_base *base;

 void (*function)(unsigned long);
 unsigned long data;

 int slack;


 int start_pid;
 void *start_site;
 char start_comm[16];




};

extern struct tvec_base boot_tvec_bases;
void init_timer_key(struct timer_list *timer, unsigned int flags,
      const char *name, struct lock_class_key *key);







static inline __attribute__((no_instrument_function)) void destroy_timer_on_stack(struct timer_list *timer) { }
static inline __attribute__((no_instrument_function)) void init_timer_on_stack_key(struct timer_list *timer,
        unsigned int flags, const char *name,
        struct lock_class_key *key)
{
 init_timer_key(timer, flags, name, key);
}
static inline __attribute__((no_instrument_function)) int timer_pending(const struct timer_list * timer)
{
 return timer->entry.next != ((void *)0);
}

extern void add_timer_on(struct timer_list *timer, int cpu);
extern int del_timer(struct timer_list * timer);
extern int mod_timer(struct timer_list *timer, unsigned long expires);
extern int mod_timer_pending(struct timer_list *timer, unsigned long expires);
extern int mod_timer_pinned(struct timer_list *timer, unsigned long expires);

extern void set_timer_slack(struct timer_list *time, int slack_hz);
extern unsigned long get_next_timer_interrupt(unsigned long now);






extern int timer_stats_active;



extern void init_timer_stats(void);

extern void timer_stats_update_stats(void *timer, pid_t pid, void *startf,
         void *timerf, char *comm,
         unsigned int timer_flag);

extern void __timer_stats_timer_set_start_info(struct timer_list *timer,
            void *addr);

static inline __attribute__((no_instrument_function)) void timer_stats_timer_set_start_info(struct timer_list *timer)
{
 if (__builtin_expect(!!(!timer_stats_active), 1))
  return;
 __timer_stats_timer_set_start_info(timer, __builtin_return_address(0));
}

static inline __attribute__((no_instrument_function)) void timer_stats_timer_clear_start_info(struct timer_list *timer)
{
 timer->start_site = ((void *)0);
}
extern void add_timer(struct timer_list *timer);

extern int try_to_del_timer_sync(struct timer_list *timer);


  extern int del_timer_sync(struct timer_list *timer);






extern void init_timers(void);
extern void run_local_timers(void);
struct hrtimer;
extern enum hrtimer_restart it_real_fn(struct hrtimer *);

unsigned long __round_jiffies(unsigned long j, int cpu);
unsigned long __round_jiffies_relative(unsigned long j, int cpu);
unsigned long round_jiffies(unsigned long j);
unsigned long round_jiffies_relative(unsigned long j);

unsigned long __round_jiffies_up(unsigned long j, int cpu);
unsigned long __round_jiffies_up_relative(unsigned long j, int cpu);
unsigned long round_jiffies_up(unsigned long j);
unsigned long round_jiffies_up_relative(unsigned long j);







struct workqueue_struct;

struct work_struct;
typedef void (*work_func_t)(struct work_struct *work);
void delayed_work_timer_fn(unsigned long __data);







enum {
 WORK_STRUCT_PENDING_BIT = 0,
 WORK_STRUCT_DELAYED_BIT = 1,
 WORK_STRUCT_PWQ_BIT = 2,
 WORK_STRUCT_LINKED_BIT = 3,




 WORK_STRUCT_COLOR_SHIFT = 4,


 WORK_STRUCT_COLOR_BITS = 4,

 WORK_STRUCT_PENDING = 1 << WORK_STRUCT_PENDING_BIT,
 WORK_STRUCT_DELAYED = 1 << WORK_STRUCT_DELAYED_BIT,
 WORK_STRUCT_PWQ = 1 << WORK_STRUCT_PWQ_BIT,
 WORK_STRUCT_LINKED = 1 << WORK_STRUCT_LINKED_BIT,



 WORK_STRUCT_STATIC = 0,






 WORK_NR_COLORS = (1 << WORK_STRUCT_COLOR_BITS) - 1,
 WORK_NO_COLOR = WORK_NR_COLORS,


 WORK_CPU_UNBOUND = 256,






 WORK_STRUCT_FLAG_BITS = WORK_STRUCT_COLOR_SHIFT +
      WORK_STRUCT_COLOR_BITS,


 WORK_OFFQ_FLAG_BASE = WORK_STRUCT_COLOR_SHIFT,

 WORK_OFFQ_CANCELING = (1 << WORK_OFFQ_FLAG_BASE),






 WORK_OFFQ_FLAG_BITS = 1,
 WORK_OFFQ_POOL_SHIFT = WORK_OFFQ_FLAG_BASE + WORK_OFFQ_FLAG_BITS,
 WORK_OFFQ_LEFT = 64 - WORK_OFFQ_POOL_SHIFT,
 WORK_OFFQ_POOL_BITS = WORK_OFFQ_LEFT <= 31 ? WORK_OFFQ_LEFT : 31,
 WORK_OFFQ_POOL_NONE = (1LU << WORK_OFFQ_POOL_BITS) - 1,


 WORK_STRUCT_FLAG_MASK = (1UL << WORK_STRUCT_FLAG_BITS) - 1,
 WORK_STRUCT_WQ_DATA_MASK = ~WORK_STRUCT_FLAG_MASK,
 WORK_STRUCT_NO_POOL = (unsigned long)WORK_OFFQ_POOL_NONE << WORK_OFFQ_POOL_SHIFT,


 WORK_BUSY_PENDING = 1 << 0,
 WORK_BUSY_RUNNING = 1 << 1,


 WORKER_DESC_LEN = 24,
};

struct work_struct {
 atomic_long_t data;
 struct list_head entry;
 work_func_t func;



};





struct delayed_work {
 struct work_struct work;
 struct timer_list timer;


 struct workqueue_struct *wq;
 int cpu;
};
struct workqueue_attrs {
 int nice;
 cpumask_var_t cpumask;
 bool no_numa;
};

static inline __attribute__((no_instrument_function)) struct delayed_work *to_delayed_work(struct work_struct *work)
{
 return ({ const typeof( ((struct delayed_work *)0)->work ) *__mptr = (work); (struct delayed_work *)( (char *)__mptr - __builtin_offsetof(struct delayed_work,work) );});
}

struct execute_work {
 struct work_struct work;
};
static inline __attribute__((no_instrument_function)) void __init_work(struct work_struct *work, int onstack) { }
static inline __attribute__((no_instrument_function)) void destroy_work_on_stack(struct work_struct *work) { }
static inline __attribute__((no_instrument_function)) void destroy_delayed_work_on_stack(struct delayed_work *work) { }
static inline __attribute__((no_instrument_function)) unsigned int work_static(struct work_struct *work) { return 0; }
enum {
 WQ_UNBOUND = 1 << 1,
 WQ_FREEZABLE = 1 << 2,
 WQ_MEM_RECLAIM = 1 << 3,
 WQ_HIGHPRI = 1 << 4,
 WQ_CPU_INTENSIVE = 1 << 5,
 WQ_SYSFS = 1 << 6,
 WQ_POWER_EFFICIENT = 1 << 7,

 __WQ_DRAINING = 1 << 16,
 __WQ_ORDERED = 1 << 17,

 WQ_MAX_ACTIVE = 512,
 WQ_MAX_UNBOUND_PER_CPU = 4,
 WQ_DFL_ACTIVE = WQ_MAX_ACTIVE / 2,
};
extern struct workqueue_struct *system_wq;
extern struct workqueue_struct *system_highpri_wq;
extern struct workqueue_struct *system_long_wq;
extern struct workqueue_struct *system_unbound_wq;
extern struct workqueue_struct *system_freezable_wq;
extern struct workqueue_struct *system_power_efficient_wq;
extern struct workqueue_struct *system_freezable_power_efficient_wq;

extern struct workqueue_struct *
__alloc_workqueue_key(const char *fmt, unsigned int flags, int max_active,
 struct lock_class_key *key, const char *lock_name, ...) __attribute__((format(printf, 1, 6)));
extern void destroy_workqueue(struct workqueue_struct *wq);

struct workqueue_attrs *alloc_workqueue_attrs(gfp_t gfp_mask);
void free_workqueue_attrs(struct workqueue_attrs *attrs);
int apply_workqueue_attrs(struct workqueue_struct *wq,
     const struct workqueue_attrs *attrs);

extern bool queue_work_on(int cpu, struct workqueue_struct *wq,
   struct work_struct *work);
extern bool queue_delayed_work_on(int cpu, struct workqueue_struct *wq,
   struct delayed_work *work, unsigned long delay);
extern bool mod_delayed_work_on(int cpu, struct workqueue_struct *wq,
   struct delayed_work *dwork, unsigned long delay);

extern void flush_workqueue(struct workqueue_struct *wq);
extern void drain_workqueue(struct workqueue_struct *wq);
extern void flush_scheduled_work(void);

extern int schedule_on_each_cpu(work_func_t func);

int execute_in_process_context(work_func_t fn, struct execute_work *);

extern bool flush_work(struct work_struct *work);
extern bool cancel_work_sync(struct work_struct *work);

extern bool flush_delayed_work(struct delayed_work *dwork);
extern bool cancel_delayed_work(struct delayed_work *dwork);
extern bool cancel_delayed_work_sync(struct delayed_work *dwork);

extern void workqueue_set_max_active(struct workqueue_struct *wq,
         int max_active);
extern bool current_is_workqueue_rescuer(void);
extern bool workqueue_congested(int cpu, struct workqueue_struct *wq);
extern unsigned int work_busy(struct work_struct *work);
extern __attribute__((format(printf, 1, 2))) void set_worker_desc(const char *fmt, ...);
extern void print_worker_info(const char *log_lvl, struct task_struct *task);
static inline __attribute__((no_instrument_function)) bool queue_work(struct workqueue_struct *wq,
         struct work_struct *work)
{
 return queue_work_on(WORK_CPU_UNBOUND, wq, work);
}
static inline __attribute__((no_instrument_function)) bool queue_delayed_work(struct workqueue_struct *wq,
          struct delayed_work *dwork,
          unsigned long delay)
{
 return queue_delayed_work_on(WORK_CPU_UNBOUND, wq, dwork, delay);
}
static inline __attribute__((no_instrument_function)) bool mod_delayed_work(struct workqueue_struct *wq,
        struct delayed_work *dwork,
        unsigned long delay)
{
 return mod_delayed_work_on(WORK_CPU_UNBOUND, wq, dwork, delay);
}
static inline __attribute__((no_instrument_function)) bool schedule_work_on(int cpu, struct work_struct *work)
{
 return queue_work_on(cpu, system_wq, work);
}
static inline __attribute__((no_instrument_function)) bool schedule_work(struct work_struct *work)
{
 return queue_work(system_wq, work);
}
static inline __attribute__((no_instrument_function)) bool schedule_delayed_work_on(int cpu, struct delayed_work *dwork,
         unsigned long delay)
{
 return queue_delayed_work_on(cpu, system_wq, dwork, delay);
}
static inline __attribute__((no_instrument_function)) bool schedule_delayed_work(struct delayed_work *dwork,
      unsigned long delay)
{
 return queue_delayed_work(system_wq, dwork, delay);
}




static inline __attribute__((no_instrument_function)) bool keventd_up(void)
{
 return system_wq != ((void *)0);
}







long work_on_cpu(int cpu, long (*fn)(void *), void *arg);



extern void freeze_workqueues_begin(void);
extern bool freeze_workqueues_busy(void);
extern void thaw_workqueues(void);



int workqueue_sysfs_register(struct workqueue_struct *wq);

struct srcu_struct_array {
 unsigned long c[2];
 unsigned long seq[2];
};

struct rcu_batch {
 struct callback_head *head, **tail;
};



struct srcu_struct {
 unsigned completed;
 struct srcu_struct_array *per_cpu_ref;
 spinlock_t queue_lock;
 bool running;

 struct rcu_batch batch_queue;

 struct rcu_batch batch_check0;

 struct rcu_batch batch_check1;
 struct rcu_batch batch_done;
 struct delayed_work work;



};
int init_srcu_struct(struct srcu_struct *sp);




void process_srcu(struct work_struct *work);
void call_srcu(struct srcu_struct *sp, struct callback_head *head,
  void (*func)(struct callback_head *head));

void cleanup_srcu_struct(struct srcu_struct *sp);
int __srcu_read_lock(struct srcu_struct *sp) ;
void __srcu_read_unlock(struct srcu_struct *sp, int idx) ;
void synchronize_srcu(struct srcu_struct *sp);
void synchronize_srcu_expedited(struct srcu_struct *sp);
long srcu_batches_completed(struct srcu_struct *sp);
void srcu_barrier(struct srcu_struct *sp);
static inline __attribute__((no_instrument_function)) int srcu_read_lock_held(struct srcu_struct *sp)
{
 return 1;
}
static inline __attribute__((no_instrument_function)) int srcu_read_lock(struct srcu_struct *sp)
{
 int retval = __srcu_read_lock(sp);

 do { } while (0);
 return retval;
}
static inline __attribute__((no_instrument_function)) void srcu_read_unlock(struct srcu_struct *sp, int idx)

{
 do { } while (0);
 __srcu_read_unlock(sp, idx);
}
static inline __attribute__((no_instrument_function)) void smp_mb__after_srcu_read_unlock(void)
{

}
typedef int (*notifier_fn_t)(struct notifier_block *nb,
   unsigned long action, void *data);

struct notifier_block {
 notifier_fn_t notifier_call;
 struct notifier_block *next;
 int priority;
};

struct atomic_notifier_head {
 spinlock_t lock;
 struct notifier_block *head;
};

struct blocking_notifier_head {
 struct rw_semaphore rwsem;
 struct notifier_block *head;
};

struct raw_notifier_head {
 struct notifier_block *head;
};

struct srcu_notifier_head {
 struct mutex mutex;
 struct srcu_struct srcu;
 struct notifier_block *head;
};
extern void srcu_init_notifier_head(struct srcu_notifier_head *nh);
extern int atomic_notifier_chain_register(struct atomic_notifier_head *nh,
  struct notifier_block *nb);
extern int blocking_notifier_chain_register(struct blocking_notifier_head *nh,
  struct notifier_block *nb);
extern int raw_notifier_chain_register(struct raw_notifier_head *nh,
  struct notifier_block *nb);
extern int srcu_notifier_chain_register(struct srcu_notifier_head *nh,
  struct notifier_block *nb);

extern int blocking_notifier_chain_cond_register(
  struct blocking_notifier_head *nh,
  struct notifier_block *nb);

extern int atomic_notifier_chain_unregister(struct atomic_notifier_head *nh,
  struct notifier_block *nb);
extern int blocking_notifier_chain_unregister(struct blocking_notifier_head *nh,
  struct notifier_block *nb);
extern int raw_notifier_chain_unregister(struct raw_notifier_head *nh,
  struct notifier_block *nb);
extern int srcu_notifier_chain_unregister(struct srcu_notifier_head *nh,
  struct notifier_block *nb);

extern int atomic_notifier_call_chain(struct atomic_notifier_head *nh,
  unsigned long val, void *v);
extern int __atomic_notifier_call_chain(struct atomic_notifier_head *nh,
 unsigned long val, void *v, int nr_to_call, int *nr_calls);
extern int blocking_notifier_call_chain(struct blocking_notifier_head *nh,
  unsigned long val, void *v);
extern int __blocking_notifier_call_chain(struct blocking_notifier_head *nh,
 unsigned long val, void *v, int nr_to_call, int *nr_calls);
extern int raw_notifier_call_chain(struct raw_notifier_head *nh,
  unsigned long val, void *v);
extern int __raw_notifier_call_chain(struct raw_notifier_head *nh,
 unsigned long val, void *v, int nr_to_call, int *nr_calls);
extern int srcu_notifier_call_chain(struct srcu_notifier_head *nh,
  unsigned long val, void *v);
extern int __srcu_notifier_call_chain(struct srcu_notifier_head *nh,
 unsigned long val, void *v, int nr_to_call, int *nr_calls);
static inline __attribute__((no_instrument_function)) int notifier_from_errno(int err)
{
 if (err)
  return 0x8000 | (0x0001 - err);

 return 0x0001;
}


static inline __attribute__((no_instrument_function)) int notifier_to_errno(int ret)
{
 ret &= ~0x8000;
 return ret > 0x0001 ? 0x0001 - ret : 0;
}
extern struct blocking_notifier_head reboot_notifier_list;


struct page;
struct zone;
struct pglist_data;
struct mem_section;
struct memory_block;







enum {
 MEMORY_HOTPLUG_MIN_BOOTMEM_TYPE = 12,
 SECTION_INFO = MEMORY_HOTPLUG_MIN_BOOTMEM_TYPE,
 MIX_SECTION_INFO,
 NODE_INFO,
 MEMORY_HOTPLUG_MAX_BOOTMEM_TYPE = NODE_INFO,
};


enum {
 MMOP_OFFLINE = -1,
 MMOP_ONLINE_KEEP,
 MMOP_ONLINE_KERNEL,
 MMOP_ONLINE_MOVABLE,
};




static inline __attribute__((no_instrument_function))
void pgdat_resize_lock(struct pglist_data *pgdat, unsigned long *flags)
{
 do { do { ({ unsigned long __dummy; typeof(*flags) __dummy2; (void)(&__dummy == &__dummy2); 1; }); *flags = _raw_spin_lock_irqsave(spinlock_check(&pgdat->node_size_lock)); } while (0); } while (0);
}
static inline __attribute__((no_instrument_function))
void pgdat_resize_unlock(struct pglist_data *pgdat, unsigned long *flags)
{
 spin_unlock_irqrestore(&pgdat->node_size_lock, *flags);
}
static inline __attribute__((no_instrument_function))
void pgdat_resize_init(struct pglist_data *pgdat)
{
 do { spinlock_check(&pgdat->node_size_lock); do { *(&(&pgdat->node_size_lock)->rlock) = (raw_spinlock_t) { .raw_lock = { { 0 } }, }; } while (0); } while (0);
}







static inline __attribute__((no_instrument_function)) unsigned zone_span_seqbegin(struct zone *zone)
{
 return read_seqbegin(&zone->span_seqlock);
}
static inline __attribute__((no_instrument_function)) int zone_span_seqretry(struct zone *zone, unsigned iv)
{
 return read_seqretry(&zone->span_seqlock, iv);
}
static inline __attribute__((no_instrument_function)) void zone_span_writelock(struct zone *zone)
{
 write_seqlock(&zone->span_seqlock);
}
static inline __attribute__((no_instrument_function)) void zone_span_writeunlock(struct zone *zone)
{
 write_sequnlock(&zone->span_seqlock);
}
static inline __attribute__((no_instrument_function)) void zone_seqlock_init(struct zone *zone)
{
 do { __seqcount_init(&(&zone->span_seqlock)->seqcount, ((void *)0), ((void *)0)); do { spinlock_check(&(&zone->span_seqlock)->lock); do { *(&(&(&zone->span_seqlock)->lock)->rlock) = (raw_spinlock_t) { .raw_lock = { { 0 } }, }; } while (0); } while (0); } while (0);
}
extern int zone_grow_free_lists(struct zone *zone, unsigned long new_nr_pages);
extern int zone_grow_waitqueues(struct zone *zone, unsigned long nr_pages);
extern int add_one_highpage(struct page *page, int pfn, int bad_ppro);

extern int online_pages(unsigned long, unsigned long, int);
extern void __offline_isolated_pages(unsigned long, unsigned long);

typedef void (*online_page_callback_t)(struct page *page);

extern int set_online_page_callback(online_page_callback_t callback);
extern int restore_online_page_callback(online_page_callback_t callback);

extern void __online_page_set_limits(struct page *page);
extern void __online_page_increment_counters(struct page *page);
extern void __online_page_free(struct page *page);

extern int try_online_node(int nid);


extern bool is_pageblock_removable_nolock(struct page *page);
extern int arch_remove_memory(u64 start, u64 size);
extern int __remove_pages(struct zone *zone, unsigned long start_pfn,
 unsigned long nr_pages);



extern int __add_pages(int nid, struct zone *zone, unsigned long start_pfn,
 unsigned long nr_pages);


extern int memory_add_physaddr_to_nid(u64 start);
extern pg_data_t *node_data[];
static inline __attribute__((no_instrument_function)) void arch_refresh_nodedata(int nid, pg_data_t *pgdat)
{
 node_data[nid] = pgdat;
}
extern void register_page_bootmem_info_node(struct pglist_data *pgdat);





extern void put_page_bootmem(struct page *page);
extern void get_page_bootmem(unsigned long ingo, struct page *page,
        unsigned long type);

void get_online_mems(void);
void put_online_mems(void);
extern int is_mem_section_removable(unsigned long pfn, unsigned long nr_pages);
extern void try_offline_node(int nid);
extern int offline_pages(unsigned long start_pfn, unsigned long nr_pages);
extern void remove_memory(int nid, u64 start, u64 size);
extern int walk_memory_range(unsigned long start_pfn, unsigned long end_pfn,
  void *arg, int (*func)(struct memory_block *, void *));
extern int add_memory(int nid, u64 start, u64 size);
extern int zone_for_memory(int nid, u64 start, u64 size, int zone_default);
extern int arch_add_memory(int nid, u64 start, u64 size);
extern int offline_pages(unsigned long start_pfn, unsigned long nr_pages);
extern bool is_memblock_offlined(struct memory_block *mem);
extern void remove_memory(int nid, u64 start, u64 size);
extern int sparse_add_one_section(struct zone *zone, unsigned long start_pfn);
extern void sparse_remove_one_section(struct zone *zone, struct mem_section *ms);
extern struct page *sparse_decode_mem_map(unsigned long coded_mem_map,
       unsigned long pnum);

extern struct mutex zonelists_mutex;
void build_all_zonelists(pg_data_t *pgdat, struct zone *zone);
void wakeup_kswapd(struct zone *zone, int order, enum zone_type classzone_idx);
bool zone_watermark_ok(struct zone *z, unsigned int order,
  unsigned long mark, int classzone_idx, int alloc_flags);
bool zone_watermark_ok_safe(struct zone *z, unsigned int order,
  unsigned long mark, int classzone_idx, int alloc_flags);
enum memmap_context {
 MEMMAP_EARLY,
 MEMMAP_HOTPLUG,
};
extern int init_currently_empty_zone(struct zone *zone, unsigned long start_pfn,
         unsigned long size,
         enum memmap_context context);

extern void lruvec_init(struct lruvec *lruvec);

static inline __attribute__((no_instrument_function)) struct zone *lruvec_zone(struct lruvec *lruvec)
{



 return ({ const typeof( ((struct zone *)0)->lruvec ) *__mptr = (lruvec); (struct zone *)( (char *)__mptr - __builtin_offsetof(struct zone,lruvec) );});

}


void memory_present(int nid, unsigned long start, unsigned long end);







static inline __attribute__((no_instrument_function)) int local_memory_node(int node_id) { return node_id; };
static inline __attribute__((no_instrument_function)) int populated_zone(struct zone *zone)
{
 return (!!zone->present_pages);
}

extern int movable_zone;

static inline __attribute__((no_instrument_function)) int zone_movable_is_highmem(void)
{





 return 0;

}

static inline __attribute__((no_instrument_function)) int is_highmem_idx(enum zone_type idx)
{




 return 0;

}







static inline __attribute__((no_instrument_function)) int is_highmem(struct zone *zone)
{






 return 0;

}


struct ctl_table;
int min_free_kbytes_sysctl_handler(struct ctl_table *, int,
     void *, size_t *, loff_t *);
extern int sysctl_lowmem_reserve_ratio[4 -1];
int lowmem_reserve_ratio_sysctl_handler(struct ctl_table *, int,
     void *, size_t *, loff_t *);
int percpu_pagelist_fraction_sysctl_handler(struct ctl_table *, int,
     void *, size_t *, loff_t *);
int sysctl_min_unmapped_ratio_sysctl_handler(struct ctl_table *, int,
   void *, size_t *, loff_t *);
int sysctl_min_slab_ratio_sysctl_handler(struct ctl_table *, int,
   void *, size_t *, loff_t *);

extern int numa_zonelist_order_handler(struct ctl_table *, int,
   void *, size_t *, loff_t *);
extern char numa_zonelist_order[];







struct mpf_intel {
 char signature[4];
 unsigned int physptr;
 unsigned char length;
 unsigned char specification;
 unsigned char checksum;
 unsigned char feature1;
 unsigned char feature2;
 unsigned char feature3;
 unsigned char feature4;
 unsigned char feature5;
};



struct mpc_table {
 char signature[4];
 unsigned short length;
 char spec;
 char checksum;
 char oem[8];
 char productid[12];
 unsigned int oemptr;
 unsigned short oemsize;
 unsigned short oemcount;
 unsigned int lapic;
 unsigned int reserved;
};
struct mpc_cpu {
 unsigned char type;
 unsigned char apicid;
 unsigned char apicver;
 unsigned char cpuflag;
 unsigned int cpufeature;
 unsigned int featureflag;
 unsigned int reserved[2];
};

struct mpc_bus {
 unsigned char type;
 unsigned char busid;
 unsigned char bustype[6];
};
struct mpc_ioapic {
 unsigned char type;
 unsigned char apicid;
 unsigned char apicver;
 unsigned char flags;
 unsigned int apicaddr;
};

struct mpc_intsrc {
 unsigned char type;
 unsigned char irqtype;
 unsigned short irqflag;
 unsigned char srcbus;
 unsigned char srcbusirq;
 unsigned char dstapic;
 unsigned char dstirq;
};

enum mp_irq_source_types {
 mp_INT = 0,
 mp_NMI = 1,
 mp_SMI = 2,
 mp_ExtINT = 3
};







struct mpc_lintsrc {
 unsigned char type;
 unsigned char irqtype;
 unsigned short irqflag;
 unsigned char srcbusid;
 unsigned char srcbusirq;
 unsigned char destapic;
 unsigned char destapiclint;
};



struct mpc_oemtable {
 char signature[4];
 unsigned short length;
 char rev;
 char checksum;
 char mpc[8];
};
enum mp_bustype {
 MP_BUS_ISA = 1,
 MP_BUS_EISA,
 MP_BUS_PCI,
};







struct screen_info {
 __u8 orig_x;
 __u8 orig_y;
 __u16 ext_mem_k;
 __u16 orig_video_page;
 __u8 orig_video_mode;
 __u8 orig_video_cols;
 __u8 flags;
 __u8 unused2;
 __u16 orig_video_ega_bx;
 __u16 unused3;
 __u8 orig_video_lines;
 __u8 orig_video_isVGA;
 __u16 orig_video_points;


 __u16 lfb_width;
 __u16 lfb_height;
 __u16 lfb_depth;
 __u32 lfb_base;
 __u32 lfb_size;
 __u16 cl_magic, cl_offset;
 __u16 lfb_linelength;
 __u8 red_size;
 __u8 red_pos;
 __u8 green_size;
 __u8 green_pos;
 __u8 blue_size;
 __u8 blue_pos;
 __u8 rsvd_size;
 __u8 rsvd_pos;
 __u16 vesapm_seg;
 __u16 vesapm_off;
 __u16 pages;
 __u16 vesa_attributes;
 __u32 capabilities;
 __u8 _reserved[6];
} __attribute__((packed));

extern struct screen_info screen_info;
typedef unsigned short apm_event_t;
typedef unsigned short apm_eventinfo_t;

struct apm_bios_info {
 __u16 version;
 __u16 cseg;
 __u32 offset;
 __u16 cseg_16;
 __u16 dseg;
 __u16 flags;
 __u16 cseg_len;
 __u16 cseg_16_len;
 __u16 dseg_len;
};
struct apm_info {
 struct apm_bios_info bios;
 unsigned short connection_version;
 int get_power_status_broken;
 int get_power_status_swabinminutes;
 int allow_ints;
 int forbid_idle;
 int realmode_power_off;
 int disabled;
};
extern struct apm_info apm_info;
struct edd_device_params {
 __u16 length;
 __u16 info_flags;
 __u32 num_default_cylinders;
 __u32 num_default_heads;
 __u32 sectors_per_track;
 __u64 number_of_sectors;
 __u16 bytes_per_sector;
 __u32 dpte_ptr;
 __u16 key;
 __u8 device_path_info_length;
 __u8 reserved2;
 __u16 reserved3;
 __u8 host_bus_type[4];
 __u8 interface_type[8];
 union {
  struct {
   __u16 base_address;
   __u16 reserved1;
   __u32 reserved2;
  } __attribute__ ((packed)) isa;
  struct {
   __u8 bus;
   __u8 slot;
   __u8 function;
   __u8 channel;
   __u32 reserved;
  } __attribute__ ((packed)) pci;

  struct {
   __u64 reserved;
  } __attribute__ ((packed)) ibnd;
  struct {
   __u64 reserved;
  } __attribute__ ((packed)) xprs;
  struct {
   __u64 reserved;
  } __attribute__ ((packed)) htpt;
  struct {
   __u64 reserved;
  } __attribute__ ((packed)) unknown;
 } interface_path;
 union {
  struct {
   __u8 device;
   __u8 reserved1;
   __u16 reserved2;
   __u32 reserved3;
   __u64 reserved4;
  } __attribute__ ((packed)) ata;
  struct {
   __u8 device;
   __u8 lun;
   __u8 reserved1;
   __u8 reserved2;
   __u32 reserved3;
   __u64 reserved4;
  } __attribute__ ((packed)) atapi;
  struct {
   __u16 id;
   __u64 lun;
   __u16 reserved1;
   __u32 reserved2;
  } __attribute__ ((packed)) scsi;
  struct {
   __u64 serial_number;
   __u64 reserved;
  } __attribute__ ((packed)) usb;
  struct {
   __u64 eui;
   __u64 reserved;
  } __attribute__ ((packed)) i1394;
  struct {
   __u64 wwid;
   __u64 lun;
  } __attribute__ ((packed)) fibre;
  struct {
   __u64 identity_tag;
   __u64 reserved;
  } __attribute__ ((packed)) i2o;
  struct {
   __u32 array_number;
   __u32 reserved1;
   __u64 reserved2;
  } __attribute__ ((packed)) raid;
  struct {
   __u8 device;
   __u8 reserved1;
   __u16 reserved2;
   __u32 reserved3;
   __u64 reserved4;
  } __attribute__ ((packed)) sata;
  struct {
   __u64 reserved1;
   __u64 reserved2;
  } __attribute__ ((packed)) unknown;
 } device_path;
 __u8 reserved4;
 __u8 checksum;
} __attribute__ ((packed));

struct edd_info {
 __u8 device;
 __u8 version;
 __u16 interface_support;
 __u16 legacy_max_cylinder;
 __u8 legacy_max_head;
 __u8 legacy_sectors_per_track;
 struct edd_device_params params;
} __attribute__ ((packed));

struct edd {
 unsigned int mbr_signature[16];
 struct edd_info edd_info[6];
 unsigned char mbr_signature_nr;
 unsigned char edd_info_nr;
};


extern struct edd edd;
struct e820entry {
 __u64 addr;
 __u64 size;
 __u32 type;
} __attribute__((packed));

struct e820map {
 __u32 nr_map;
 struct e820entry map[(128 + 3 * (1 << 6))];
};


extern struct e820map e820;
extern struct e820map e820_saved;

extern unsigned long pci_mem_start;
extern int e820_any_mapped(u64 start, u64 end, unsigned type);
extern int e820_all_mapped(u64 start, u64 end, unsigned type);
extern void e820_add_region(u64 start, u64 size, int type);
extern void e820_print_map(char *who);
extern int
sanitize_e820_map(struct e820entry *biosmap, int max_nr_map, u32 *pnr_map);
extern u64 e820_update_range(u64 start, u64 size, unsigned old_type,
          unsigned new_type);
extern u64 e820_remove_range(u64 start, u64 size, unsigned old_type,
        int checktype);
extern void update_e820(void);
extern void e820_setup_gap(void);
extern int e820_search_gap(unsigned long *gapstart, unsigned long *gapsize,
   unsigned long start_addr, unsigned long long end_addr);
struct setup_data;
extern void parse_e820_ext(u64 phys_addr, u32 data_len);



extern void e820_mark_nosave_regions(unsigned long limit_pfn);
static inline __attribute__((no_instrument_function)) void early_memtest(unsigned long start, unsigned long end)
{
}


extern unsigned long e820_end_of_ram_pfn(void);
extern unsigned long e820_end_of_low_ram_pfn(void);
extern u64 early_reserve_e820(u64 sizet, u64 align);

void memblock_x86_fill(void);
void memblock_find_dma_reserve(void);

extern void finish_e820_parsing(void);
extern void e820_reserve_resources(void);
extern void e820_reserve_resources_late(void);
extern void setup_memory_map(void);
extern char *default_machine_specific_memory_setup(void);





static inline __attribute__((no_instrument_function)) bool is_ISA_range(u64 s, u64 e)
{
 return s >= 0xa0000 && e <= 0x100000;
}


struct resource {
 resource_size_t start;
 resource_size_t end;
 const char *name;
 unsigned long flags;
 struct resource *parent, *sibling, *child;
};
extern struct resource ioport_resource;
extern struct resource iomem_resource;

extern struct resource *request_resource_conflict(struct resource *root, struct resource *new);
extern int request_resource(struct resource *root, struct resource *new);
extern int release_resource(struct resource *new);
void release_child_resources(struct resource *new);
extern void reserve_region_with_split(struct resource *root,
        resource_size_t start, resource_size_t end,
        const char *name);
extern struct resource *insert_resource_conflict(struct resource *parent, struct resource *new);
extern int insert_resource(struct resource *parent, struct resource *new);
extern void insert_resource_expand_to_fit(struct resource *root, struct resource *new);
extern void arch_remove_reservations(struct resource *avail);
extern int allocate_resource(struct resource *root, struct resource *new,
        resource_size_t size, resource_size_t min,
        resource_size_t max, resource_size_t align,
        resource_size_t (*alignf)(void *,
             const struct resource *,
             resource_size_t,
             resource_size_t),
        void *alignf_data);
struct resource *lookup_resource(struct resource *root, resource_size_t start);
int adjust_resource(struct resource *res, resource_size_t start,
      resource_size_t size);
resource_size_t resource_alignment(struct resource *res);
static inline __attribute__((no_instrument_function)) resource_size_t resource_size(const struct resource *res)
{
 return res->end - res->start + 1;
}
static inline __attribute__((no_instrument_function)) unsigned long resource_type(const struct resource *res)
{
 return res->flags & 0x00001f00;
}

static inline __attribute__((no_instrument_function)) bool resource_contains(struct resource *r1, struct resource *r2)
{
 if (resource_type(r1) != resource_type(r2))
  return false;
 if (r1->flags & 0x20000000 || r2->flags & 0x20000000)
  return false;
 return r1->start <= r2->start && r1->end >= r2->end;
}
extern struct resource * __request_region(struct resource *,
     resource_size_t start,
     resource_size_t n,
     const char *name, int flags);






extern int __check_region(struct resource *, resource_size_t, resource_size_t);
extern void __release_region(struct resource *, resource_size_t,
    resource_size_t);

extern int release_mem_region_adjustable(struct resource *, resource_size_t,
    resource_size_t);


static inline __attribute__((no_instrument_function)) int check_region(resource_size_t s,
      resource_size_t n)
{
 return __check_region(&ioport_resource, s, n);
}


struct device;





extern struct resource * __devm_request_region(struct device *dev,
    struct resource *parent, resource_size_t start,
    resource_size_t n, const char *name);






extern void __devm_release_region(struct device *dev, struct resource *parent,
      resource_size_t start, resource_size_t n);
extern int iomem_map_sanity_check(resource_size_t addr, unsigned long size);
extern int iomem_is_exclusive(u64 addr);

extern int
walk_system_ram_range(unsigned long start_pfn, unsigned long nr_pages,
  void *arg, int (*func)(unsigned long, unsigned long, void *));
extern int
walk_system_ram_res(u64 start, u64 end, void *arg,
      int (*func)(u64, u64, void *));
extern int
walk_iomem_res(char *name, unsigned long flags, u64 start, u64 end, void *arg,
        int (*func)(u64, u64, void *));


static inline __attribute__((no_instrument_function)) bool resource_overlaps(struct resource *r1, struct resource *r2)
{
       return (r1->start <= r2->end && r1->end >= r2->start);
}
struct ist_info {
 __u32 signature;
 __u32 command;
 __u32 event;
 __u32 perf_level;
};


extern struct ist_info ist_info;






struct edid_info {
 unsigned char dummy[128];
};


extern struct edid_info edid_info;


struct setup_data {
 __u64 next;
 __u32 type;
 __u32 len;
 __u8 data[0];
};

struct setup_header {
 __u8 setup_sects;
 __u16 root_flags;
 __u32 syssize;
 __u16 ram_size;
 __u16 vid_mode;
 __u16 root_dev;
 __u16 boot_flag;
 __u16 jump;
 __u32 header;
 __u16 version;
 __u32 realmode_swtch;
 __u16 start_sys;
 __u16 kernel_version;
 __u8 type_of_loader;
 __u8 loadflags;
 __u16 setup_move_size;
 __u32 code32_start;
 __u32 ramdisk_image;
 __u32 ramdisk_size;
 __u32 bootsect_kludge;
 __u16 heap_end_ptr;
 __u8 ext_loader_ver;
 __u8 ext_loader_type;
 __u32 cmd_line_ptr;
 __u32 initrd_addr_max;
 __u32 kernel_alignment;
 __u8 relocatable_kernel;
 __u8 min_alignment;
 __u16 xloadflags;
 __u32 cmdline_size;
 __u32 hardware_subarch;
 __u64 hardware_subarch_data;
 __u32 payload_offset;
 __u32 payload_length;
 __u64 setup_data;
 __u64 pref_address;
 __u32 init_size;
 __u32 handover_offset;
} __attribute__((packed));

struct sys_desc_table {
 __u16 length;
 __u8 table[14];
};


struct olpc_ofw_header {
 __u32 ofw_magic;
 __u32 ofw_version;
 __u32 cif_handler;
 __u32 irq_desc_table;
} __attribute__((packed));

struct efi_info {
 __u32 efi_loader_signature;
 __u32 efi_systab;
 __u32 efi_memdesc_size;
 __u32 efi_memdesc_version;
 __u32 efi_memmap;
 __u32 efi_memmap_size;
 __u32 efi_systab_hi;
 __u32 efi_memmap_hi;
};


struct boot_params {
 struct screen_info screen_info;
 struct apm_bios_info apm_bios_info;
 __u8 _pad2[4];
 __u64 tboot_addr;
 struct ist_info ist_info;
 __u8 _pad3[16];
 __u8 hd0_info[16];
 __u8 hd1_info[16];
 struct sys_desc_table sys_desc_table;
 struct olpc_ofw_header olpc_ofw_header;
 __u32 ext_ramdisk_image;
 __u32 ext_ramdisk_size;
 __u32 ext_cmd_line_ptr;
 __u8 _pad4[116];
 struct edid_info edid_info;
 struct efi_info efi_info;
 __u32 alt_mem_k;
 __u32 scratch;
 __u8 e820_entries;
 __u8 eddbuf_entries;
 __u8 edd_mbr_sig_buf_entries;
 __u8 kbd_status;
 __u8 _pad5[3];
 __u8 sentinel;
 __u8 _pad6[1];
 struct setup_header hdr;
 __u8 _pad7[0x290-0x1f1-sizeof(struct setup_header)];
 __u32 edd_mbr_sig_buffer[16];
 struct e820entry e820_map[128];
 __u8 _pad8[48];
 struct edd_info eddbuf[6];
 __u8 _pad9[276];
} __attribute__((packed));

enum {
 X86_SUBARCH_PC = 0,
 X86_SUBARCH_LGUEST,
 X86_SUBARCH_XEN,
 X86_SUBARCH_INTEL_MID,
 X86_SUBARCH_CE4100,
 X86_NR_SUBARCHS,
};

struct mpc_bus;
struct mpc_cpu;
struct mpc_table;
struct cpuinfo_x86;
struct x86_init_mpparse {
 void (*mpc_record)(unsigned int mode);
 void (*setup_ioapic_ids)(void);
 int (*mpc_apic_id)(struct mpc_cpu *m);
 void (*smp_read_mpc_oem)(struct mpc_table *mpc);
 void (*mpc_oem_pci_bus)(struct mpc_bus *m);
 void (*mpc_oem_bus_info)(struct mpc_bus *m, char *name);
 void (*find_smp_config)(void);
 void (*get_smp_config)(unsigned int early);
};
struct x86_init_resources {
 void (*probe_roms)(void);
 void (*reserve_resources)(void);
 char *(*memory_setup)(void);
};
struct x86_init_irqs {
 void (*pre_vector_init)(void);
 void (*intr_init)(void);
 void (*trap_init)(void);
};






struct x86_init_oem {
 void (*arch_setup)(void);
 void (*banner)(void);
};
struct x86_init_paging {
 void (*pagetable_init)(void);
};
struct x86_init_timers {
 void (*setup_percpu_clockev)(void);
 void (*tsc_pre_init)(void);
 void (*timer_init)(void);
 void (*wallclock_init)(void);
};





struct x86_init_iommu {
 int (*iommu_init)(void);
};
struct x86_init_pci {
 int (*arch_init)(void);
 int (*init)(void);
 void (*init_irq)(void);
 void (*fixup_irqs)(void);
};





struct x86_init_ops {
 struct x86_init_resources resources;
 struct x86_init_mpparse mpparse;
 struct x86_init_irqs irqs;
 struct x86_init_oem oem;
 struct x86_init_paging paging;
 struct x86_init_timers timers;
 struct x86_init_iommu iommu;
 struct x86_init_pci pci;
};






struct x86_cpuinit_ops {
 void (*setup_percpu_clockev)(void);
 void (*early_percpu_clock_init)(void);
 void (*fixup_cpu_id)(struct cpuinfo_x86 *c, int node);
};

struct timespec;
struct x86_platform_ops {
 unsigned long (*calibrate_tsc)(void);
 void (*get_wallclock)(struct timespec *ts);
 int (*set_wallclock)(const struct timespec *ts);
 void (*iommu_shutdown)(void);
 bool (*is_untracked_pat_range)(u64 start, u64 end);
 void (*nmi_init)(void);
 unsigned char (*get_nmi_reason)(void);
 int (*i8042_detect)(void);
 void (*save_sched_clock_state)(void);
 void (*restore_sched_clock_state)(void);
 void (*apic_post_init)(void);
};

struct pci_dev;
struct msi_msg;
struct msi_desc;

struct x86_msi_ops {
 int (*setup_msi_irqs)(struct pci_dev *dev, int nvec, int type);
 void (*compose_msi_msg)(struct pci_dev *dev, unsigned int irq,
    unsigned int dest, struct msi_msg *msg,
          u8 hpet_id);
 void (*teardown_msi_irq)(unsigned int irq);
 void (*teardown_msi_irqs)(struct pci_dev *dev);
 void (*restore_msi_irqs)(struct pci_dev *dev);
 int (*setup_hpet_msi)(unsigned int irq, unsigned int id);
 u32 (*msi_mask_irq)(struct msi_desc *desc, u32 mask, u32 flag);
 u32 (*msix_mask_irq)(struct msi_desc *desc, u32 flag);
};

struct IO_APIC_route_entry;
struct io_apic_irq_attr;
struct irq_data;
struct cpumask;

struct x86_io_apic_ops {
 void (*init) (void);
 unsigned int (*read) (unsigned int apic, unsigned int reg);
 void (*write) (unsigned int apic, unsigned int reg, unsigned int value);
 void (*modify) (unsigned int apic, unsigned int reg, unsigned int value);
 void (*disable)(void);
 void (*print_entries)(unsigned int apic, unsigned int nr_entries);
 int (*set_affinity)(struct irq_data *data,
     const struct cpumask *mask,
     bool force);
 int (*setup_entry)(int irq, struct IO_APIC_route_entry *entry,
           unsigned int destination, int vector,
           struct io_apic_irq_attr *attr);
 void (*eoi_ioapic_pin)(int apic, int pin, int vector);
};

extern struct x86_init_ops x86_init;
extern struct x86_cpuinit_ops x86_cpuinit;
extern struct x86_platform_ops x86_platform;
extern struct x86_msi_ops x86_msi;
extern struct x86_io_apic_ops x86_io_apic_ops;
extern void x86_init_noop(void);
extern void x86_init_uint_noop(unsigned int unused);
struct local_apic {

        struct { unsigned int __reserved[4]; } __reserved_01;

        struct { unsigned int __reserved[4]; } __reserved_02;

        struct {
  unsigned int __reserved_1 : 24,
   phys_apic_id : 4,
   __reserved_2 : 4;
  unsigned int __reserved[3];
 } id;

        const
 struct {
  unsigned int version : 8,
   __reserved_1 : 8,
   max_lvt : 8,
   __reserved_2 : 8;
  unsigned int __reserved[3];
 } version;

        struct { unsigned int __reserved[4]; } __reserved_03;

        struct { unsigned int __reserved[4]; } __reserved_04;

        struct { unsigned int __reserved[4]; } __reserved_05;

        struct { unsigned int __reserved[4]; } __reserved_06;

        struct {
  unsigned int priority : 8,
   __reserved_1 : 24;
  unsigned int __reserved_2[3];
 } tpr;

        const
 struct {
  unsigned int priority : 8,
   __reserved_1 : 24;
  unsigned int __reserved_2[3];
 } apr;

        const
 struct {
  unsigned int priority : 8,
   __reserved_1 : 24;
  unsigned int __reserved_2[3];
 } ppr;

        struct {
  unsigned int eoi;
  unsigned int __reserved[3];
 } eoi;

        struct { unsigned int __reserved[4]; } __reserved_07;

        struct {
  unsigned int __reserved_1 : 24,
   logical_dest : 8;
  unsigned int __reserved_2[3];
 } ldr;

        struct {
  unsigned int __reserved_1 : 28,
   model : 4;
  unsigned int __reserved_2[3];
 } dfr;

        struct {
  unsigned int spurious_vector : 8,
   apic_enabled : 1,
   focus_cpu : 1,
   __reserved_2 : 22;
  unsigned int __reserved_3[3];
 } svr;

        struct {
         unsigned int bitfield;
  unsigned int __reserved[3];
 } isr [8];

        struct {
         unsigned int bitfield;
  unsigned int __reserved[3];
 } tmr [8];

        struct {
         unsigned int bitfield;
  unsigned int __reserved[3];
 } irr [8];

        union {
  struct {
   unsigned int send_cs_error : 1,
    receive_cs_error : 1,
    send_accept_error : 1,
    receive_accept_error : 1,
    __reserved_1 : 1,
    send_illegal_vector : 1,
    receive_illegal_vector : 1,
    illegal_register_address : 1,
    __reserved_2 : 24;
   unsigned int __reserved_3[3];
  } error_bits;
  struct {
   unsigned int errors;
   unsigned int __reserved_3[3];
  } all_errors;
 } esr;

        struct { unsigned int __reserved[4]; } __reserved_08;

        struct { unsigned int __reserved[4]; } __reserved_09;

        struct { unsigned int __reserved[4]; } __reserved_10;

        struct { unsigned int __reserved[4]; } __reserved_11;

        struct { unsigned int __reserved[4]; } __reserved_12;

        struct { unsigned int __reserved[4]; } __reserved_13;

        struct { unsigned int __reserved[4]; } __reserved_14;

        struct {
  unsigned int vector : 8,
   delivery_mode : 3,
   destination_mode : 1,
   delivery_status : 1,
   __reserved_1 : 1,
   level : 1,
   trigger : 1,
   __reserved_2 : 2,
   shorthand : 2,
   __reserved_3 : 12;
  unsigned int __reserved_4[3];
 } icr1;

        struct {
  union {
   unsigned int __reserved_1 : 24,
    phys_dest : 4,
    __reserved_2 : 4;
   unsigned int __reserved_3 : 24,
    logical_dest : 8;
  } dest;
  unsigned int __reserved_4[3];
 } icr2;

        struct {
  unsigned int vector : 8,
   __reserved_1 : 4,
   delivery_status : 1,
   __reserved_2 : 3,
   mask : 1,
   timer_mode : 1,
   __reserved_3 : 14;
  unsigned int __reserved_4[3];
 } lvt_timer;

        struct {
  unsigned int vector : 8,
   delivery_mode : 3,
   __reserved_1 : 1,
   delivery_status : 1,
   __reserved_2 : 3,
   mask : 1,
   __reserved_3 : 15;
  unsigned int __reserved_4[3];
 } lvt_thermal;

        struct {
  unsigned int vector : 8,
   delivery_mode : 3,
   __reserved_1 : 1,
   delivery_status : 1,
   __reserved_2 : 3,
   mask : 1,
   __reserved_3 : 15;
  unsigned int __reserved_4[3];
 } lvt_pc;

        struct {
  unsigned int vector : 8,
   delivery_mode : 3,
   __reserved_1 : 1,
   delivery_status : 1,
   polarity : 1,
   remote_irr : 1,
   trigger : 1,
   mask : 1,
   __reserved_2 : 15;
  unsigned int __reserved_3[3];
 } lvt_lint0;

        struct {
  unsigned int vector : 8,
   delivery_mode : 3,
   __reserved_1 : 1,
   delivery_status : 1,
   polarity : 1,
   remote_irr : 1,
   trigger : 1,
   mask : 1,
   __reserved_2 : 15;
  unsigned int __reserved_3[3];
 } lvt_lint1;

        struct {
  unsigned int vector : 8,
   __reserved_1 : 4,
   delivery_status : 1,
   __reserved_2 : 3,
   mask : 1,
   __reserved_3 : 15;
  unsigned int __reserved_4[3];
 } lvt_error;

        struct {
  unsigned int initial_count;
  unsigned int __reserved_2[3];
 } timer_icr;

        const
 struct {
  unsigned int curr_count;
  unsigned int __reserved_2[3];
 } timer_ccr;

        struct { unsigned int __reserved[4]; } __reserved_16;

        struct { unsigned int __reserved[4]; } __reserved_17;

        struct { unsigned int __reserved[4]; } __reserved_18;

        struct { unsigned int __reserved[4]; } __reserved_19;

        struct {
  unsigned int divisor : 4,
   __reserved_1 : 28;
  unsigned int __reserved_2[3];
 } timer_dcr;

        struct { unsigned int __reserved[4]; } __reserved_20;

} __attribute__ ((packed));
enum ioapic_irq_destination_types {
 dest_Fixed = 0,
 dest_LowestPrio = 1,
 dest_SMI = 2,
 dest__reserved_1 = 3,
 dest_NMI = 4,
 dest_INIT = 5,
 dest__reserved_2 = 6,
 dest_ExtINT = 7
};

extern int apic_version[];
extern int pic_mode;
extern unsigned long mp_bus_not_pci[(((256) + (8 * sizeof(long)) - 1) / (8 * sizeof(long)))];

extern unsigned int boot_cpu_physical_apicid;
extern unsigned long mp_lapic_addr;


extern int smp_found_config;




static inline __attribute__((no_instrument_function)) void get_smp_config(void)
{
 x86_init.mpparse.get_smp_config(0);
}

static inline __attribute__((no_instrument_function)) void early_get_smp_config(void)
{
 x86_init.mpparse.get_smp_config(1);
}

static inline __attribute__((no_instrument_function)) void find_smp_config(void)
{
 x86_init.mpparse.find_smp_config();
}


extern void early_reserve_e820_mpc_new(void);
extern int enable_update_mptable;
extern int default_mpc_apic_id(struct mpc_cpu *m);
extern void default_smp_read_mpc_oem(struct mpc_table *mpc);

extern void default_mpc_oem_bus_info(struct mpc_bus *m, char *str);



extern void default_find_smp_config(void);
extern void default_get_smp_config(unsigned int early);
int generic_processor_info(int apicid, int version);



struct physid_mask {
 unsigned long mask[(((32768) + (8 * sizeof(long)) - 1) / (8 * sizeof(long)))];
};

typedef struct physid_mask physid_mask_t;
static inline __attribute__((no_instrument_function)) unsigned long physids_coerce(physid_mask_t *map)
{
 return map->mask[0];
}

static inline __attribute__((no_instrument_function)) void physids_promote(unsigned long physids, physid_mask_t *map)
{
 bitmap_zero((*map).mask, 32768);
 map->mask[0] = physids;
}

static inline __attribute__((no_instrument_function)) void physid_set_mask_of_physid(int physid, physid_mask_t *map)
{
 bitmap_zero((*map).mask, 32768);
 set_bit(physid, (*map).mask);
}




extern physid_mask_t phys_cpu_present_map;




extern void (*pm_power_off)(void);
extern void (*pm_power_off_prepare)(void);

struct device;

extern void pm_vt_switch_required(struct device *dev, bool required);
extern void pm_vt_switch_unregister(struct device *dev);
struct device;


extern const char power_group_name[];




typedef struct pm_message {
 int event;
} pm_message_t;
struct dev_pm_ops {
 int (*prepare)(struct device *dev);
 void (*complete)(struct device *dev);
 int (*suspend)(struct device *dev);
 int (*resume)(struct device *dev);
 int (*freeze)(struct device *dev);
 int (*thaw)(struct device *dev);
 int (*poweroff)(struct device *dev);
 int (*restore)(struct device *dev);
 int (*suspend_late)(struct device *dev);
 int (*resume_early)(struct device *dev);
 int (*freeze_late)(struct device *dev);
 int (*thaw_early)(struct device *dev);
 int (*poweroff_late)(struct device *dev);
 int (*restore_early)(struct device *dev);
 int (*suspend_noirq)(struct device *dev);
 int (*resume_noirq)(struct device *dev);
 int (*freeze_noirq)(struct device *dev);
 int (*thaw_noirq)(struct device *dev);
 int (*poweroff_noirq)(struct device *dev);
 int (*restore_noirq)(struct device *dev);
 int (*runtime_suspend)(struct device *dev);
 int (*runtime_resume)(struct device *dev);
 int (*runtime_idle)(struct device *dev);
};
enum rpm_status {
 RPM_ACTIVE = 0,
 RPM_RESUMING,
 RPM_SUSPENDED,
 RPM_SUSPENDING,
};
enum rpm_request {
 RPM_REQ_NONE = 0,
 RPM_REQ_IDLE,
 RPM_REQ_SUSPEND,
 RPM_REQ_AUTOSUSPEND,
 RPM_REQ_RESUME,
};

struct wakeup_source;

struct pm_domain_data {
 struct list_head list_node;
 struct device *dev;
};

struct pm_subsys_data {
 spinlock_t lock;
 unsigned int refcount;






};

struct dev_pm_info {
 pm_message_t power_state;
 unsigned int can_wakeup:1;
 unsigned int async_suspend:1;
 bool is_prepared:1;
 bool is_suspended:1;
 bool is_noirq_suspended:1;
 bool is_late_suspended:1;
 bool ignore_children:1;
 bool early_init:1;
 bool direct_complete:1;
 spinlock_t lock;

 struct list_head entry;
 struct completion completion;
 struct wakeup_source *wakeup;
 bool wakeup_path:1;
 bool syscore:1;




 struct timer_list suspend_timer;
 unsigned long timer_expires;
 struct work_struct work;
 wait_queue_head_t wait_queue;
 atomic_t usage_count;
 atomic_t child_count;
 unsigned int disable_depth:3;
 unsigned int idle_notification:1;
 unsigned int request_pending:1;
 unsigned int deferred_resume:1;
 unsigned int run_wake:1;
 unsigned int runtime_auto:1;
 unsigned int no_callbacks:1;
 unsigned int irq_safe:1;
 unsigned int use_autosuspend:1;
 unsigned int timer_autosuspends:1;
 unsigned int memalloc_noio:1;
 enum rpm_request request;
 enum rpm_status runtime_status;
 int runtime_error;
 int autosuspend_delay;
 unsigned long last_busy;
 unsigned long active_jiffies;
 unsigned long suspended_jiffies;
 unsigned long accounting_timestamp;

 struct pm_subsys_data *subsys_data;
 void (*set_latency_tolerance)(struct device *, s32);
 struct dev_pm_qos *qos;
};

extern void update_pm_runtime_accounting(struct device *dev);
extern int dev_pm_get_subsys_data(struct device *dev);
extern int dev_pm_put_subsys_data(struct device *dev);






struct dev_pm_domain {
 struct dev_pm_ops ops;
};
extern void device_pm_lock(void);
extern void dpm_resume_start(pm_message_t state);
extern void dpm_resume_end(pm_message_t state);
extern void dpm_resume(pm_message_t state);
extern void dpm_complete(pm_message_t state);

extern void device_pm_unlock(void);
extern int dpm_suspend_end(pm_message_t state);
extern int dpm_suspend_start(pm_message_t state);
extern int dpm_suspend(pm_message_t state);
extern int dpm_prepare(pm_message_t state);

extern void __suspend_report_result(const char *function, void *fn, int ret);






extern int device_pm_wait_for_dev(struct device *sub, struct device *dev);
extern void dpm_for_each_dev(void *data, void (*fn)(struct device *, void *));

extern int pm_generic_prepare(struct device *dev);
extern int pm_generic_suspend_late(struct device *dev);
extern int pm_generic_suspend_noirq(struct device *dev);
extern int pm_generic_suspend(struct device *dev);
extern int pm_generic_resume_early(struct device *dev);
extern int pm_generic_resume_noirq(struct device *dev);
extern int pm_generic_resume(struct device *dev);
extern int pm_generic_freeze_noirq(struct device *dev);
extern int pm_generic_freeze_late(struct device *dev);
extern int pm_generic_freeze(struct device *dev);
extern int pm_generic_thaw_noirq(struct device *dev);
extern int pm_generic_thaw_early(struct device *dev);
extern int pm_generic_thaw(struct device *dev);
extern int pm_generic_restore_noirq(struct device *dev);
extern int pm_generic_restore_early(struct device *dev);
extern int pm_generic_restore(struct device *dev);
extern int pm_generic_poweroff_noirq(struct device *dev);
extern int pm_generic_poweroff_late(struct device *dev);
extern int pm_generic_poweroff(struct device *dev);
extern void pm_generic_complete(struct device *dev);
enum dpm_order {
 DPM_ORDER_NONE,
 DPM_ORDER_DEV_AFTER_PARENT,
 DPM_ORDER_PARENT_BEFORE_DEV,
 DPM_ORDER_DEV_LAST,
};












extern __attribute__((section(".data..percpu" ""))) __typeof__(int) x86_cpu_to_node_map; extern __typeof__(int) *x86_cpu_to_node_map_early_ptr; extern __typeof__(int) x86_cpu_to_node_map_early_map[];
static inline __attribute__((no_instrument_function)) int early_cpu_to_node(int cpu)
{
 return *((x86_cpu_to_node_map_early_ptr) ? &(x86_cpu_to_node_map_early_ptr)[cpu] : &(*({ do { const void *__vpp_verify = (typeof((&(x86_cpu_to_node_map)) + 0))((void *)0); (void)__vpp_verify; } while (0); ({ unsigned long __ptr; __asm__ ("" : "=r"(__ptr) : "0"((typeof(*((&(x86_cpu_to_node_map)))) *)((&(x86_cpu_to_node_map))))); (typeof((typeof(*((&(x86_cpu_to_node_map)))) *)((&(x86_cpu_to_node_map))))) (__ptr + (((__per_cpu_offset[(cpu)])))); }); })));
}




extern cpumask_var_t node_to_cpumask_map[(1 << 6)];





static inline __attribute__((no_instrument_function)) const struct cpumask *cpumask_of_node(int node)
{
 return node_to_cpumask_map[node];
}


extern void setup_node_to_cpumask_map(void);
extern int __node_distance(int, int);

extern const struct cpumask *cpu_coregroup_mask(int cpu);
static inline __attribute__((no_instrument_function)) void arch_fix_phys_package_id(int num, u32 slot)
{
}

struct pci_bus;
int x86_pci_root_bus_node(int bus);
void x86_pci_root_bus_resources(int bus, struct list_head *resources);
extern int numa_off;
extern s16 __apicid_to_node[32768];
extern nodemask_t numa_nodes_parsed __attribute__ ((__section__(".init.data")));

extern int __attribute__ ((__section__(".init.text"))) __attribute__((__cold__)) __attribute__((no_instrument_function)) numa_add_memblk(int nodeid, u64 start, u64 end);
extern void __attribute__ ((__section__(".init.text"))) __attribute__((__cold__)) __attribute__((no_instrument_function)) numa_set_distance(int from, int to, int distance);

static inline __attribute__((no_instrument_function)) void set_apicid_to_node(int apicid, s16 node)
{
 __apicid_to_node[apicid] = node;
}

extern int numa_cpu_node(int cpu);
extern void numa_set_node(int cpu, int node);
extern void numa_clear_node(int cpu);
extern void __attribute__ ((__section__(".init.text"))) __attribute__((__cold__)) __attribute__((no_instrument_function)) init_cpu_to_node(void);
extern void numa_add_cpu(int cpu);
extern void numa_remove_cpu(int cpu);

typedef struct {
 void *ldt;
 int size;



 unsigned short ia32_compat;


 struct mutex lock;
 void *vdso;
} mm_context_t;


void leave_mm(int cpu);





extern void *early_ioremap(resource_size_t phys_addr,
       unsigned long size);
extern void *early_memremap(resource_size_t phys_addr,
       unsigned long size);
extern void early_iounmap(void *addr, unsigned long size);
extern void early_memunmap(void *addr, unsigned long size);





extern void early_ioremap_shutdown(void);



extern void early_ioremap_init(void);


extern void early_ioremap_setup(void);





extern void early_ioremap_reset(void);
static inline __attribute__((no_instrument_function)) unsigned char readb(const volatile void *addr) { unsigned char ret; asm volatile("mov" "b" " %1,%0":"=q" (ret) :"m" (*(volatile unsigned char *)addr) :"memory"); return ret; }
static inline __attribute__((no_instrument_function)) unsigned short readw(const volatile void *addr) { unsigned short ret; asm volatile("mov" "w" " %1,%0":"=r" (ret) :"m" (*(volatile unsigned short *)addr) :"memory"); return ret; }
static inline __attribute__((no_instrument_function)) unsigned int readl(const volatile void *addr) { unsigned int ret; asm volatile("mov" "l" " %1,%0":"=r" (ret) :"m" (*(volatile unsigned int *)addr) :"memory"); return ret; }

static inline __attribute__((no_instrument_function)) unsigned char __readb(const volatile void *addr) { unsigned char ret; asm volatile("mov" "b" " %1,%0":"=q" (ret) :"m" (*(volatile unsigned char *)addr) ); return ret; }
static inline __attribute__((no_instrument_function)) unsigned short __readw(const volatile void *addr) { unsigned short ret; asm volatile("mov" "w" " %1,%0":"=r" (ret) :"m" (*(volatile unsigned short *)addr) ); return ret; }
static inline __attribute__((no_instrument_function)) unsigned int __readl(const volatile void *addr) { unsigned int ret; asm volatile("mov" "l" " %1,%0":"=r" (ret) :"m" (*(volatile unsigned int *)addr) ); return ret; }

static inline __attribute__((no_instrument_function)) void writeb(unsigned char val, volatile void *addr) { asm volatile("mov" "b" " %0,%1": :"q" (val), "m" (*(volatile unsigned char *)addr) :"memory"); }
static inline __attribute__((no_instrument_function)) void writew(unsigned short val, volatile void *addr) { asm volatile("mov" "w" " %0,%1": :"r" (val), "m" (*(volatile unsigned short *)addr) :"memory"); }
static inline __attribute__((no_instrument_function)) void writel(unsigned int val, volatile void *addr) { asm volatile("mov" "l" " %0,%1": :"r" (val), "m" (*(volatile unsigned int *)addr) :"memory"); }

static inline __attribute__((no_instrument_function)) void __writeb(unsigned char val, volatile void *addr) { asm volatile("mov" "b" " %0,%1": :"q" (val), "m" (*(volatile unsigned char *)addr) ); }
static inline __attribute__((no_instrument_function)) void __writew(unsigned short val, volatile void *addr) { asm volatile("mov" "w" " %0,%1": :"r" (val), "m" (*(volatile unsigned short *)addr) ); }
static inline __attribute__((no_instrument_function)) void __writel(unsigned int val, volatile void *addr) { asm volatile("mov" "l" " %0,%1": :"r" (val), "m" (*(volatile unsigned int *)addr) ); }
static inline __attribute__((no_instrument_function)) unsigned long readq(const volatile void *addr) { unsigned long ret; asm volatile("mov" "q" " %1,%0":"=r" (ret) :"m" (*(volatile unsigned long *)addr) :"memory"); return ret; }
static inline __attribute__((no_instrument_function)) void writeq(unsigned long val, volatile void *addr) { asm volatile("mov" "q" " %0,%1": :"r" (val), "m" (*(volatile unsigned long *)addr) :"memory"); }
static inline __attribute__((no_instrument_function)) phys_addr_t virt_to_phys(volatile void *address)
{
 return __phys_addr_nodebug((unsigned long)(address));
}
static inline __attribute__((no_instrument_function)) void *phys_to_virt(phys_addr_t address)
{
 return ((void *)((unsigned long)(address)+((unsigned long)(0xffff880000000000UL))));
}
static inline __attribute__((no_instrument_function)) unsigned int isa_virt_to_bus(volatile void *address)
{
 return (unsigned int)virt_to_phys(address);
}
extern void *ioremap_nocache(resource_size_t offset, unsigned long size);
extern void *ioremap_cache(resource_size_t offset, unsigned long size);
extern void *ioremap_prot(resource_size_t offset, unsigned long size,
    unsigned long prot_val);




static inline __attribute__((no_instrument_function)) void *ioremap(resource_size_t offset, unsigned long size)
{
 return ioremap_nocache(offset, size);
}

extern void iounmap(volatile void *addr);

extern void set_iounmap_nonlazy(void);



extern unsigned int ioread8(void *);
extern unsigned int ioread16(void *);
extern unsigned int ioread16be(void *);
extern unsigned int ioread32(void *);
extern unsigned int ioread32be(void *);

extern void iowrite8(u8, void *);
extern void iowrite16(u16, void *);
extern void iowrite16be(u16, void *);
extern void iowrite32(u32, void *);
extern void iowrite32be(u32, void *);
extern void ioread8_rep(void *port, void *buf, unsigned long count);
extern void ioread16_rep(void *port, void *buf, unsigned long count);
extern void ioread32_rep(void *port, void *buf, unsigned long count);

extern void iowrite8_rep(void *port, const void *buf, unsigned long count);
extern void iowrite16_rep(void *port, const void *buf, unsigned long count);
extern void iowrite32_rep(void *port, const void *buf, unsigned long count);



extern void *ioport_map(unsigned long port, unsigned int nr);
extern void ioport_unmap(void *);
struct pci_dev;
extern void pci_iounmap(struct pci_dev *dev, void *);






struct pci_dev;


extern void *pci_iomap(struct pci_dev *dev, int bar, unsigned long max);








struct rb_node {
 unsigned long __rb_parent_color;
 struct rb_node *rb_right;
 struct rb_node *rb_left;
} __attribute__((aligned(sizeof(long))));


struct rb_root {
 struct rb_node *rb_node;
};
extern void rb_insert_color(struct rb_node *, struct rb_root *);
extern void rb_erase(struct rb_node *, struct rb_root *);



extern struct rb_node *rb_next(const struct rb_node *);
extern struct rb_node *rb_prev(const struct rb_node *);
extern struct rb_node *rb_first(const struct rb_root *);
extern struct rb_node *rb_last(const struct rb_root *);


extern struct rb_node *rb_first_postorder(const struct rb_root *);
extern struct rb_node *rb_next_postorder(const struct rb_node *);


extern void rb_replace_node(struct rb_node *victim, struct rb_node *new,
       struct rb_root *root);

static inline __attribute__((no_instrument_function)) void rb_link_node(struct rb_node * node, struct rb_node * parent,
    struct rb_node ** rb_link)
{
 node->__rb_parent_color = (unsigned long)parent;
 node->rb_left = node->rb_right = ((void *)0);

 *rb_link = node;
}

struct vm_area_struct;
struct vm_struct {
 struct vm_struct *next;
 void *addr;
 unsigned long size;
 unsigned long flags;
 struct page **pages;
 unsigned int nr_pages;
 phys_addr_t phys_addr;
 const void *caller;
};

struct vmap_area {
 unsigned long va_start;
 unsigned long va_end;
 unsigned long flags;
 struct rb_node rb_node;
 struct list_head list;
 struct list_head purge_list;
 struct vm_struct *vm;
 struct callback_head callback_head;
};




extern void vm_unmap_ram(const void *mem, unsigned int count);
extern void *vm_map_ram(struct page **pages, unsigned int count,
    int node, pgprot_t prot);
extern void vm_unmap_aliases(void);


extern void __attribute__ ((__section__(".init.text"))) __attribute__((__cold__)) __attribute__((no_instrument_function)) vmalloc_init(void);






extern void *vmalloc(unsigned long size);
extern void *vzalloc(unsigned long size);
extern void *vmalloc_user(unsigned long size);
extern void *vmalloc_node(unsigned long size, int node);
extern void *vzalloc_node(unsigned long size, int node);
extern void *vmalloc_exec(unsigned long size);
extern void *vmalloc_32(unsigned long size);
extern void *vmalloc_32_user(unsigned long size);
extern void *__vmalloc(unsigned long size, gfp_t gfp_mask, pgprot_t prot);
extern void *__vmalloc_node_range(unsigned long size, unsigned long align,
   unsigned long start, unsigned long end, gfp_t gfp_mask,
   pgprot_t prot, int node, const void *caller);
extern void vfree(const void *addr);

extern void *vmap(struct page **pages, unsigned int count,
   unsigned long flags, pgprot_t prot);
extern void vunmap(const void *addr);

extern int remap_vmalloc_range_partial(struct vm_area_struct *vma,
           unsigned long uaddr, void *kaddr,
           unsigned long size);

extern int remap_vmalloc_range(struct vm_area_struct *vma, void *addr,
       unsigned long pgoff);
void vmalloc_sync_all(void);





static inline __attribute__((no_instrument_function)) size_t get_vm_area_size(const struct vm_struct *area)
{

 return area->size - ((1UL) << 12);
}

extern struct vm_struct *get_vm_area(unsigned long size, unsigned long flags);
extern struct vm_struct *get_vm_area_caller(unsigned long size,
     unsigned long flags, const void *caller);
extern struct vm_struct *__get_vm_area(unsigned long size, unsigned long flags,
     unsigned long start, unsigned long end);
extern struct vm_struct *__get_vm_area_caller(unsigned long size,
     unsigned long flags,
     unsigned long start, unsigned long end,
     const void *caller);
extern struct vm_struct *remove_vm_area(const void *addr);
extern struct vm_struct *find_vm_area(const void *addr);

extern int map_vm_area(struct vm_struct *area, pgprot_t prot,
   struct page **pages);

extern int map_kernel_range_noflush(unsigned long start, unsigned long size,
        pgprot_t prot, struct page **pages);
extern void unmap_kernel_range_noflush(unsigned long addr, unsigned long size);
extern void unmap_kernel_range(unsigned long addr, unsigned long size);
extern struct vm_struct *alloc_vm_area(size_t size, pte_t **ptes);
extern void free_vm_area(struct vm_struct *area);


extern long vread(char *buf, char *addr, unsigned long count);
extern long vwrite(char *buf, char *addr, unsigned long count);




extern struct list_head vmap_area_list;
extern __attribute__ ((__section__(".init.text"))) __attribute__((__cold__)) __attribute__((no_instrument_function)) void vm_area_add_early(struct vm_struct *vm);
extern __attribute__ ((__section__(".init.text"))) __attribute__((__cold__)) __attribute__((no_instrument_function)) void vm_area_register_early(struct vm_struct *vm, size_t align);



struct vm_struct **pcpu_get_vm_areas(const unsigned long *offsets,
         const size_t *sizes, int nr_vms,
         size_t align);

void pcpu_free_vm_areas(struct vm_struct **vms, int nr_vms);
struct vmalloc_info {
 unsigned long used;
 unsigned long largest_chunk;
};



extern void get_vmalloc_info(struct vmalloc_info *vmi);






static inline __attribute__((no_instrument_function)) void
memset_io(volatile void *addr, unsigned char val, size_t count)
{
 memset((void *)addr, val, count);
}

static inline __attribute__((no_instrument_function)) void
memcpy_fromio(void *dst, const volatile void *src, size_t count)
{
 memcpy(dst, (const void *)src, count);
}

static inline __attribute__((no_instrument_function)) void
memcpy_toio(volatile void *dst, const void *src, size_t count)
{
 memcpy((void *)dst, src, count);
}
static inline __attribute__((no_instrument_function)) void flush_write_buffers(void)
{



}



extern void native_io_delay(void);

extern int io_delay_type;
extern void io_delay_init(void);





static inline __attribute__((no_instrument_function)) void slow_down_io(void)
{
 native_io_delay();





}
static inline __attribute__((no_instrument_function)) void outb(unsigned char value, int port) { asm volatile("out" "b" " %" "b" "0, %w1" : : "a"(value), "Nd"(port)); } static inline __attribute__((no_instrument_function)) unsigned char inb(int port) { unsigned char value; asm volatile("in" "b" " %w1, %" "b" "0" : "=a"(value) : "Nd"(port)); return value; } static inline __attribute__((no_instrument_function)) void outb_p(unsigned char value, int port) { outb(value, port); slow_down_io(); } static inline __attribute__((no_instrument_function)) unsigned char inb_p(int port) { unsigned char value = inb(port); slow_down_io(); return value; } static inline __attribute__((no_instrument_function)) void outsb(int port, const void *addr, unsigned long count) { asm volatile("rep; outs" "b" : "+S"(addr), "+c"(count) : "d"(port)); } static inline __attribute__((no_instrument_function)) void insb(int port, void *addr, unsigned long count) { asm volatile("rep; ins" "b" : "+D"(addr), "+c"(count) : "d"(port)); }
static inline __attribute__((no_instrument_function)) void outw(unsigned short value, int port) { asm volatile("out" "w" " %" "w" "0, %w1" : : "a"(value), "Nd"(port)); } static inline __attribute__((no_instrument_function)) unsigned short inw(int port) { unsigned short value; asm volatile("in" "w" " %w1, %" "w" "0" : "=a"(value) : "Nd"(port)); return value; } static inline __attribute__((no_instrument_function)) void outw_p(unsigned short value, int port) { outw(value, port); slow_down_io(); } static inline __attribute__((no_instrument_function)) unsigned short inw_p(int port) { unsigned short value = inw(port); slow_down_io(); return value; } static inline __attribute__((no_instrument_function)) void outsw(int port, const void *addr, unsigned long count) { asm volatile("rep; outs" "w" : "+S"(addr), "+c"(count) : "d"(port)); } static inline __attribute__((no_instrument_function)) void insw(int port, void *addr, unsigned long count) { asm volatile("rep; ins" "w" : "+D"(addr), "+c"(count) : "d"(port)); }
static inline __attribute__((no_instrument_function)) void outl(unsigned int value, int port) { asm volatile("out" "l" " %" "" "0, %w1" : : "a"(value), "Nd"(port)); } static inline __attribute__((no_instrument_function)) unsigned int inl(int port) { unsigned int value; asm volatile("in" "l" " %w1, %" "" "0" : "=a"(value) : "Nd"(port)); return value; } static inline __attribute__((no_instrument_function)) void outl_p(unsigned int value, int port) { outl(value, port); slow_down_io(); } static inline __attribute__((no_instrument_function)) unsigned int inl_p(int port) { unsigned int value = inl(port); slow_down_io(); return value; } static inline __attribute__((no_instrument_function)) void outsl(int port, const void *addr, unsigned long count) { asm volatile("rep; outs" "l" : "+S"(addr), "+c"(count) : "d"(port)); } static inline __attribute__((no_instrument_function)) void insl(int port, void *addr, unsigned long count) { asm volatile("rep; ins" "l" : "+D"(addr), "+c"(count) : "d"(port)); }

extern void *xlate_dev_mem_ptr(unsigned long phys);
extern void unxlate_dev_mem_ptr(unsigned long phys, void *addr);

extern int ioremap_change_attr(unsigned long vaddr, unsigned long size,
    unsigned long prot_val);
extern void *ioremap_wc(resource_size_t offset, unsigned long size);

extern bool is_early_ioremap_ptep(pte_t *ptep);
extern int arch_phys_wc_add(unsigned long base,
      unsigned long size);
extern void arch_phys_wc_del(int handle);


struct real_mode_header {
 u32 text_start;
 u32 ro_end;

 u32 trampoline_start;
 u32 trampoline_status;
 u32 trampoline_header;

 u32 trampoline_pgd;



 u32 wakeup_start;
 u32 wakeup_header;


 u32 machine_real_restart_asm;

 u32 machine_real_restart_seg;

};


struct trampoline_header {






 u64 start;
 u64 efer;
 u32 cr4;

};

extern struct real_mode_header *real_mode_header;
extern unsigned char real_mode_blob_end[];

extern unsigned long init_rsp;
extern unsigned long initial_code;
extern unsigned long initial_gs;

extern unsigned char real_mode_blob[];
extern unsigned char real_mode_relocs[];





extern unsigned char secondary_startup_64[];


void reserve_real_mode(void);
void setup_real_mode(void);


extern int acpi_lapic;
extern int acpi_ioapic;
extern int acpi_noirq;
extern int acpi_strict;
extern int acpi_disabled;
extern int acpi_pci_disabled;
extern int acpi_skip_timer_override;
extern int acpi_use_timer_override;
extern int acpi_fix_pin2_polarity;
extern int acpi_disable_cmcff;

extern u8 acpi_sci_flags;
extern int acpi_sci_override_gsi;
void acpi_pic_sci_set_trigger(unsigned int, u16);

extern int (*__acpi_register_gsi)(struct device *dev, u32 gsi,
      int trigger, int polarity);

static inline __attribute__((no_instrument_function)) void disable_acpi(void)
{
 acpi_disabled = 1;
 acpi_pci_disabled = 1;
 acpi_noirq = 1;
}

extern int acpi_gsi_to_irq(u32 gsi, unsigned int *irq);

static inline __attribute__((no_instrument_function)) void acpi_noirq_set(void) { acpi_noirq = 1; }
static inline __attribute__((no_instrument_function)) void acpi_disable_pci(void)
{
 acpi_pci_disabled = 1;
 acpi_noirq_set();
}


extern int (*acpi_suspend_lowlevel)(void);







static inline __attribute__((no_instrument_function)) unsigned int acpi_processor_cstate_check(unsigned int max_cstate)
{






 if (boot_cpu_data.x86 == 0x0F &&
     boot_cpu_data.x86_vendor == 2 &&
     boot_cpu_data.x86_model <= 0x05 &&
     boot_cpu_data.x86_mask < 0x0A)
  return 1;
 else if (amd_e400_c1e_detected)
  return 1;
 else
  return max_cstate;
}

static inline __attribute__((no_instrument_function)) bool arch_has_acpi_pdc(void)
{
 struct cpuinfo_x86 *c = &(*({ do { const void *__vpp_verify = (typeof((&(cpu_info)) + 0))((void *)0); (void)__vpp_verify; } while (0); ({ unsigned long __ptr; __asm__ ("" : "=r"(__ptr) : "0"((typeof(*((&(cpu_info)))) *)((&(cpu_info))))); (typeof((typeof(*((&(cpu_info)))) *)((&(cpu_info))))) (__ptr + (((__per_cpu_offset[(0)])))); }); }));
 return (c->x86_vendor == 0 ||
  c->x86_vendor == 5);
}

static inline __attribute__((no_instrument_function)) void arch_acpi_set_pdc_bits(u32 *buf)
{
 struct cpuinfo_x86 *c = &(*({ do { const void *__vpp_verify = (typeof((&(cpu_info)) + 0))((void *)0); (void)__vpp_verify; } while (0); ({ unsigned long __ptr; __asm__ ("" : "=r"(__ptr) : "0"((typeof(*((&(cpu_info)))) *)((&(cpu_info))))); (typeof((typeof(*((&(cpu_info)))) *)((&(cpu_info))))) (__ptr + (((__per_cpu_offset[(0)])))); }); }));

 buf[2] |= ((0x0010) | (0x0008) | (0x0002) | (0x0100) | (0x0200));

 if ((__builtin_constant_p(( 4*32+ 7)) && ( (((( 4*32+ 7))>>5)==0 && (1UL<<((( 4*32+ 7))&31) & ((1<<(( 0*32+ 0) & 31))|(1<<(( 0*32+ 3)) & 31)|(1<<(( 0*32+ 5) & 31))|(1<<(( 0*32+ 6) & 31))| (1<<(( 0*32+ 8) & 31))|(1<<(( 0*32+13)) & 31)|(1<<(( 0*32+24) & 31))|(1<<(( 0*32+15) & 31))| (1<<(( 0*32+25) & 31))|(1<<(( 0*32+26) & 31))))) || (((( 4*32+ 7))>>5)==1 && (1UL<<((( 4*32+ 7))&31) & ((1<<(( 1*32+29) & 31))|0))) || (((( 4*32+ 7))>>5)==2 && (1UL<<((( 4*32+ 7))&31) & 0)) || (((( 4*32+ 7))>>5)==3 && (1UL<<((( 4*32+ 7))&31) & ((1<<(( 3*32+20) & 31))))) || (((( 4*32+ 7))>>5)==4 && (1UL<<((( 4*32+ 7))&31) & (0))) || (((( 4*32+ 7))>>5)==5 && (1UL<<((( 4*32+ 7))&31) & 0)) || (((( 4*32+ 7))>>5)==6 && (1UL<<((( 4*32+ 7))&31) & 0)) || (((( 4*32+ 7))>>5)==7 && (1UL<<((( 4*32+ 7))&31) & 0)) || (((( 4*32+ 7))>>5)==8 && (1UL<<((( 4*32+ 7))&31) & 0)) || (((( 4*32+ 7))>>5)==9 && (1UL<<((( 4*32+ 7))&31) & 0)) ) ? 1 : (__builtin_constant_p((( 4*32+ 7))) ? constant_test_bit((( 4*32+ 7)), ((unsigned long *)((c)->x86_capability))) : variable_test_bit((( 4*32+ 7)), ((unsigned long *)((c)->x86_capability))))))
  buf[2] |= ((0x0008) | (0x0002) | (0x0020) | (0x0800) | (0x0001));

 if ((__builtin_constant_p(( 0*32+22)) && ( (((( 0*32+22))>>5)==0 && (1UL<<((( 0*32+22))&31) & ((1<<(( 0*32+ 0) & 31))|(1<<(( 0*32+ 3)) & 31)|(1<<(( 0*32+ 5) & 31))|(1<<(( 0*32+ 6) & 31))| (1<<(( 0*32+ 8) & 31))|(1<<(( 0*32+13)) & 31)|(1<<(( 0*32+24) & 31))|(1<<(( 0*32+15) & 31))| (1<<(( 0*32+25) & 31))|(1<<(( 0*32+26) & 31))))) || (((( 0*32+22))>>5)==1 && (1UL<<((( 0*32+22))&31) & ((1<<(( 1*32+29) & 31))|0))) || (((( 0*32+22))>>5)==2 && (1UL<<((( 0*32+22))&31) & 0)) || (((( 0*32+22))>>5)==3 && (1UL<<((( 0*32+22))&31) & ((1<<(( 3*32+20) & 31))))) || (((( 0*32+22))>>5)==4 && (1UL<<((( 0*32+22))&31) & (0))) || (((( 0*32+22))>>5)==5 && (1UL<<((( 0*32+22))&31) & 0)) || (((( 0*32+22))>>5)==6 && (1UL<<((( 0*32+22))&31) & 0)) || (((( 0*32+22))>>5)==7 && (1UL<<((( 0*32+22))&31) & 0)) || (((( 0*32+22))>>5)==8 && (1UL<<((( 0*32+22))&31) & 0)) || (((( 0*32+22))>>5)==9 && (1UL<<((( 0*32+22))&31) & 0)) ) ? 1 : (__builtin_constant_p((( 0*32+22))) ? constant_test_bit((( 0*32+22)), ((unsigned long *)((c)->x86_capability))) : variable_test_bit((( 0*32+22)), ((unsigned long *)((c)->x86_capability))))))
  buf[2] |= (0x0004);




 if (!(__builtin_constant_p(( 4*32+ 3)) && ( (((( 4*32+ 3))>>5)==0 && (1UL<<((( 4*32+ 3))&31) & ((1<<(( 0*32+ 0) & 31))|(1<<(( 0*32+ 3)) & 31)|(1<<(( 0*32+ 5) & 31))|(1<<(( 0*32+ 6) & 31))| (1<<(( 0*32+ 8) & 31))|(1<<(( 0*32+13)) & 31)|(1<<(( 0*32+24) & 31))|(1<<(( 0*32+15) & 31))| (1<<(( 0*32+25) & 31))|(1<<(( 0*32+26) & 31))))) || (((( 4*32+ 3))>>5)==1 && (1UL<<((( 4*32+ 3))&31) & ((1<<(( 1*32+29) & 31))|0))) || (((( 4*32+ 3))>>5)==2 && (1UL<<((( 4*32+ 3))&31) & 0)) || (((( 4*32+ 3))>>5)==3 && (1UL<<((( 4*32+ 3))&31) & ((1<<(( 3*32+20) & 31))))) || (((( 4*32+ 3))>>5)==4 && (1UL<<((( 4*32+ 3))&31) & (0))) || (((( 4*32+ 3))>>5)==5 && (1UL<<((( 4*32+ 3))&31) & 0)) || (((( 4*32+ 3))>>5)==6 && (1UL<<((( 4*32+ 3))&31) & 0)) || (((( 4*32+ 3))>>5)==7 && (1UL<<((( 4*32+ 3))&31) & 0)) || (((( 4*32+ 3))>>5)==8 && (1UL<<((( 4*32+ 3))&31) & 0)) || (((( 4*32+ 3))>>5)==9 && (1UL<<((( 4*32+ 3))&31) & 0)) ) ? 1 : (__builtin_constant_p((( 4*32+ 3))) ? constant_test_bit((( 4*32+ 3)), ((unsigned long *)((c)->x86_capability))) : variable_test_bit((( 4*32+ 3)), ((unsigned long *)((c)->x86_capability))))))
  buf[2] &= ~((0x0200));
}

static inline __attribute__((no_instrument_function)) bool acpi_has_cpu_in_madt(void)
{
 return !!acpi_lapic;
}
extern int acpi_numa;
extern int x86_acpi_numa_init(void);





typedef u64 cycle_t;
struct clocksource;
struct module;


struct arch_clocksource_data {
 int vclock_mode;
};
struct cyclecounter {
 cycle_t (*read)(const struct cyclecounter *cc);
 cycle_t mask;
 u32 mult;
 u32 shift;
};
struct timecounter {
 const struct cyclecounter *cc;
 cycle_t cycle_last;
 u64 nsec;
};
static inline __attribute__((no_instrument_function)) u64 cyclecounter_cyc2ns(const struct cyclecounter *cc,
          cycle_t cycles)
{
 u64 ret = (u64)cycles;
 ret = (ret * cc->mult) >> cc->shift;
 return ret;
}
extern void timecounter_init(struct timecounter *tc,
        const struct cyclecounter *cc,
        u64 start_tstamp);
extern u64 timecounter_read(struct timecounter *tc);
extern u64 timecounter_cyc2time(struct timecounter *tc,
    cycle_t cycle_tstamp);
struct clocksource {




 cycle_t (*read)(struct clocksource *cs);
 cycle_t mask;
 u32 mult;
 u32 shift;
 u64 max_idle_ns;
 u32 maxadj;

 struct arch_clocksource_data archdata;


 const char *name;
 struct list_head list;
 int rating;
 int (*enable)(struct clocksource *cs);
 void (*disable)(struct clocksource *cs);
 unsigned long flags;
 void (*suspend)(struct clocksource *cs);
 void (*resume)(struct clocksource *cs);




 struct list_head wd_list;
 cycle_t cs_last;
 cycle_t wd_last;

 struct module *owner;
} __attribute__((__aligned__((1 << (6)))));
static inline __attribute__((no_instrument_function)) u32 clocksource_khz2mult(u32 khz, u32 shift_constant)
{







 u64 tmp = ((u64)1000000) << shift_constant;

 tmp += khz/2;
 ({ uint32_t __base = (khz); uint32_t __rem; __rem = ((uint64_t)(tmp)) % __base; (tmp) = ((uint64_t)(tmp)) / __base; __rem; });

 return (u32)tmp;
}
static inline __attribute__((no_instrument_function)) u32 clocksource_hz2mult(u32 hz, u32 shift_constant)
{







 u64 tmp = ((u64)1000000000) << shift_constant;

 tmp += hz/2;
 ({ uint32_t __base = (hz); uint32_t __rem; __rem = ((uint64_t)(tmp)) % __base; (tmp) = ((uint64_t)(tmp)) / __base; __rem; });

 return (u32)tmp;
}
static inline __attribute__((no_instrument_function)) s64 clocksource_cyc2ns(cycle_t cycles, u32 mult, u32 shift)
{
 return ((u64) cycles * mult) >> shift;
}


extern int clocksource_register(struct clocksource*);
extern int clocksource_unregister(struct clocksource*);
extern void clocksource_touch_watchdog(void);
extern struct clocksource* clocksource_get_next(void);
extern void clocksource_change_rating(struct clocksource *cs, int rating);
extern void clocksource_suspend(void);
extern void clocksource_resume(void);
extern struct clocksource * __attribute__ ((__section__(".init.text"))) __attribute__((__cold__)) __attribute__((no_instrument_function)) __attribute__((weak)) clocksource_default_clock(void);
extern void clocksource_mark_unstable(struct clocksource *cs);

extern u64
clocks_calc_max_nsecs(u32 mult, u32 shift, u32 maxadj, u64 mask);
extern void
clocks_calc_mult_shift(u32 *mult, u32 *shift, u32 from, u32 to, u32 minsec);





extern int
__clocksource_register_scale(struct clocksource *cs, u32 scale, u32 freq);
extern void
__clocksource_updatefreq_scale(struct clocksource *cs, u32 scale, u32 freq);

static inline __attribute__((no_instrument_function)) int clocksource_register_hz(struct clocksource *cs, u32 hz)
{
 return __clocksource_register_scale(cs, 1, hz);
}

static inline __attribute__((no_instrument_function)) int clocksource_register_khz(struct clocksource *cs, u32 khz)
{
 return __clocksource_register_scale(cs, 1000, khz);
}

static inline __attribute__((no_instrument_function)) void __clocksource_updatefreq_hz(struct clocksource *cs, u32 hz)
{
 __clocksource_updatefreq_scale(cs, 1, hz);
}

static inline __attribute__((no_instrument_function)) void __clocksource_updatefreq_khz(struct clocksource *cs, u32 khz)
{
 __clocksource_updatefreq_scale(cs, 1000, khz);
}


extern int timekeeping_notify(struct clocksource *clock);

extern cycle_t clocksource_mmio_readl_up(struct clocksource *);
extern cycle_t clocksource_mmio_readl_down(struct clocksource *);
extern cycle_t clocksource_mmio_readw_up(struct clocksource *);
extern cycle_t clocksource_mmio_readw_down(struct clocksource *);

extern int clocksource_mmio_init(void *, const char *,
 unsigned long, int, unsigned, cycle_t (*)(struct clocksource *));

extern int clocksource_i8253_init(void);







static inline __attribute__((no_instrument_function)) void clocksource_of_init(void) {}
struct pvclock_vcpu_time_info {
 u32 version;
 u32 pad0;
 u64 tsc_timestamp;
 u64 system_time;
 u32 tsc_to_system_mul;
 s8 tsc_shift;
 u8 flags;
 u8 pad[2];
} __attribute__((__packed__));

struct pvclock_wall_clock {
 u32 version;
 u32 sec;
 u32 nsec;
} __attribute__((__packed__));


cycle_t pvclock_clocksource_read(struct pvclock_vcpu_time_info *src);
u8 pvclock_read_flags(struct pvclock_vcpu_time_info *src);
void pvclock_set_flags(u8 flags);
unsigned long pvclock_tsc_khz(struct pvclock_vcpu_time_info *src);
void pvclock_read_wallclock(struct pvclock_wall_clock *wall,
       struct pvclock_vcpu_time_info *vcpu,
       struct timespec *ts);
void pvclock_resume(void);

void pvclock_touch_watchdogs(void);





static inline __attribute__((no_instrument_function)) u64 pvclock_scale_delta(u64 delta, u32 mul_frac, int shift)
{
 u64 product;



 ulong tmp;


 if (shift < 0)
  delta >>= -shift;
 else
  delta <<= shift;
 __asm__ (
  "mulq %[mul_frac] ; shrd $32, %[hi], %[lo]"
  : [lo]"=a"(product),
    [hi]"=d"(tmp)
  : "0"(delta),
    [mul_frac]"rm"((u64)mul_frac));




 return product;
}

static inline __attribute__((no_instrument_function)) __attribute__((always_inline))
u64 pvclock_get_nsec_offset(const struct pvclock_vcpu_time_info *src)
{
 u64 delta = __native_read_tsc() - src->tsc_timestamp;
 return pvclock_scale_delta(delta, src->tsc_to_system_mul,
       src->tsc_shift);
}

static inline __attribute__((no_instrument_function)) __attribute__((always_inline))
unsigned __pvclock_read_cycles(const struct pvclock_vcpu_time_info *src,
          cycle_t *cycles, u8 *flags)
{
 unsigned version;
 cycle_t ret, offset;
 u8 ret_flags;

 version = src->version;






 rdtsc_barrier();
 offset = pvclock_get_nsec_offset(src);
 ret = src->system_time + offset;
 ret_flags = src->flags;
 rdtsc_barrier();

 *cycles = ret;
 *flags = ret_flags;
 return version;
}

struct pvclock_vsyscall_time_info {
 struct pvclock_vcpu_time_info pvti;
} __attribute__((__aligned__((1 << (6)))));




int __attribute__ ((__section__(".init.text"))) __attribute__((__cold__)) __attribute__((no_instrument_function)) pvclock_init_vsyscall(struct pvclock_vsyscall_time_info *i,
     int size);
struct pvclock_vcpu_time_info *pvclock_get_vsyscall_time_info(int cpu);







enum vsyscall_num {
 __NR_vgettimeofday,
 __NR_vtime,
 __NR_vgetcpu,
};
enum fixed_addresses {



 VSYSCALL_PAGE = (((((((-10UL << 20) + ((1UL) << 12))-1) | ((__typeof__((-10UL << 20) + ((1UL) << 12)))((1<<21)-1)))+1) - ((1UL) << 12)) - (-10UL << 20)) >> 12,





 FIX_DBGP_BASE,
 FIX_EARLYCON_MEM_BASE,




 FIX_APIC_BASE,


 FIX_IO_APIC_BASE_0,
 FIX_IO_APIC_BASE_END = FIX_IO_APIC_BASE_0 + 128 - 1,

 FIX_RO_IDT,
 FIX_TEXT_POKE1,
 FIX_TEXT_POKE0,



 __end_of_permanent_fixed_addresses,
 FIX_BTMAP_END =
  (__end_of_permanent_fixed_addresses ^
   (__end_of_permanent_fixed_addresses + (64 * 8) - 1)) &
  -512
  ? __end_of_permanent_fixed_addresses + (64 * 8) -
    (__end_of_permanent_fixed_addresses & ((64 * 8) - 1))
  : __end_of_permanent_fixed_addresses,
 FIX_BTMAP_BEGIN = FIX_BTMAP_END + (64 * 8) - 1,




 FIX_TBOOT_BASE,

 __end_of_fixed_addresses
};


extern void reserve_top_address(unsigned long reserve);






extern int fixmaps_set;

extern pte_t *kmap_pte;
extern pgprot_t kmap_prot;
extern pte_t *pkmap_page_table;

void __native_set_fixmap(enum fixed_addresses idx, pte_t pte);
void native_set_fixmap(enum fixed_addresses idx,
         phys_addr_t phys, pgprot_t flags);


static inline __attribute__((no_instrument_function)) void __set_fixmap(enum fixed_addresses idx,
    phys_addr_t phys, pgprot_t flags)
{
 native_set_fixmap(idx, phys, flags);
}


static inline __attribute__((no_instrument_function)) __attribute__((always_inline)) unsigned long fix_to_virt(const unsigned int idx)
{
 ((void)sizeof(char[1 - 2*!!(idx >= __end_of_fixed_addresses)]));
 return (((((((-10UL << 20) + ((1UL) << 12))-1) | ((__typeof__((-10UL << 20) + ((1UL) << 12)))((1<<21)-1)))+1) - ((1UL) << 12)) - ((idx) << 12));
}

static inline __attribute__((no_instrument_function)) unsigned long virt_to_fix(const unsigned long vaddr)
{
 do { if (__builtin_expect(!!(vaddr >= ((((((-10UL << 20) + ((1UL) << 12))-1) | ((__typeof__((-10UL << 20) + ((1UL) << 12)))((1<<21)-1)))+1) - ((1UL) << 12)) || vaddr < (((((((-10UL << 20) + ((1UL) << 12))-1) | ((__typeof__((-10UL << 20) + ((1UL) << 12)))((1<<21)-1)))+1) - ((1UL) << 12)) - (__end_of_permanent_fixed_addresses << 12))), 0)) do { asm volatile("1:\tud2\n" ".pushsection __bug_table,\"a\"\n" "2:\t.long 1b - 2b, %c0 - 2b\n" "\t.word %c1, 0\n" "\t.org 2b+%c2\n" ".popsection" : : "i" ("include/asm-generic/fixmap.h"), "i" (37), "i" (sizeof(struct bug_entry))); __builtin_unreachable(); } while (0); } while (0);
 return ((((((((-10UL << 20) + ((1UL) << 12))-1) | ((__typeof__((-10UL << 20) + ((1UL) << 12)))((1<<21)-1)))+1) - ((1UL) << 12)) - ((vaddr)&(~(((1UL) << 12)-1)))) >> 12);
}




void __early_set_fixmap(enum fixed_addresses idx,
   phys_addr_t phys, pgprot_t flags);








struct notifier_block;
void idle_notifier_register(struct notifier_block *n);
void idle_notifier_unregister(struct notifier_block *n);


void enter_idle(void);
void exit_idle(void);






void amd_e400_remove_cpu(int cpu);
static inline __attribute__((no_instrument_function)) void generic_apic_probe(void)
{
}




extern unsigned int apic_verbosity;
extern int local_apic_timer_c2_ok;

extern int disable_apic;
extern unsigned int lapic_timer_frequency;


extern void __inquire_remote_apic(int apicid);






static inline __attribute__((no_instrument_function)) void default_inquire_remote_apic(int apicid)
{
 if (apic_verbosity >= 2)
  __inquire_remote_apic(apicid);
}
static inline __attribute__((no_instrument_function)) bool apic_from_smp_config(void)
{
 return smp_found_config && !disable_apic;
}
extern int setup_profiling_timer(unsigned int);

static inline __attribute__((no_instrument_function)) void native_apic_mem_write(u32 reg, u32 v)
{
 volatile u32 *addr = (volatile u32 *)((fix_to_virt(FIX_APIC_BASE)) + reg);

 asm volatile ("661:\n\t" "movl %0, %1" "\n662:\n" ".pushsection .altinstructions,\"a\"\n" " .long 661b - .\n" " .long " "663""1""f - .\n" " .word " "(11*32 + (5))" "\n" " .byte " "662b-661b" "\n" " .byte " "664""1""f-""663""1""f" "\n" ".popsection\n" ".pushsection .discard,\"aw\",@progbits\n" " .byte 0xff + (" "664""1""f-""663""1""f" ") - (" "662b-661b" ")\n" ".popsection\n" ".pushsection .altinstr_replacement, \"ax\"\n" "663""1"":\n\t" "xchgl %0, %1" "\n" "664""1" ":\n\t" ".popsection" : "=r" (v), "=m" (*addr) : "i" (0), "0" (v), "m" (*addr))

                                           ;
}

static inline __attribute__((no_instrument_function)) u32 native_apic_mem_read(u32 reg)
{
 return *((volatile u32 *)((fix_to_virt(FIX_APIC_BASE)) + reg));
}

extern void native_apic_wait_icr_idle(void);
extern u32 native_safe_apic_wait_icr_idle(void);
extern void native_apic_icr_write(u32 low, u32 id);
extern u64 native_apic_icr_read(void);

extern int x2apic_mode;







static inline __attribute__((no_instrument_function)) void x2apic_wrmsr_fence(void)
{
 asm volatile("mfence" : : : "memory");
}

static inline __attribute__((no_instrument_function)) void native_apic_msr_write(u32 reg, u32 v)
{
 if (reg == 0xE0 || reg == 0x20 || reg == 0xD0 ||
     reg == 0x30)
  return;

 wrmsr(0x800 + (reg >> 4), v, 0);
}

static inline __attribute__((no_instrument_function)) void native_apic_msr_eoi_write(u32 reg, u32 v)
{
 wrmsr(0x800 + (0xB0 >> 4), 0x0, 0);
}

static inline __attribute__((no_instrument_function)) u32 native_apic_msr_read(u32 reg)
{
 u64 msr;

 if (reg == 0xE0)
  return -1;

 ((msr) = native_read_msr((0x800 + (reg >> 4))));
 return (u32)msr;
}

static inline __attribute__((no_instrument_function)) void native_x2apic_wait_icr_idle(void)
{

 return;
}

static inline __attribute__((no_instrument_function)) u32 native_safe_x2apic_wait_icr_idle(void)
{

 return 0;
}

static inline __attribute__((no_instrument_function)) void native_x2apic_icr_write(u32 low, u32 id)
{
 native_write_msr((0x800 + (0x300 >> 4)), (u32)((u64)(((__u64) id) << 32 | low)), (u32)((u64)(((__u64) id) << 32 | low) >> 32));
}

static inline __attribute__((no_instrument_function)) u64 native_x2apic_icr_read(void)
{
 unsigned long val;

 ((val) = native_read_msr((0x800 + (0x300 >> 4))));
 return val;
}

extern int x2apic_phys;
extern int x2apic_preenabled;
extern void check_x2apic(void);
extern void enable_x2apic(void);
static inline __attribute__((no_instrument_function)) int x2apic_enabled(void)
{
 u64 msr;

 if (!(__builtin_constant_p(( 4*32+21)) && ( (((( 4*32+21))>>5)==0 && (1UL<<((( 4*32+21))&31) & ((1<<(( 0*32+ 0) & 31))|(1<<(( 0*32+ 3)) & 31)|(1<<(( 0*32+ 5) & 31))|(1<<(( 0*32+ 6) & 31))| (1<<(( 0*32+ 8) & 31))|(1<<(( 0*32+13)) & 31)|(1<<(( 0*32+24) & 31))|(1<<(( 0*32+15) & 31))| (1<<(( 0*32+25) & 31))|(1<<(( 0*32+26) & 31))))) || (((( 4*32+21))>>5)==1 && (1UL<<((( 4*32+21))&31) & ((1<<(( 1*32+29) & 31))|0))) || (((( 4*32+21))>>5)==2 && (1UL<<((( 4*32+21))&31) & 0)) || (((( 4*32+21))>>5)==3 && (1UL<<((( 4*32+21))&31) & ((1<<(( 3*32+20) & 31))))) || (((( 4*32+21))>>5)==4 && (1UL<<((( 4*32+21))&31) & (0))) || (((( 4*32+21))>>5)==5 && (1UL<<((( 4*32+21))&31) & 0)) || (((( 4*32+21))>>5)==6 && (1UL<<((( 4*32+21))&31) & 0)) || (((( 4*32+21))>>5)==7 && (1UL<<((( 4*32+21))&31) & 0)) || (((( 4*32+21))>>5)==8 && (1UL<<((( 4*32+21))&31) & 0)) || (((( 4*32+21))>>5)==9 && (1UL<<((( 4*32+21))&31) & 0)) ) ? 1 : (__builtin_constant_p((( 4*32+21))) ? constant_test_bit((( 4*32+21)), ((unsigned long *)((&boot_cpu_data)->x86_capability))) : variable_test_bit((( 4*32+21)), ((unsigned long *)((&boot_cpu_data)->x86_capability))))))
  return 0;

 ((msr) = native_read_msr((0x0000001b)));
 if (msr & (1UL << 10))
  return 1;
 return 0;
}


static inline __attribute__((no_instrument_function)) void x2apic_force_phys(void)
{
 x2apic_phys = 1;
}
extern void enable_IR_x2apic(void);

extern int get_physical_broadcast(void);

extern int lapic_get_maxlvt(void);
extern void clear_local_APIC(void);
extern void connect_bsp_APIC(void);
extern void disconnect_bsp_APIC(int virt_wire_setup);
extern void disable_local_APIC(void);
extern void lapic_shutdown(void);
extern int verify_local_APIC(void);
extern void sync_Arb_IDs(void);
extern void init_bsp_APIC(void);
extern void setup_local_APIC(void);
extern void end_local_APIC_setup(void);
extern void bsp_end_local_APIC_setup(void);
extern void init_apic_mappings(void);
void register_lapic_address(unsigned long address);
extern void setup_boot_APIC_clock(void);
extern void setup_secondary_APIC_clock(void);
extern int APIC_init_uniprocessor(void);
extern int apic_force_enable(unsigned long addr);





extern int apic_is_clustered_box(void);







extern int setup_APIC_eilvt(u8 lvt_off, u8 vector, u8 msg_type, u8 mask);
struct apic {
 char *name;

 int (*probe)(void);
 int (*acpi_madt_oem_check)(char *oem_id, char *oem_table_id);
 int (*apic_id_valid)(int apicid);
 int (*apic_id_registered)(void);

 u32 irq_delivery_mode;
 u32 irq_dest_mode;

 const struct cpumask *(*target_cpus)(void);

 int disable_esr;

 int dest_logical;
 unsigned long (*check_apicid_used)(physid_mask_t *map, int apicid);

 void (*vector_allocation_domain)(int cpu, struct cpumask *retmask,
      const struct cpumask *mask);
 void (*init_apic_ldr)(void);

 void (*ioapic_phys_id_map)(physid_mask_t *phys_map, physid_mask_t *retmap);

 void (*setup_apic_routing)(void);
 int (*cpu_present_to_apicid)(int mps_cpu);
 void (*apicid_to_cpu_present)(int phys_apicid, physid_mask_t *retmap);
 int (*check_phys_apicid_present)(int phys_apicid);
 int (*phys_pkg_id)(int cpuid_apic, int index_msb);

 unsigned int (*get_apic_id)(unsigned long x);
 unsigned long (*set_apic_id)(unsigned int id);
 unsigned long apic_id_mask;

 int (*cpu_mask_to_apicid_and)(const struct cpumask *cpumask,
          const struct cpumask *andmask,
          unsigned int *apicid);


 void (*send_IPI_mask)(const struct cpumask *mask, int vector);
 void (*send_IPI_mask_allbutself)(const struct cpumask *mask,
      int vector);
 void (*send_IPI_allbutself)(int vector);
 void (*send_IPI_all)(int vector);
 void (*send_IPI_self)(int vector);


 int (*wakeup_secondary_cpu)(int apicid, unsigned long start_eip);

 bool wait_for_init_deassert;
 void (*inquire_remote_apic)(int apicid);


 u32 (*read)(u32 reg);
 void (*write)(u32 reg, u32 v);







 void (*eoi_write)(u32 reg, u32 v);
 u64 (*icr_read)(void);
 void (*icr_write)(u32 low, u32 high);
 void (*wait_icr_idle)(void);
 u32 (*safe_wait_icr_idle)(void);
};






extern struct apic *apic;
extern struct apic *__apicdrivers[], *__apicdrivers_end[];





extern atomic_t init_deasserted;
extern int wakeup_secondary_cpu_via_nmi(int apicid, unsigned long start_eip);




static inline __attribute__((no_instrument_function)) u32 apic_read(u32 reg)
{
 return apic->read(reg);
}

static inline __attribute__((no_instrument_function)) void apic_write(u32 reg, u32 val)
{
 apic->write(reg, val);
}

static inline __attribute__((no_instrument_function)) void apic_eoi(void)
{
 apic->eoi_write(0xB0, 0x0);
}

static inline __attribute__((no_instrument_function)) u64 apic_icr_read(void)
{
 return apic->icr_read();
}

static inline __attribute__((no_instrument_function)) void apic_icr_write(u32 low, u32 high)
{
 apic->icr_write(low, high);
}

static inline __attribute__((no_instrument_function)) void apic_wait_icr_idle(void)
{
 apic->wait_icr_idle();
}

static inline __attribute__((no_instrument_function)) u32 safe_apic_wait_icr_idle(void)
{
 return apic->safe_wait_icr_idle();
}

extern void __attribute__ ((__section__(".init.text"))) __attribute__((__cold__)) __attribute__((no_instrument_function)) apic_set_eoi_write(void (*eoi_write)(u32 reg, u32 v));
static inline __attribute__((no_instrument_function)) void ack_APIC_irq(void)
{




 apic_eoi();
}

static inline __attribute__((no_instrument_function)) unsigned default_get_apic_id(unsigned long x)
{
 unsigned int ver = ((apic_read(0x30)) & 0xFFu);

 if (((ver) >= 0x14) || (__builtin_constant_p(( 3*32+26)) && ( (((( 3*32+26))>>5)==0 && (1UL<<((( 3*32+26))&31) & ((1<<(( 0*32+ 0) & 31))|(1<<(( 0*32+ 3)) & 31)|(1<<(( 0*32+ 5) & 31))|(1<<(( 0*32+ 6) & 31))| (1<<(( 0*32+ 8) & 31))|(1<<(( 0*32+13)) & 31)|(1<<(( 0*32+24) & 31))|(1<<(( 0*32+15) & 31))| (1<<(( 0*32+25) & 31))|(1<<(( 0*32+26) & 31))))) || (((( 3*32+26))>>5)==1 && (1UL<<((( 3*32+26))&31) & ((1<<(( 1*32+29) & 31))|0))) || (((( 3*32+26))>>5)==2 && (1UL<<((( 3*32+26))&31) & 0)) || (((( 3*32+26))>>5)==3 && (1UL<<((( 3*32+26))&31) & ((1<<(( 3*32+20) & 31))))) || (((( 3*32+26))>>5)==4 && (1UL<<((( 3*32+26))&31) & (0))) || (((( 3*32+26))>>5)==5 && (1UL<<((( 3*32+26))&31) & 0)) || (((( 3*32+26))>>5)==6 && (1UL<<((( 3*32+26))&31) & 0)) || (((( 3*32+26))>>5)==7 && (1UL<<((( 3*32+26))&31) & 0)) || (((( 3*32+26))>>5)==8 && (1UL<<((( 3*32+26))&31) & 0)) || (((( 3*32+26))>>5)==9 && (1UL<<((( 3*32+26))&31) & 0)) ) ? 1 : (__builtin_constant_p((( 3*32+26))) ? constant_test_bit((( 3*32+26)), ((unsigned long *)((&boot_cpu_data)->x86_capability))) : variable_test_bit((( 3*32+26)), ((unsigned long *)((&boot_cpu_data)->x86_capability))))))
  return (x >> 24) & 0xFF;
 else
  return (x >> 24) & 0x0F;
}
extern void apic_send_IPI_self(int vector);

extern __attribute__((section(".data..percpu" ""))) __typeof__(int) x2apic_extra_bits;

extern int default_cpu_present_to_apicid(int mps_cpu);
extern int default_check_phys_apicid_present(int phys_apicid);


extern void generic_bigsmp_probe(void);







static inline __attribute__((no_instrument_function)) const struct cpumask *default_target_cpus(void)
{

 return cpu_online_mask;



}

static inline __attribute__((no_instrument_function)) const struct cpumask *online_target_cpus(void)
{
 return cpu_online_mask;
}

extern __attribute__((section(".data..percpu" "..read_mostly"))) __typeof__(u16) x86_bios_cpu_apicid; extern __typeof__(u16) *x86_bios_cpu_apicid_early_ptr; extern __typeof__(u16) x86_bios_cpu_apicid_early_map[];


static inline __attribute__((no_instrument_function)) unsigned int read_apic_id(void)
{
 unsigned int reg;

 reg = apic_read(0x20);

 return apic->get_apic_id(reg);
}

static inline __attribute__((no_instrument_function)) int default_apic_id_valid(int apicid)
{
 return (apicid < 255);
}

extern int default_acpi_madt_oem_check(char *, char *);

extern void default_setup_apic_routing(void);

extern struct apic apic_noop;
static inline __attribute__((no_instrument_function)) int
flat_cpu_mask_to_apicid_and(const struct cpumask *cpumask,
       const struct cpumask *andmask,
       unsigned int *apicid)
{
 unsigned long cpu_mask = ((cpumask)->bits)[0] &
     ((andmask)->bits)[0] &
     ((cpu_online_mask)->bits)[0] &
     0xFFu;

 if (__builtin_expect(!!(cpu_mask), 1)) {
  *apicid = (unsigned int)cpu_mask;
  return 0;
 } else {
  return -22;
 }
}

extern int
default_cpu_mask_to_apicid_and(const struct cpumask *cpumask,
          const struct cpumask *andmask,
          unsigned int *apicid);

static inline __attribute__((no_instrument_function)) void
flat_vector_allocation_domain(int cpu, struct cpumask *retmask,
         const struct cpumask *mask)
{
 cpumask_clear(retmask);
 ((retmask)->bits)[0] = 0xFFu;
}

static inline __attribute__((no_instrument_function)) void
default_vector_allocation_domain(int cpu, struct cpumask *retmask,
     const struct cpumask *mask)
{
 cpumask_copy(retmask, (get_cpu_mask(cpu)));
}

static inline __attribute__((no_instrument_function)) unsigned long default_check_apicid_used(physid_mask_t *map, int apicid)
{
 return (__builtin_constant_p((apicid)) ? constant_test_bit((apicid), ((*map).mask)) : variable_test_bit((apicid), ((*map).mask)));
}

static inline __attribute__((no_instrument_function)) void default_ioapic_phys_id_map(physid_mask_t *phys_map, physid_mask_t *retmap)
{
 *retmap = *phys_map;
}

static inline __attribute__((no_instrument_function)) int __default_cpu_present_to_apicid(int mps_cpu)
{
 if (mps_cpu < nr_cpu_ids && (__builtin_constant_p((cpumask_check((mps_cpu)))) ? constant_test_bit((cpumask_check((mps_cpu))), ((((cpu_present_mask))->bits))) : variable_test_bit((cpumask_check((mps_cpu))), ((((cpu_present_mask))->bits)))))
  return (int)(*({ do { const void *__vpp_verify = (typeof((&(x86_bios_cpu_apicid)) + 0))((void *)0); (void)__vpp_verify; } while (0); ({ unsigned long __ptr; __asm__ ("" : "=r"(__ptr) : "0"((typeof(*((&(x86_bios_cpu_apicid)))) *)((&(x86_bios_cpu_apicid))))); (typeof((typeof(*((&(x86_bios_cpu_apicid)))) *)((&(x86_bios_cpu_apicid))))) (__ptr + (((__per_cpu_offset[(mps_cpu)])))); }); }));
 else
  return 0xFFFFu;
}

static inline __attribute__((no_instrument_function)) int
__default_check_phys_apicid_present(int phys_apicid)
{
 return (__builtin_constant_p((phys_apicid)) ? constant_test_bit((phys_apicid), ((phys_cpu_present_map).mask)) : variable_test_bit((phys_apicid), ((phys_cpu_present_map).mask)));
}
extern int default_cpu_present_to_apicid(int mps_cpu);
extern int default_check_phys_apicid_present(int phys_apicid);



extern void irq_enter(void);
extern void irq_exit(void);

static inline __attribute__((no_instrument_function)) void entering_irq(void)
{
 irq_enter();
 exit_idle();
}

static inline __attribute__((no_instrument_function)) void entering_ack_irq(void)
{
 ack_APIC_irq();
 entering_irq();
}

static inline __attribute__((no_instrument_function)) void exiting_irq(void)
{
 irq_exit();
}

static inline __attribute__((no_instrument_function)) void exiting_ack_irq(void)
{
 irq_exit();

 ack_APIC_irq();
}

extern void ioapic_zap_locks(void);







static inline __attribute__((no_instrument_function)) int invalid_vm86_irq(int irq)
{
 return irq < 3 || irq > 15;
}
union IO_APIC_reg_00 {
 u32 raw;
 struct {
  u32 __reserved_2 : 14,
   LTS : 1,
   delivery_type : 1,
   __reserved_1 : 8,
   ID : 8;
 } __attribute__ ((packed)) bits;
};

union IO_APIC_reg_01 {
 u32 raw;
 struct {
  u32 version : 8,
   __reserved_2 : 7,
   PRQ : 1,
   entries : 8,
   __reserved_1 : 8;
 } __attribute__ ((packed)) bits;
};

union IO_APIC_reg_02 {
 u32 raw;
 struct {
  u32 __reserved_2 : 24,
   arbitration : 4,
   __reserved_1 : 4;
 } __attribute__ ((packed)) bits;
};

union IO_APIC_reg_03 {
 u32 raw;
 struct {
  u32 boot_DT : 1,
   __reserved_1 : 31;
 } __attribute__ ((packed)) bits;
};

struct IO_APIC_route_entry {
 __u32 vector : 8,
  delivery_mode : 3,



  dest_mode : 1,
  delivery_status : 1,
  polarity : 1,
  irr : 1,
  trigger : 1,
  mask : 1,
  __reserved_2 : 15;

 __u32 __reserved_3 : 24,
  dest : 8;
} __attribute__ ((packed));

struct IR_IO_APIC_route_entry {
 __u64 vector : 8,
  zero : 3,
  index2 : 1,
  delivery_status : 1,
  polarity : 1,
  irr : 1,
  trigger : 1,
  mask : 1,
  reserved : 31,
  format : 1,
  index : 15;
} __attribute__ ((packed));
extern int nr_ioapics;

extern int mpc_ioapic_id(int ioapic);
extern unsigned int mpc_ioapic_addr(int ioapic);
extern struct mp_ioapic_gsi *mp_ioapic_gsi_routing(int ioapic);




extern int mp_irq_entries;


extern struct mpc_intsrc mp_irqs[(256 * 4)];


extern int sis_apic_bug;


extern int skip_ioapic_setup;


extern int noioapicquirk;


extern int noioapicreroute;
struct io_apic_irq_attr;
struct irq_cfg;
extern void ioapic_insert_resources(void);

extern int native_setup_ioapic_entry(int, struct IO_APIC_route_entry *,
         unsigned int, int,
         struct io_apic_irq_attr *);
extern void eoi_ioapic_irq(unsigned int irq, struct irq_cfg *cfg);

extern void native_compose_msi_msg(struct pci_dev *pdev,
       unsigned int irq, unsigned int dest,
       struct msi_msg *msg, u8 hpet_id);
extern void native_eoi_ioapic_pin(int apic, int pin, int vector);

extern int save_ioapic_entries(void);
extern void mask_ioapic_entries(void);
extern int restore_ioapic_entries(void);

extern void setup_ioapic_ids_from_mpc(void);
extern void setup_ioapic_ids_from_mpc_nocheck(void);

enum ioapic_domain_type {
 IOAPIC_DOMAIN_INVALID,
 IOAPIC_DOMAIN_LEGACY,
 IOAPIC_DOMAIN_STRICT,
 IOAPIC_DOMAIN_DYNAMIC,
};

struct device_node;
struct irq_domain;
struct irq_domain_ops;

struct ioapic_domain_cfg {
 enum ioapic_domain_type type;
 const struct irq_domain_ops *ops;
 struct device_node *dev;
};

struct mp_ioapic_gsi{
 u32 gsi_base;
 u32 gsi_end;
};
extern u32 gsi_top;

extern int mp_find_ioapic(u32 gsi);
extern int mp_find_ioapic_pin(int ioapic, u32 gsi);
extern u32 mp_pin_to_gsi(int ioapic, int pin);
extern int mp_map_gsi_to_irq(u32 gsi, unsigned int flags);
extern void mp_unmap_irq(int irq);
extern void __attribute__ ((__section__(".init.text"))) __attribute__((__cold__)) __attribute__((no_instrument_function)) mp_register_ioapic(int id, u32 address, u32 gsi_base,
          struct ioapic_domain_cfg *cfg);
extern int mp_irqdomain_map(struct irq_domain *domain, unsigned int virq,
       irq_hw_number_t hwirq);
extern void mp_irqdomain_unmap(struct irq_domain *domain, unsigned int virq);
extern int mp_set_gsi_attr(u32 gsi, int trigger, int polarity, int node);
extern void __attribute__ ((__section__(".init.text"))) __attribute__((__cold__)) __attribute__((no_instrument_function)) pre_init_apic_IRQ0(void);

extern void mp_save_irq(struct mpc_intsrc *m);

extern void disable_ioapic_support(void);

extern void __attribute__ ((__section__(".init.text"))) __attribute__((__cold__)) __attribute__((no_instrument_function)) native_io_apic_init_mappings(void);
extern unsigned int native_io_apic_read(unsigned int apic, unsigned int reg);
extern void native_io_apic_write(unsigned int apic, unsigned int reg, unsigned int val);
extern void native_io_apic_modify(unsigned int apic, unsigned int reg, unsigned int val);
extern void native_disable_io_apic(void);
extern void native_io_apic_print_entries(unsigned int apic, unsigned int nr_entries);
extern void intel_ir_io_apic_print_entries(unsigned int apic, unsigned int nr_entries);
extern int native_ioapic_set_affinity(struct irq_data *,
          const struct cpumask *,
          bool);

static inline __attribute__((no_instrument_function)) unsigned int io_apic_read(unsigned int apic, unsigned int reg)
{
 return x86_io_apic_ops.read(apic, reg);
}

static inline __attribute__((no_instrument_function)) void io_apic_write(unsigned int apic, unsigned int reg, unsigned int value)
{
 x86_io_apic_ops.write(apic, reg, value);
}
static inline __attribute__((no_instrument_function)) void io_apic_modify(unsigned int apic, unsigned int reg, unsigned int value)
{
 x86_io_apic_ops.modify(apic, reg, value);
}

extern void io_apic_eoi(unsigned int apic, unsigned int vector);

extern bool mp_should_keep_irq(struct device *dev);






extern int smp_num_siblings;
extern unsigned int num_processors;

static inline __attribute__((no_instrument_function)) bool cpu_has_ht_siblings(void)
{
 bool has_siblings = false;

 has_siblings = (__builtin_constant_p(( 0*32+28)) && ( (((( 0*32+28))>>5)==0 && (1UL<<((( 0*32+28))&31) & ((1<<(( 0*32+ 0) & 31))|(1<<(( 0*32+ 3)) & 31)|(1<<(( 0*32+ 5) & 31))|(1<<(( 0*32+ 6) & 31))| (1<<(( 0*32+ 8) & 31))|(1<<(( 0*32+13)) & 31)|(1<<(( 0*32+24) & 31))|(1<<(( 0*32+15) & 31))| (1<<(( 0*32+25) & 31))|(1<<(( 0*32+26) & 31))))) || (((( 0*32+28))>>5)==1 && (1UL<<((( 0*32+28))&31) & ((1<<(( 1*32+29) & 31))|0))) || (((( 0*32+28))>>5)==2 && (1UL<<((( 0*32+28))&31) & 0)) || (((( 0*32+28))>>5)==3 && (1UL<<((( 0*32+28))&31) & ((1<<(( 3*32+20) & 31))))) || (((( 0*32+28))>>5)==4 && (1UL<<((( 0*32+28))&31) & (0))) || (((( 0*32+28))>>5)==5 && (1UL<<((( 0*32+28))&31) & 0)) || (((( 0*32+28))>>5)==6 && (1UL<<((( 0*32+28))&31) & 0)) || (((( 0*32+28))>>5)==7 && (1UL<<((( 0*32+28))&31) & 0)) || (((( 0*32+28))>>5)==8 && (1UL<<((( 0*32+28))&31) & 0)) || (((( 0*32+28))>>5)==9 && (1UL<<((( 0*32+28))&31) & 0)) ) ? 1 : (__builtin_constant_p((( 0*32+28))) ? constant_test_bit((( 0*32+28)), ((unsigned long *)((&boot_cpu_data)->x86_capability))) : variable_test_bit((( 0*32+28)), ((unsigned long *)((&boot_cpu_data)->x86_capability))))) && smp_num_siblings > 1;

 return has_siblings;
}

extern __attribute__((section(".data..percpu" "..read_mostly"))) __typeof__(cpumask_var_t) cpu_sibling_map;
extern __attribute__((section(".data..percpu" "..read_mostly"))) __typeof__(cpumask_var_t) cpu_core_map;

extern __attribute__((section(".data..percpu" "..read_mostly"))) __typeof__(cpumask_var_t) cpu_llc_shared_map;
extern __attribute__((section(".data..percpu" "..read_mostly"))) __typeof__(u16) cpu_llc_id;
extern __attribute__((section(".data..percpu" "..read_mostly"))) __typeof__(int) cpu_number;

static inline __attribute__((no_instrument_function)) struct cpumask *cpu_sibling_mask(int cpu)
{
 return (*({ do { const void *__vpp_verify = (typeof((&(cpu_sibling_map)) + 0))((void *)0); (void)__vpp_verify; } while (0); ({ unsigned long __ptr; __asm__ ("" : "=r"(__ptr) : "0"((typeof(*((&(cpu_sibling_map)))) *)((&(cpu_sibling_map))))); (typeof((typeof(*((&(cpu_sibling_map)))) *)((&(cpu_sibling_map))))) (__ptr + (((__per_cpu_offset[(cpu)])))); }); }));
}

static inline __attribute__((no_instrument_function)) struct cpumask *cpu_core_mask(int cpu)
{
 return (*({ do { const void *__vpp_verify = (typeof((&(cpu_core_map)) + 0))((void *)0); (void)__vpp_verify; } while (0); ({ unsigned long __ptr; __asm__ ("" : "=r"(__ptr) : "0"((typeof(*((&(cpu_core_map)))) *)((&(cpu_core_map))))); (typeof((typeof(*((&(cpu_core_map)))) *)((&(cpu_core_map))))) (__ptr + (((__per_cpu_offset[(cpu)])))); }); }));
}

static inline __attribute__((no_instrument_function)) struct cpumask *cpu_llc_shared_mask(int cpu)
{
 return (*({ do { const void *__vpp_verify = (typeof((&(cpu_llc_shared_map)) + 0))((void *)0); (void)__vpp_verify; } while (0); ({ unsigned long __ptr; __asm__ ("" : "=r"(__ptr) : "0"((typeof(*((&(cpu_llc_shared_map)))) *)((&(cpu_llc_shared_map))))); (typeof((typeof(*((&(cpu_llc_shared_map)))) *)((&(cpu_llc_shared_map))))) (__ptr + (((__per_cpu_offset[(cpu)])))); }); }));
}

extern __attribute__((section(".data..percpu" "..read_mostly"))) __typeof__(u16) x86_cpu_to_apicid; extern __typeof__(u16) *x86_cpu_to_apicid_early_ptr; extern __typeof__(u16) x86_cpu_to_apicid_early_map[];
extern __attribute__((section(".data..percpu" "..read_mostly"))) __typeof__(u16) x86_bios_cpu_apicid; extern __typeof__(u16) *x86_bios_cpu_apicid_early_ptr; extern __typeof__(u16) x86_bios_cpu_apicid_early_map[];





extern unsigned long stack_start;

struct task_struct;

struct smp_ops {
 void (*smp_prepare_boot_cpu)(void);
 void (*smp_prepare_cpus)(unsigned max_cpus);
 void (*smp_cpus_done)(unsigned max_cpus);

 void (*stop_other_cpus)(int wait);
 void (*smp_send_reschedule)(int cpu);

 int (*cpu_up)(unsigned cpu, struct task_struct *tidle);
 int (*cpu_disable)(void);
 void (*cpu_die)(unsigned int cpu);
 void (*play_dead)(void);

 void (*send_call_func_ipi)(const struct cpumask *mask);
 void (*send_call_func_single_ipi)(int cpu);
};


extern void set_cpu_sibling_map(int cpu);





extern struct smp_ops smp_ops;

static inline __attribute__((no_instrument_function)) void smp_send_stop(void)
{
 smp_ops.stop_other_cpus(0);
}

static inline __attribute__((no_instrument_function)) void stop_other_cpus(void)
{
 smp_ops.stop_other_cpus(1);
}

static inline __attribute__((no_instrument_function)) void smp_prepare_boot_cpu(void)
{
 smp_ops.smp_prepare_boot_cpu();
}

static inline __attribute__((no_instrument_function)) void smp_prepare_cpus(unsigned int max_cpus)
{
 smp_ops.smp_prepare_cpus(max_cpus);
}

static inline __attribute__((no_instrument_function)) void smp_cpus_done(unsigned int max_cpus)
{
 smp_ops.smp_cpus_done(max_cpus);
}

static inline __attribute__((no_instrument_function)) int __cpu_up(unsigned int cpu, struct task_struct *tidle)
{
 return smp_ops.cpu_up(cpu, tidle);
}

static inline __attribute__((no_instrument_function)) int __cpu_disable(void)
{
 return smp_ops.cpu_disable();
}

static inline __attribute__((no_instrument_function)) void __cpu_die(unsigned int cpu)
{
 smp_ops.cpu_die(cpu);
}

static inline __attribute__((no_instrument_function)) void play_dead(void)
{
 smp_ops.play_dead();
}

static inline __attribute__((no_instrument_function)) void smp_send_reschedule(int cpu)
{
 smp_ops.smp_send_reschedule(cpu);
}

static inline __attribute__((no_instrument_function)) void arch_send_call_function_single_ipi(int cpu)
{
 smp_ops.send_call_func_single_ipi(cpu);
}

static inline __attribute__((no_instrument_function)) void arch_send_call_function_ipi_mask(const struct cpumask *mask)
{
 smp_ops.send_call_func_ipi(mask);
}

void cpu_disable_common(void);
void native_smp_prepare_boot_cpu(void);
void native_smp_prepare_cpus(unsigned int max_cpus);
void native_smp_cpus_done(unsigned int max_cpus);
int native_cpu_up(unsigned int cpunum, struct task_struct *tidle);
int native_cpu_disable(void);
void native_cpu_die(unsigned int cpu);
void native_play_dead(void);
void play_dead_common(void);
void wbinvd_on_cpu(int cpu);
int wbinvd_on_all_cpus(void);

void native_send_call_func_ipi(const struct cpumask *mask);
void native_send_call_func_single_ipi(int cpu);
void x86_idle_thread_init(unsigned int cpu, struct task_struct *idle);

void smp_store_boot_cpu_info(void);
void smp_store_cpu_info(int id);
extern unsigned disabled_cpus;
extern int hard_smp_processor_id(void);

extern struct pglist_data *node_data[];



extern struct pglist_data *first_online_pgdat(void);
extern struct pglist_data *next_online_pgdat(struct pglist_data *pgdat);
extern struct zone *next_zone(struct zone *zone);
static inline __attribute__((no_instrument_function)) struct zone *zonelist_zone(struct zoneref *zoneref)
{
 return zoneref->zone;
}

static inline __attribute__((no_instrument_function)) int zonelist_zone_idx(struct zoneref *zoneref)
{
 return zoneref->zone_idx;
}

static inline __attribute__((no_instrument_function)) int zonelist_node_idx(struct zoneref *zoneref)
{


 return zoneref->zone->node;



}
struct zoneref *next_zones_zonelist(struct zoneref *z,
     enum zone_type highest_zoneidx,
     nodemask_t *nodes,
     struct zone **zone);
static inline __attribute__((no_instrument_function)) struct zoneref *first_zones_zonelist(struct zonelist *zonelist,
     enum zone_type highest_zoneidx,
     nodemask_t *nodes,
     struct zone **zone)
{
 return next_zones_zonelist(zonelist->_zonerefs, highest_zoneidx, nodes,
        zone);
}
struct page;
struct page_cgroup;
struct mem_section {
 unsigned long section_mem_map;


 unsigned long *pageblock_flags;
};
extern struct mem_section *mem_section[((((1UL << (46 - 27))) + ((((1UL) << 12) / sizeof (struct mem_section))) - 1) / ((((1UL) << 12) / sizeof (struct mem_section))))];




static inline __attribute__((no_instrument_function)) struct mem_section *__nr_to_section(unsigned long nr)
{
 if (!mem_section[((nr) / (((1UL) << 12) / sizeof (struct mem_section)))])
  return ((void *)0);
 return &mem_section[((nr) / (((1UL) << 12) / sizeof (struct mem_section)))][nr & ((((1UL) << 12) / sizeof (struct mem_section)) - 1)];
}
extern int __section_nr(struct mem_section* ms);
extern unsigned long usemap_size(void);
static inline __attribute__((no_instrument_function)) struct page *__section_mem_map_addr(struct mem_section *section)
{
 unsigned long map = section->section_mem_map;
 map &= (~((1UL<<2)-1));
 return (struct page *)map;
}

static inline __attribute__((no_instrument_function)) int present_section(struct mem_section *section)
{
 return (section && (section->section_mem_map & (1UL<<0)));
}

static inline __attribute__((no_instrument_function)) int present_section_nr(unsigned long nr)
{
 return present_section(__nr_to_section(nr));
}

static inline __attribute__((no_instrument_function)) int valid_section(struct mem_section *section)
{
 return (section && (section->section_mem_map & (1UL<<1)));
}

static inline __attribute__((no_instrument_function)) int valid_section_nr(unsigned long nr)
{
 return valid_section(__nr_to_section(nr));
}

static inline __attribute__((no_instrument_function)) struct mem_section *__pfn_to_section(unsigned long pfn)
{
 return __nr_to_section(((pfn) >> (27 - 12)));
}


static inline __attribute__((no_instrument_function)) int pfn_valid(unsigned long pfn)
{
 if (((pfn) >> (27 - 12)) >= (1UL << (46 - 27)))
  return 0;
 return valid_section(__nr_to_section(((pfn) >> (27 - 12))));
}


static inline __attribute__((no_instrument_function)) int pfn_present(unsigned long pfn)
{
 if (((pfn) >> (27 - 12)) >= (1UL << (46 - 27)))
  return 0;
 return present_section(__nr_to_section(((pfn) >> (27 - 12))));
}
void sparse_init(void);






bool early_pfn_in_nid(unsigned long pfn, int nid);
void memory_present(int nid, unsigned long start, unsigned long end);
unsigned long __attribute__ ((__section__(".init.text"))) __attribute__((__cold__)) __attribute__((no_instrument_function)) node_memmap_size_bytes(int, unsigned long, unsigned long);
static inline __attribute__((no_instrument_function)) int memmap_valid_within(unsigned long pfn,
     struct page *page, struct zone *zone)
{
 return 1;
}


struct llist_head {
 struct llist_node *first;
};

struct llist_node {
 struct llist_node *next;
};
static inline __attribute__((no_instrument_function)) void init_llist_head(struct llist_head *list)
{
 list->first = ((void *)0);
}
static inline __attribute__((no_instrument_function)) bool llist_empty(const struct llist_head *head)
{
 return (*(volatile typeof(head->first) *)&(head->first)) == ((void *)0);
}

static inline __attribute__((no_instrument_function)) struct llist_node *llist_next(struct llist_node *node)
{
 return node->next;
}

extern bool llist_add_batch(struct llist_node *new_first,
       struct llist_node *new_last,
       struct llist_head *head);







static inline __attribute__((no_instrument_function)) bool llist_add(struct llist_node *new, struct llist_head *head)
{
 return llist_add_batch(new, new, head);
}
static inline __attribute__((no_instrument_function)) struct llist_node *llist_del_all(struct llist_head *head)
{
 return ({ __typeof__ (*((&head->first))) __ret = ((((void *)0))); switch (sizeof(*((&head->first)))) { case 1: asm volatile ("" "xchg" "b %b0, %1\n" : "+q" (__ret), "+m" (*((&head->first))) : : "memory", "cc"); break; case 2: asm volatile ("" "xchg" "w %w0, %1\n" : "+r" (__ret), "+m" (*((&head->first))) : : "memory", "cc"); break; case 4: asm volatile ("" "xchg" "l %0, %1\n" : "+r" (__ret), "+m" (*((&head->first))) : : "memory", "cc"); break; case 8: asm volatile ("" "xchg" "q %q0, %1\n" : "+r" (__ret), "+m" (*((&head->first))) : : "memory", "cc"); break; default: __xchg_wrong_size(); } __ret; });
}

extern struct llist_node *llist_del_first(struct llist_head *head);

struct llist_node *llist_reverse_order(struct llist_node *head);

typedef void (*smp_call_func_t)(void *info);
struct call_single_data {
 struct llist_node llist;
 smp_call_func_t func;
 void *info;
 u16 flags;
};


extern unsigned int total_cpus;

int smp_call_function_single(int cpuid, smp_call_func_t func, void *info,
        int wait);




int on_each_cpu(smp_call_func_t func, void *info, int wait);





void on_each_cpu_mask(const struct cpumask *mask, smp_call_func_t func,
  void *info, bool wait);






void on_each_cpu_cond(bool (*cond_func)(int cpu, void *info),
  smp_call_func_t func, void *info, bool wait,
  gfp_t gfp_flags);

int smp_call_function_single_async(int cpu, struct call_single_data *csd);
extern void smp_send_stop(void);




extern void smp_send_reschedule(int cpu);





extern void smp_prepare_cpus(unsigned int max_cpus);




extern int __cpu_up(unsigned int cpunum, struct task_struct *tidle);




extern void smp_cpus_done(unsigned int max_cpus);




int smp_call_function(smp_call_func_t func, void *info, int wait);
void smp_call_function_many(const struct cpumask *mask,
       smp_call_func_t func, void *info, bool wait);

int smp_call_function_any(const struct cpumask *mask,
     smp_call_func_t func, void *info, int wait);

void kick_all_cpus_sync(void);




void __attribute__ ((__section__(".init.text"))) __attribute__((__cold__)) __attribute__((no_instrument_function)) call_function_init(void);
void generic_smp_call_function_single_interrupt(void);







void smp_prepare_boot_cpu(void);

extern unsigned int setup_max_cpus;
extern void __attribute__ ((__section__(".init.text"))) __attribute__((__cold__)) __attribute__((no_instrument_function)) setup_nr_cpu_ids(void);
extern void __attribute__ ((__section__(".init.text"))) __attribute__((__cold__)) __attribute__((no_instrument_function)) smp_init(void);
extern void arch_disable_smp_support(void);

extern void arch_enable_nonboot_cpus_begin(void);
extern void arch_enable_nonboot_cpus_end(void);

void smp_setup_processor_id(void);







extern void *pcpu_base_addr;
extern const unsigned long *pcpu_unit_offsets;

struct pcpu_group_info {
 int nr_units;
 unsigned long base_offset;
 unsigned int *cpu_map;

};

struct pcpu_alloc_info {
 size_t static_size;
 size_t reserved_size;
 size_t dyn_size;
 size_t unit_size;
 size_t atom_size;
 size_t alloc_size;
 size_t __ai_size;
 int nr_groups;
 struct pcpu_group_info groups[];
};

enum pcpu_fc {
 PCPU_FC_AUTO,
 PCPU_FC_EMBED,
 PCPU_FC_PAGE,

 PCPU_FC_NR,
};
extern const char * const pcpu_fc_names[PCPU_FC_NR];

extern enum pcpu_fc pcpu_chosen_fc;

typedef void * (*pcpu_fc_alloc_fn_t)(unsigned int cpu, size_t size,
         size_t align);
typedef void (*pcpu_fc_free_fn_t)(void *ptr, size_t size);
typedef void (*pcpu_fc_populate_pte_fn_t)(unsigned long addr);
typedef int (pcpu_fc_cpu_distance_fn_t)(unsigned int from, unsigned int to);

extern struct pcpu_alloc_info * __attribute__ ((__section__(".init.text"))) __attribute__((__cold__)) __attribute__((no_instrument_function)) pcpu_alloc_alloc_info(int nr_groups,
            int nr_units);
extern void __attribute__ ((__section__(".init.text"))) __attribute__((__cold__)) __attribute__((no_instrument_function)) pcpu_free_alloc_info(struct pcpu_alloc_info *ai);

extern int __attribute__ ((__section__(".init.text"))) __attribute__((__cold__)) __attribute__((no_instrument_function)) pcpu_setup_first_chunk(const struct pcpu_alloc_info *ai,
      void *base_addr);


extern int __attribute__ ((__section__(".init.text"))) __attribute__((__cold__)) __attribute__((no_instrument_function)) pcpu_embed_first_chunk(size_t reserved_size, size_t dyn_size,
    size_t atom_size,
    pcpu_fc_cpu_distance_fn_t cpu_distance_fn,
    pcpu_fc_alloc_fn_t alloc_fn,
    pcpu_fc_free_fn_t free_fn);



extern int __attribute__ ((__section__(".init.text"))) __attribute__((__cold__)) __attribute__((no_instrument_function)) pcpu_page_first_chunk(size_t reserved_size,
    pcpu_fc_alloc_fn_t alloc_fn,
    pcpu_fc_free_fn_t free_fn,
    pcpu_fc_populate_pte_fn_t populate_pte_fn);


extern void *__alloc_reserved_percpu(size_t size, size_t align);
extern bool is_kernel_percpu_address(unsigned long addr);




extern void __attribute__ ((__section__(".init.text"))) __attribute__((__cold__)) __attribute__((no_instrument_function)) percpu_init_late(void);

extern void *__alloc_percpu(size_t size, size_t align);
extern void free_percpu(void *__pdata);
extern phys_addr_t per_cpu_ptr_to_phys(void *addr);
int arch_update_cpu_topology(void);
extern __attribute__((section(".data..percpu" ""))) __typeof__(int) numa_node;



static inline __attribute__((no_instrument_function)) int numa_node_id(void)
{
 return ({ typeof(numa_node) pscr_ret__; do { const void *__vpp_verify = (typeof((&(numa_node)) + 0))((void *)0); (void)__vpp_verify; } while (0); switch(sizeof(numa_node)) { case 1: pscr_ret__ = ({ typeof((numa_node)) pfo_ret__; switch (sizeof((numa_node))) { case 1: asm("mov" "b ""%%""gs"":" "%P" "1"",%0" : "=q" (pfo_ret__) : "m"(numa_node)); break; case 2: asm("mov" "w ""%%""gs"":" "%P" "1"",%0" : "=r" (pfo_ret__) : "m"(numa_node)); break; case 4: asm("mov" "l ""%%""gs"":" "%P" "1"",%0" : "=r" (pfo_ret__) : "m"(numa_node)); break; case 8: asm("mov" "q ""%%""gs"":" "%P" "1"",%0" : "=r" (pfo_ret__) : "m"(numa_node)); break; default: __bad_percpu_size(); } pfo_ret__; }); break; case 2: pscr_ret__ = ({ typeof((numa_node)) pfo_ret__; switch (sizeof((numa_node))) { case 1: asm("mov" "b ""%%""gs"":" "%P" "1"",%0" : "=q" (pfo_ret__) : "m"(numa_node)); break; case 2: asm("mov" "w ""%%""gs"":" "%P" "1"",%0" : "=r" (pfo_ret__) : "m"(numa_node)); break; case 4: asm("mov" "l ""%%""gs"":" "%P" "1"",%0" : "=r" (pfo_ret__) : "m"(numa_node)); break; case 8: asm("mov" "q ""%%""gs"":" "%P" "1"",%0" : "=r" (pfo_ret__) : "m"(numa_node)); break; default: __bad_percpu_size(); } pfo_ret__; }); break; case 4: pscr_ret__ = ({ typeof((numa_node)) pfo_ret__; switch (sizeof((numa_node))) { case 1: asm("mov" "b ""%%""gs"":" "%P" "1"",%0" : "=q" (pfo_ret__) : "m"(numa_node)); break; case 2: asm("mov" "w ""%%""gs"":" "%P" "1"",%0" : "=r" (pfo_ret__) : "m"(numa_node)); break; case 4: asm("mov" "l ""%%""gs"":" "%P" "1"",%0" : "=r" (pfo_ret__) : "m"(numa_node)); break; case 8: asm("mov" "q ""%%""gs"":" "%P" "1"",%0" : "=r" (pfo_ret__) : "m"(numa_node)); break; default: __bad_percpu_size(); } pfo_ret__; }); break; case 8: pscr_ret__ = ({ typeof((numa_node)) pfo_ret__; switch (sizeof((numa_node))) { case 1: asm("mov" "b ""%%""gs"":" "%P" "1"",%0" : "=q" (pfo_ret__) : "m"(numa_node)); break; case 2: asm("mov" "w ""%%""gs"":" "%P" "1"",%0" : "=r" (pfo_ret__) : "m"(numa_node)); break; case 4: asm("mov" "l ""%%""gs"":" "%P" "1"",%0" : "=r" (pfo_ret__) : "m"(numa_node)); break; case 8: asm("mov" "q ""%%""gs"":" "%P" "1"",%0" : "=r" (pfo_ret__) : "m"(numa_node)); break; default: __bad_percpu_size(); } pfo_ret__; }); break; default: __bad_size_call_parameter(); break; } pscr_ret__; });
}



static inline __attribute__((no_instrument_function)) int cpu_to_node(int cpu)
{
 return (*({ do { const void *__vpp_verify = (typeof((&(numa_node)) + 0))((void *)0); (void)__vpp_verify; } while (0); ({ unsigned long __ptr; __asm__ ("" : "=r"(__ptr) : "0"((typeof(*((&(numa_node)))) *)((&(numa_node))))); (typeof((typeof(*((&(numa_node)))) *)((&(numa_node))))) (__ptr + (((__per_cpu_offset[(cpu)])))); }); }));
}



static inline __attribute__((no_instrument_function)) void set_numa_node(int node)
{
 do { do { const void *__vpp_verify = (typeof((&(numa_node)) + 0))((void *)0); (void)__vpp_verify; } while (0); switch(sizeof(numa_node)) { case 1: do { typedef typeof((numa_node)) pto_T__; if (0) { pto_T__ pto_tmp__; pto_tmp__ = (node); (void)pto_tmp__; } switch (sizeof((numa_node))) { case 1: asm("mov" "b %1,""%%""gs"":" "%P" "0" : "+m" ((numa_node)) : "qi" ((pto_T__)(node))); break; case 2: asm("mov" "w %1,""%%""gs"":" "%P" "0" : "+m" ((numa_node)) : "ri" ((pto_T__)(node))); break; case 4: asm("mov" "l %1,""%%""gs"":" "%P" "0" : "+m" ((numa_node)) : "ri" ((pto_T__)(node))); break; case 8: asm("mov" "q %1,""%%""gs"":" "%P" "0" : "+m" ((numa_node)) : "re" ((pto_T__)(node))); break; default: __bad_percpu_size(); } } while (0);break; case 2: do { typedef typeof((numa_node)) pto_T__; if (0) { pto_T__ pto_tmp__; pto_tmp__ = (node); (void)pto_tmp__; } switch (sizeof((numa_node))) { case 1: asm("mov" "b %1,""%%""gs"":" "%P" "0" : "+m" ((numa_node)) : "qi" ((pto_T__)(node))); break; case 2: asm("mov" "w %1,""%%""gs"":" "%P" "0" : "+m" ((numa_node)) : "ri" ((pto_T__)(node))); break; case 4: asm("mov" "l %1,""%%""gs"":" "%P" "0" : "+m" ((numa_node)) : "ri" ((pto_T__)(node))); break; case 8: asm("mov" "q %1,""%%""gs"":" "%P" "0" : "+m" ((numa_node)) : "re" ((pto_T__)(node))); break; default: __bad_percpu_size(); } } while (0);break; case 4: do { typedef typeof((numa_node)) pto_T__; if (0) { pto_T__ pto_tmp__; pto_tmp__ = (node); (void)pto_tmp__; } switch (sizeof((numa_node))) { case 1: asm("mov" "b %1,""%%""gs"":" "%P" "0" : "+m" ((numa_node)) : "qi" ((pto_T__)(node))); break; case 2: asm("mov" "w %1,""%%""gs"":" "%P" "0" : "+m" ((numa_node)) : "ri" ((pto_T__)(node))); break; case 4: asm("mov" "l %1,""%%""gs"":" "%P" "0" : "+m" ((numa_node)) : "ri" ((pto_T__)(node))); break; case 8: asm("mov" "q %1,""%%""gs"":" "%P" "0" : "+m" ((numa_node)) : "re" ((pto_T__)(node))); break; default: __bad_percpu_size(); } } while (0);break; case 8: do { typedef typeof((numa_node)) pto_T__; if (0) { pto_T__ pto_tmp__; pto_tmp__ = (node); (void)pto_tmp__; } switch (sizeof((numa_node))) { case 1: asm("mov" "b %1,""%%""gs"":" "%P" "0" : "+m" ((numa_node)) : "qi" ((pto_T__)(node))); break; case 2: asm("mov" "w %1,""%%""gs"":" "%P" "0" : "+m" ((numa_node)) : "ri" ((pto_T__)(node))); break; case 4: asm("mov" "l %1,""%%""gs"":" "%P" "0" : "+m" ((numa_node)) : "ri" ((pto_T__)(node))); break; case 8: asm("mov" "q %1,""%%""gs"":" "%P" "0" : "+m" ((numa_node)) : "re" ((pto_T__)(node))); break; default: __bad_percpu_size(); } } while (0);break; default: __bad_size_call_parameter();break; } } while (0);
}



static inline __attribute__((no_instrument_function)) void set_cpu_numa_node(int cpu, int node)
{
 (*({ do { const void *__vpp_verify = (typeof((&(numa_node)) + 0))((void *)0); (void)__vpp_verify; } while (0); ({ unsigned long __ptr; __asm__ ("" : "=r"(__ptr) : "0"((typeof(*((&(numa_node)))) *)((&(numa_node))))); (typeof((typeof(*((&(numa_node)))) *)((&(numa_node))))) (__ptr + (((__per_cpu_offset[(cpu)])))); }); })) = node;
}
static inline __attribute__((no_instrument_function)) int numa_mem_id(void)
{
 return numa_node_id();
}



static inline __attribute__((no_instrument_function)) int cpu_to_mem(int cpu)
{
 return cpu_to_node(cpu);
}
static inline __attribute__((no_instrument_function)) const struct cpumask *cpu_smt_mask(int cpu)
{
 return ((*({ do { const void *__vpp_verify = (typeof((&(cpu_sibling_map)) + 0))((void *)0); (void)__vpp_verify; } while (0); ({ unsigned long __ptr; __asm__ ("" : "=r"(__ptr) : "0"((typeof(*((&(cpu_sibling_map)))) *)((&(cpu_sibling_map))))); (typeof((typeof(*((&(cpu_sibling_map)))) *)((&(cpu_sibling_map))))) (__ptr + (((__per_cpu_offset[(cpu)])))); }); })));
}


static inline __attribute__((no_instrument_function)) const struct cpumask *cpu_cpu_mask(int cpu)
{
 return cpumask_of_node(cpu_to_node(cpu));
}

struct vm_area_struct;
static inline __attribute__((no_instrument_function)) int allocflags_to_migratetype(gfp_t gfp_flags)
{
 ({ int __ret_warn_on = !!((gfp_flags & ((( gfp_t)0x80000u)|(( gfp_t)0x08u))) == ((( gfp_t)0x80000u)|(( gfp_t)0x08u))); if (__builtin_expect(!!(__ret_warn_on), 0)) warn_slowpath_null("include/linux/gfp.h", 161); __builtin_expect(!!(__ret_warn_on), 0); });

 if (__builtin_expect(!!(page_group_by_mobility_disabled), 0))
  return MIGRATE_UNMOVABLE;


 return (((gfp_flags & (( gfp_t)0x08u)) != 0) << 1) |
  ((gfp_flags & (( gfp_t)0x80000u)) != 0);
}
static inline __attribute__((no_instrument_function)) enum zone_type gfp_zone(gfp_t flags)
{
 enum zone_type z;
 int bit = ( int) (flags & ((( gfp_t)0x01u)|(( gfp_t)0x02u)|(( gfp_t)0x04u)|(( gfp_t)0x08u)));

 z = (( (ZONE_NORMAL << 0 * 2) | (ZONE_DMA << 0x01u * 2) | (ZONE_NORMAL << 0x02u * 2) | (ZONE_DMA32 << 0x04u * 2) | (ZONE_NORMAL << 0x08u * 2) | (ZONE_DMA << (0x08u | 0x01u) * 2) | (ZONE_MOVABLE << (0x08u | 0x02u) * 2) | (ZONE_DMA32 << (0x08u | 0x04u) * 2) ) >> (bit * 2)) &
      ((1 << 2) - 1);
 ((void)(sizeof(( long)((( 1 << (0x01u | 0x02u) | 1 << (0x01u | 0x04u) | 1 << (0x04u | 0x02u) | 1 << (0x01u | 0x04u | 0x02u) | 1 << (0x08u | 0x02u | 0x01u) | 1 << (0x08u | 0x04u | 0x01u) | 1 << (0x08u | 0x04u | 0x02u) | 1 << (0x08u | 0x04u | 0x01u | 0x02u) ) >> bit) & 1))));
 return z;
}
static inline __attribute__((no_instrument_function)) int gfp_zonelist(gfp_t flags)
{
 if ((1 || 0) && __builtin_expect(!!(flags & (( gfp_t)0x40000u)), 0))
  return 1;

 return 0;
}
static inline __attribute__((no_instrument_function)) struct zonelist *node_zonelist(int nid, gfp_t flags)
{
 return (node_data[nid])->node_zonelists + gfp_zonelist(flags);
}


static inline __attribute__((no_instrument_function)) void arch_free_page(struct page *page, int order) { }


static inline __attribute__((no_instrument_function)) void arch_alloc_page(struct page *page, int order) { }


struct page *
__alloc_pages_nodemask(gfp_t gfp_mask, unsigned int order,
         struct zonelist *zonelist, nodemask_t *nodemask);

static inline __attribute__((no_instrument_function)) struct page *
__alloc_pages(gfp_t gfp_mask, unsigned int order,
  struct zonelist *zonelist)
{
 return __alloc_pages_nodemask(gfp_mask, order, zonelist, ((void *)0));
}

static inline __attribute__((no_instrument_function)) struct page *alloc_pages_node(int nid, gfp_t gfp_mask,
      unsigned int order)
{

 if (nid < 0)
  nid = numa_node_id();

 return __alloc_pages(gfp_mask, order, node_zonelist(nid, gfp_mask));
}

static inline __attribute__((no_instrument_function)) struct page *alloc_pages_exact_node(int nid, gfp_t gfp_mask,
      unsigned int order)
{
 ((void)(sizeof(( long)(nid < 0 || nid >= (1 << 6) || !node_state((nid), N_ONLINE)))));

 return __alloc_pages(gfp_mask, order, node_zonelist(nid, gfp_mask));
}


extern struct page *alloc_pages_current(gfp_t gfp_mask, unsigned order);

static inline __attribute__((no_instrument_function)) struct page *
alloc_pages(gfp_t gfp_mask, unsigned int order)
{
 return alloc_pages_current(gfp_mask, order);
}
extern struct page *alloc_pages_vma(gfp_t gfp_mask, int order,
   struct vm_area_struct *vma, unsigned long addr,
   int node);
extern struct page *alloc_kmem_pages(gfp_t gfp_mask, unsigned int order);
extern struct page *alloc_kmem_pages_node(int nid, gfp_t gfp_mask,
       unsigned int order);

extern unsigned long __get_free_pages(gfp_t gfp_mask, unsigned int order);
extern unsigned long get_zeroed_page(gfp_t gfp_mask);

void *alloc_pages_exact(size_t size, gfp_t gfp_mask);
void free_pages_exact(void *virt, size_t size);

void * __attribute__ ((__section__(".meminit.text"))) __attribute__((__cold__)) __attribute__((no_instrument_function)) alloc_pages_exact_nid(int nid, size_t size, gfp_t gfp_mask);







extern void __free_pages(struct page *page, unsigned int order);
extern void free_pages(unsigned long addr, unsigned int order);
extern void free_hot_cold_page(struct page *page, bool cold);
extern void free_hot_cold_page_list(struct list_head *list, bool cold);

extern void __free_kmem_pages(struct page *page, unsigned int order);
extern void free_kmem_pages(unsigned long addr, unsigned int order);




void page_alloc_init(void);
void drain_zone_pages(struct zone *zone, struct per_cpu_pages *pcp);
void drain_all_pages(void);
void drain_local_pages(void *dummy);
extern gfp_t gfp_allowed_mask;


bool gfp_pfmemalloc_allowed(gfp_t gfp_mask);

extern void pm_restrict_gfp_mask(void);
extern void pm_restore_gfp_mask(void);


extern bool pm_suspended_storage(void);




struct completion;






struct __sysctl_args {
 int *name;
 int nlen;
 void *oldval;
 size_t *oldlenp;
 void *newval;
 size_t newlen;
 unsigned long __unused[4];
};





enum
{
 CTL_KERN=1,
 CTL_VM=2,
 CTL_NET=3,
 CTL_PROC=4,
 CTL_FS=5,
 CTL_DEBUG=6,
 CTL_DEV=7,
 CTL_BUS=8,
 CTL_ABI=9,
 CTL_CPU=10,
 CTL_ARLAN=254,
 CTL_S390DBF=5677,
 CTL_SUNRPC=7249,
 CTL_PM=9899,
 CTL_FRV=9898,
};


enum
{
 CTL_BUS_ISA=1
};


enum
{
 INOTIFY_MAX_USER_INSTANCES=1,
 INOTIFY_MAX_USER_WATCHES=2,
 INOTIFY_MAX_QUEUED_EVENTS=3
};


enum
{
 KERN_OSTYPE=1,
 KERN_OSRELEASE=2,
 KERN_OSREV=3,
 KERN_VERSION=4,
 KERN_SECUREMASK=5,
 KERN_PROF=6,
 KERN_NODENAME=7,
 KERN_DOMAINNAME=8,

 KERN_PANIC=15,
 KERN_REALROOTDEV=16,

 KERN_SPARC_REBOOT=21,
 KERN_CTLALTDEL=22,
 KERN_PRINTK=23,
 KERN_NAMETRANS=24,
 KERN_PPC_HTABRECLAIM=25,
 KERN_PPC_ZEROPAGED=26,
 KERN_PPC_POWERSAVE_NAP=27,
 KERN_MODPROBE=28,
 KERN_SG_BIG_BUFF=29,
 KERN_ACCT=30,
 KERN_PPC_L2CR=31,

 KERN_RTSIGNR=32,
 KERN_RTSIGMAX=33,

 KERN_SHMMAX=34,
 KERN_MSGMAX=35,
 KERN_MSGMNB=36,
 KERN_MSGPOOL=37,
 KERN_SYSRQ=38,
 KERN_MAX_THREADS=39,
  KERN_RANDOM=40,
  KERN_SHMALL=41,
  KERN_MSGMNI=42,
  KERN_SEM=43,
  KERN_SPARC_STOP_A=44,
  KERN_SHMMNI=45,
 KERN_OVERFLOWUID=46,
 KERN_OVERFLOWGID=47,
 KERN_SHMPATH=48,
 KERN_HOTPLUG=49,
 KERN_IEEE_EMULATION_WARNINGS=50,
 KERN_S390_USER_DEBUG_LOGGING=51,
 KERN_CORE_USES_PID=52,
 KERN_TAINTED=53,
 KERN_CADPID=54,
 KERN_PIDMAX=55,
   KERN_CORE_PATTERN=56,
 KERN_PANIC_ON_OOPS=57,
 KERN_HPPA_PWRSW=58,
 KERN_HPPA_UNALIGNED=59,
 KERN_PRINTK_RATELIMIT=60,
 KERN_PRINTK_RATELIMIT_BURST=61,
 KERN_PTY=62,
 KERN_NGROUPS_MAX=63,
 KERN_SPARC_SCONS_PWROFF=64,
 KERN_HZ_TIMER=65,
 KERN_UNKNOWN_NMI_PANIC=66,
 KERN_BOOTLOADER_TYPE=67,
 KERN_RANDOMIZE=68,
 KERN_SETUID_DUMPABLE=69,
 KERN_SPIN_RETRY=70,
 KERN_ACPI_VIDEO_FLAGS=71,
 KERN_IA64_UNALIGNED=72,
 KERN_COMPAT_LOG=73,
 KERN_MAX_LOCK_DEPTH=74,
 KERN_NMI_WATCHDOG=75,
 KERN_PANIC_ON_NMI=76,
};




enum
{
 VM_UNUSED1=1,
 VM_UNUSED2=2,
 VM_UNUSED3=3,
 VM_UNUSED4=4,
 VM_OVERCOMMIT_MEMORY=5,
 VM_UNUSED5=6,
 VM_UNUSED7=7,
 VM_UNUSED8=8,
 VM_UNUSED9=9,
 VM_PAGE_CLUSTER=10,
 VM_DIRTY_BACKGROUND=11,
 VM_DIRTY_RATIO=12,
 VM_DIRTY_WB_CS=13,
 VM_DIRTY_EXPIRE_CS=14,
 VM_NR_PDFLUSH_THREADS=15,
 VM_OVERCOMMIT_RATIO=16,
 VM_PAGEBUF=17,
 VM_HUGETLB_PAGES=18,
 VM_SWAPPINESS=19,
 VM_LOWMEM_RESERVE_RATIO=20,
 VM_MIN_FREE_KBYTES=21,
 VM_MAX_MAP_COUNT=22,
 VM_LAPTOP_MODE=23,
 VM_BLOCK_DUMP=24,
 VM_HUGETLB_GROUP=25,
 VM_VFS_CACHE_PRESSURE=26,
 VM_LEGACY_VA_LAYOUT=27,
 VM_SWAP_TOKEN_TIMEOUT=28,
 VM_DROP_PAGECACHE=29,
 VM_PERCPU_PAGELIST_FRACTION=30,
 VM_ZONE_RECLAIM_MODE=31,
 VM_MIN_UNMAPPED=32,
 VM_PANIC_ON_OOM=33,
 VM_VDSO_ENABLED=34,
 VM_MIN_SLAB=35,
};



enum
{
 NET_CORE=1,
 NET_ETHER=2,
 NET_802=3,
 NET_UNIX=4,
 NET_IPV4=5,
 NET_IPX=6,
 NET_ATALK=7,
 NET_NETROM=8,
 NET_AX25=9,
 NET_BRIDGE=10,
 NET_ROSE=11,
 NET_IPV6=12,
 NET_X25=13,
 NET_TR=14,
 NET_DECNET=15,
 NET_ECONET=16,
 NET_SCTP=17,
 NET_LLC=18,
 NET_NETFILTER=19,
 NET_DCCP=20,
 NET_IRDA=412,
};


enum
{
 RANDOM_POOLSIZE=1,
 RANDOM_ENTROPY_COUNT=2,
 RANDOM_READ_THRESH=3,
 RANDOM_WRITE_THRESH=4,
 RANDOM_BOOT_ID=5,
 RANDOM_UUID=6
};


enum
{
 PTY_MAX=1,
 PTY_NR=2
};


enum
{
 BUS_ISA_MEM_BASE=1,
 BUS_ISA_PORT_BASE=2,
 BUS_ISA_PORT_SHIFT=3
};


enum
{
 NET_CORE_WMEM_MAX=1,
 NET_CORE_RMEM_MAX=2,
 NET_CORE_WMEM_DEFAULT=3,
 NET_CORE_RMEM_DEFAULT=4,

 NET_CORE_MAX_BACKLOG=6,
 NET_CORE_FASTROUTE=7,
 NET_CORE_MSG_COST=8,
 NET_CORE_MSG_BURST=9,
 NET_CORE_OPTMEM_MAX=10,
 NET_CORE_HOT_LIST_LENGTH=11,
 NET_CORE_DIVERT_VERSION=12,
 NET_CORE_NO_CONG_THRESH=13,
 NET_CORE_NO_CONG=14,
 NET_CORE_LO_CONG=15,
 NET_CORE_MOD_CONG=16,
 NET_CORE_DEV_WEIGHT=17,
 NET_CORE_SOMAXCONN=18,
 NET_CORE_BUDGET=19,
 NET_CORE_AEVENT_ETIME=20,
 NET_CORE_AEVENT_RSEQTH=21,
 NET_CORE_WARNINGS=22,
};







enum
{
 NET_UNIX_DESTROY_DELAY=1,
 NET_UNIX_DELETE_DELAY=2,
 NET_UNIX_MAX_DGRAM_QLEN=3,
};


enum
{
 NET_NF_CONNTRACK_MAX=1,
 NET_NF_CONNTRACK_TCP_TIMEOUT_SYN_SENT=2,
 NET_NF_CONNTRACK_TCP_TIMEOUT_SYN_RECV=3,
 NET_NF_CONNTRACK_TCP_TIMEOUT_ESTABLISHED=4,
 NET_NF_CONNTRACK_TCP_TIMEOUT_FIN_WAIT=5,
 NET_NF_CONNTRACK_TCP_TIMEOUT_CLOSE_WAIT=6,
 NET_NF_CONNTRACK_TCP_TIMEOUT_LAST_ACK=7,
 NET_NF_CONNTRACK_TCP_TIMEOUT_TIME_WAIT=8,
 NET_NF_CONNTRACK_TCP_TIMEOUT_CLOSE=9,
 NET_NF_CONNTRACK_UDP_TIMEOUT=10,
 NET_NF_CONNTRACK_UDP_TIMEOUT_STREAM=11,
 NET_NF_CONNTRACK_ICMP_TIMEOUT=12,
 NET_NF_CONNTRACK_GENERIC_TIMEOUT=13,
 NET_NF_CONNTRACK_BUCKETS=14,
 NET_NF_CONNTRACK_LOG_INVALID=15,
 NET_NF_CONNTRACK_TCP_TIMEOUT_MAX_RETRANS=16,
 NET_NF_CONNTRACK_TCP_LOOSE=17,
 NET_NF_CONNTRACK_TCP_BE_LIBERAL=18,
 NET_NF_CONNTRACK_TCP_MAX_RETRANS=19,
 NET_NF_CONNTRACK_SCTP_TIMEOUT_CLOSED=20,
 NET_NF_CONNTRACK_SCTP_TIMEOUT_COOKIE_WAIT=21,
 NET_NF_CONNTRACK_SCTP_TIMEOUT_COOKIE_ECHOED=22,
 NET_NF_CONNTRACK_SCTP_TIMEOUT_ESTABLISHED=23,
 NET_NF_CONNTRACK_SCTP_TIMEOUT_SHUTDOWN_SENT=24,
 NET_NF_CONNTRACK_SCTP_TIMEOUT_SHUTDOWN_RECD=25,
 NET_NF_CONNTRACK_SCTP_TIMEOUT_SHUTDOWN_ACK_SENT=26,
 NET_NF_CONNTRACK_COUNT=27,
 NET_NF_CONNTRACK_ICMPV6_TIMEOUT=28,
 NET_NF_CONNTRACK_FRAG6_TIMEOUT=29,
 NET_NF_CONNTRACK_FRAG6_LOW_THRESH=30,
 NET_NF_CONNTRACK_FRAG6_HIGH_THRESH=31,
 NET_NF_CONNTRACK_CHECKSUM=32,
};


enum
{

 NET_IPV4_FORWARD=8,
 NET_IPV4_DYNADDR=9,

 NET_IPV4_CONF=16,
 NET_IPV4_NEIGH=17,
 NET_IPV4_ROUTE=18,
 NET_IPV4_FIB_HASH=19,
 NET_IPV4_NETFILTER=20,

 NET_IPV4_TCP_TIMESTAMPS=33,
 NET_IPV4_TCP_WINDOW_SCALING=34,
 NET_IPV4_TCP_SACK=35,
 NET_IPV4_TCP_RETRANS_COLLAPSE=36,
 NET_IPV4_DEFAULT_TTL=37,
 NET_IPV4_AUTOCONFIG=38,
 NET_IPV4_NO_PMTU_DISC=39,
 NET_IPV4_TCP_SYN_RETRIES=40,
 NET_IPV4_IPFRAG_HIGH_THRESH=41,
 NET_IPV4_IPFRAG_LOW_THRESH=42,
 NET_IPV4_IPFRAG_TIME=43,
 NET_IPV4_TCP_MAX_KA_PROBES=44,
 NET_IPV4_TCP_KEEPALIVE_TIME=45,
 NET_IPV4_TCP_KEEPALIVE_PROBES=46,
 NET_IPV4_TCP_RETRIES1=47,
 NET_IPV4_TCP_RETRIES2=48,
 NET_IPV4_TCP_FIN_TIMEOUT=49,
 NET_IPV4_IP_MASQ_DEBUG=50,
 NET_TCP_SYNCOOKIES=51,
 NET_TCP_STDURG=52,
 NET_TCP_RFC1337=53,
 NET_TCP_SYN_TAILDROP=54,
 NET_TCP_MAX_SYN_BACKLOG=55,
 NET_IPV4_LOCAL_PORT_RANGE=56,
 NET_IPV4_ICMP_ECHO_IGNORE_ALL=57,
 NET_IPV4_ICMP_ECHO_IGNORE_BROADCASTS=58,
 NET_IPV4_ICMP_SOURCEQUENCH_RATE=59,
 NET_IPV4_ICMP_DESTUNREACH_RATE=60,
 NET_IPV4_ICMP_TIMEEXCEED_RATE=61,
 NET_IPV4_ICMP_PARAMPROB_RATE=62,
 NET_IPV4_ICMP_ECHOREPLY_RATE=63,
 NET_IPV4_ICMP_IGNORE_BOGUS_ERROR_RESPONSES=64,
 NET_IPV4_IGMP_MAX_MEMBERSHIPS=65,
 NET_TCP_TW_RECYCLE=66,
 NET_IPV4_ALWAYS_DEFRAG=67,
 NET_IPV4_TCP_KEEPALIVE_INTVL=68,
 NET_IPV4_INET_PEER_THRESHOLD=69,
 NET_IPV4_INET_PEER_MINTTL=70,
 NET_IPV4_INET_PEER_MAXTTL=71,
 NET_IPV4_INET_PEER_GC_MINTIME=72,
 NET_IPV4_INET_PEER_GC_MAXTIME=73,
 NET_TCP_ORPHAN_RETRIES=74,
 NET_TCP_ABORT_ON_OVERFLOW=75,
 NET_TCP_SYNACK_RETRIES=76,
 NET_TCP_MAX_ORPHANS=77,
 NET_TCP_MAX_TW_BUCKETS=78,
 NET_TCP_FACK=79,
 NET_TCP_REORDERING=80,
 NET_TCP_ECN=81,
 NET_TCP_DSACK=82,
 NET_TCP_MEM=83,
 NET_TCP_WMEM=84,
 NET_TCP_RMEM=85,
 NET_TCP_APP_WIN=86,
 NET_TCP_ADV_WIN_SCALE=87,
 NET_IPV4_NONLOCAL_BIND=88,
 NET_IPV4_ICMP_RATELIMIT=89,
 NET_IPV4_ICMP_RATEMASK=90,
 NET_TCP_TW_REUSE=91,
 NET_TCP_FRTO=92,
 NET_TCP_LOW_LATENCY=93,
 NET_IPV4_IPFRAG_SECRET_INTERVAL=94,
 NET_IPV4_IGMP_MAX_MSF=96,
 NET_TCP_NO_METRICS_SAVE=97,
 NET_TCP_DEFAULT_WIN_SCALE=105,
 NET_TCP_MODERATE_RCVBUF=106,
 NET_TCP_TSO_WIN_DIVISOR=107,
 NET_TCP_BIC_BETA=108,
 NET_IPV4_ICMP_ERRORS_USE_INBOUND_IFADDR=109,
 NET_TCP_CONG_CONTROL=110,
 NET_TCP_ABC=111,
 NET_IPV4_IPFRAG_MAX_DIST=112,
  NET_TCP_MTU_PROBING=113,
 NET_TCP_BASE_MSS=114,
 NET_IPV4_TCP_WORKAROUND_SIGNED_WINDOWS=115,
 NET_TCP_DMA_COPYBREAK=116,
 NET_TCP_SLOW_START_AFTER_IDLE=117,
 NET_CIPSOV4_CACHE_ENABLE=118,
 NET_CIPSOV4_CACHE_BUCKET_SIZE=119,
 NET_CIPSOV4_RBM_OPTFMT=120,
 NET_CIPSOV4_RBM_STRICTVALID=121,
 NET_TCP_AVAIL_CONG_CONTROL=122,
 NET_TCP_ALLOWED_CONG_CONTROL=123,
 NET_TCP_MAX_SSTHRESH=124,
 NET_TCP_FRTO_RESPONSE=125,
};

enum {
 NET_IPV4_ROUTE_FLUSH=1,
 NET_IPV4_ROUTE_MIN_DELAY=2,
 NET_IPV4_ROUTE_MAX_DELAY=3,
 NET_IPV4_ROUTE_GC_THRESH=4,
 NET_IPV4_ROUTE_MAX_SIZE=5,
 NET_IPV4_ROUTE_GC_MIN_INTERVAL=6,
 NET_IPV4_ROUTE_GC_TIMEOUT=7,
 NET_IPV4_ROUTE_GC_INTERVAL=8,
 NET_IPV4_ROUTE_REDIRECT_LOAD=9,
 NET_IPV4_ROUTE_REDIRECT_NUMBER=10,
 NET_IPV4_ROUTE_REDIRECT_SILENCE=11,
 NET_IPV4_ROUTE_ERROR_COST=12,
 NET_IPV4_ROUTE_ERROR_BURST=13,
 NET_IPV4_ROUTE_GC_ELASTICITY=14,
 NET_IPV4_ROUTE_MTU_EXPIRES=15,
 NET_IPV4_ROUTE_MIN_PMTU=16,
 NET_IPV4_ROUTE_MIN_ADVMSS=17,
 NET_IPV4_ROUTE_SECRET_INTERVAL=18,
 NET_IPV4_ROUTE_GC_MIN_INTERVAL_MS=19,
};

enum
{
 NET_PROTO_CONF_ALL=-2,
 NET_PROTO_CONF_DEFAULT=-3


};

enum
{
 NET_IPV4_CONF_FORWARDING=1,
 NET_IPV4_CONF_MC_FORWARDING=2,
 NET_IPV4_CONF_PROXY_ARP=3,
 NET_IPV4_CONF_ACCEPT_REDIRECTS=4,
 NET_IPV4_CONF_SECURE_REDIRECTS=5,
 NET_IPV4_CONF_SEND_REDIRECTS=6,
 NET_IPV4_CONF_SHARED_MEDIA=7,
 NET_IPV4_CONF_RP_FILTER=8,
 NET_IPV4_CONF_ACCEPT_SOURCE_ROUTE=9,
 NET_IPV4_CONF_BOOTP_RELAY=10,
 NET_IPV4_CONF_LOG_MARTIANS=11,
 NET_IPV4_CONF_TAG=12,
 NET_IPV4_CONF_ARPFILTER=13,
 NET_IPV4_CONF_MEDIUM_ID=14,
 NET_IPV4_CONF_NOXFRM=15,
 NET_IPV4_CONF_NOPOLICY=16,
 NET_IPV4_CONF_FORCE_IGMP_VERSION=17,
 NET_IPV4_CONF_ARP_ANNOUNCE=18,
 NET_IPV4_CONF_ARP_IGNORE=19,
 NET_IPV4_CONF_PROMOTE_SECONDARIES=20,
 NET_IPV4_CONF_ARP_ACCEPT=21,
 NET_IPV4_CONF_ARP_NOTIFY=22,
};


enum
{
 NET_IPV4_NF_CONNTRACK_MAX=1,
 NET_IPV4_NF_CONNTRACK_TCP_TIMEOUT_SYN_SENT=2,
 NET_IPV4_NF_CONNTRACK_TCP_TIMEOUT_SYN_RECV=3,
 NET_IPV4_NF_CONNTRACK_TCP_TIMEOUT_ESTABLISHED=4,
 NET_IPV4_NF_CONNTRACK_TCP_TIMEOUT_FIN_WAIT=5,
 NET_IPV4_NF_CONNTRACK_TCP_TIMEOUT_CLOSE_WAIT=6,
 NET_IPV4_NF_CONNTRACK_TCP_TIMEOUT_LAST_ACK=7,
 NET_IPV4_NF_CONNTRACK_TCP_TIMEOUT_TIME_WAIT=8,
 NET_IPV4_NF_CONNTRACK_TCP_TIMEOUT_CLOSE=9,
 NET_IPV4_NF_CONNTRACK_UDP_TIMEOUT=10,
 NET_IPV4_NF_CONNTRACK_UDP_TIMEOUT_STREAM=11,
 NET_IPV4_NF_CONNTRACK_ICMP_TIMEOUT=12,
 NET_IPV4_NF_CONNTRACK_GENERIC_TIMEOUT=13,
 NET_IPV4_NF_CONNTRACK_BUCKETS=14,
 NET_IPV4_NF_CONNTRACK_LOG_INVALID=15,
 NET_IPV4_NF_CONNTRACK_TCP_TIMEOUT_MAX_RETRANS=16,
 NET_IPV4_NF_CONNTRACK_TCP_LOOSE=17,
 NET_IPV4_NF_CONNTRACK_TCP_BE_LIBERAL=18,
 NET_IPV4_NF_CONNTRACK_TCP_MAX_RETRANS=19,
  NET_IPV4_NF_CONNTRACK_SCTP_TIMEOUT_CLOSED=20,
  NET_IPV4_NF_CONNTRACK_SCTP_TIMEOUT_COOKIE_WAIT=21,
  NET_IPV4_NF_CONNTRACK_SCTP_TIMEOUT_COOKIE_ECHOED=22,
  NET_IPV4_NF_CONNTRACK_SCTP_TIMEOUT_ESTABLISHED=23,
  NET_IPV4_NF_CONNTRACK_SCTP_TIMEOUT_SHUTDOWN_SENT=24,
  NET_IPV4_NF_CONNTRACK_SCTP_TIMEOUT_SHUTDOWN_RECD=25,
  NET_IPV4_NF_CONNTRACK_SCTP_TIMEOUT_SHUTDOWN_ACK_SENT=26,
 NET_IPV4_NF_CONNTRACK_COUNT=27,
 NET_IPV4_NF_CONNTRACK_CHECKSUM=28,
};


enum {
 NET_IPV6_CONF=16,
 NET_IPV6_NEIGH=17,
 NET_IPV6_ROUTE=18,
 NET_IPV6_ICMP=19,
 NET_IPV6_BINDV6ONLY=20,
 NET_IPV6_IP6FRAG_HIGH_THRESH=21,
 NET_IPV6_IP6FRAG_LOW_THRESH=22,
 NET_IPV6_IP6FRAG_TIME=23,
 NET_IPV6_IP6FRAG_SECRET_INTERVAL=24,
 NET_IPV6_MLD_MAX_MSF=25,
};

enum {
 NET_IPV6_ROUTE_FLUSH=1,
 NET_IPV6_ROUTE_GC_THRESH=2,
 NET_IPV6_ROUTE_MAX_SIZE=3,
 NET_IPV6_ROUTE_GC_MIN_INTERVAL=4,
 NET_IPV6_ROUTE_GC_TIMEOUT=5,
 NET_IPV6_ROUTE_GC_INTERVAL=6,
 NET_IPV6_ROUTE_GC_ELASTICITY=7,
 NET_IPV6_ROUTE_MTU_EXPIRES=8,
 NET_IPV6_ROUTE_MIN_ADVMSS=9,
 NET_IPV6_ROUTE_GC_MIN_INTERVAL_MS=10
};

enum {
 NET_IPV6_FORWARDING=1,
 NET_IPV6_HOP_LIMIT=2,
 NET_IPV6_MTU=3,
 NET_IPV6_ACCEPT_RA=4,
 NET_IPV6_ACCEPT_REDIRECTS=5,
 NET_IPV6_AUTOCONF=6,
 NET_IPV6_DAD_TRANSMITS=7,
 NET_IPV6_RTR_SOLICITS=8,
 NET_IPV6_RTR_SOLICIT_INTERVAL=9,
 NET_IPV6_RTR_SOLICIT_DELAY=10,
 NET_IPV6_USE_TEMPADDR=11,
 NET_IPV6_TEMP_VALID_LFT=12,
 NET_IPV6_TEMP_PREFERED_LFT=13,
 NET_IPV6_REGEN_MAX_RETRY=14,
 NET_IPV6_MAX_DESYNC_FACTOR=15,
 NET_IPV6_MAX_ADDRESSES=16,
 NET_IPV6_FORCE_MLD_VERSION=17,
 NET_IPV6_ACCEPT_RA_DEFRTR=18,
 NET_IPV6_ACCEPT_RA_PINFO=19,
 NET_IPV6_ACCEPT_RA_RTR_PREF=20,
 NET_IPV6_RTR_PROBE_INTERVAL=21,
 NET_IPV6_ACCEPT_RA_RT_INFO_MAX_PLEN=22,
 NET_IPV6_PROXY_NDP=23,
 NET_IPV6_ACCEPT_SOURCE_ROUTE=25,
 NET_IPV6_ACCEPT_RA_FROM_LOCAL=26,
 __NET_IPV6_MAX
};


enum {
 NET_IPV6_ICMP_RATELIMIT=1
};


enum {
 NET_NEIGH_MCAST_SOLICIT=1,
 NET_NEIGH_UCAST_SOLICIT=2,
 NET_NEIGH_APP_SOLICIT=3,
 NET_NEIGH_RETRANS_TIME=4,
 NET_NEIGH_REACHABLE_TIME=5,
 NET_NEIGH_DELAY_PROBE_TIME=6,
 NET_NEIGH_GC_STALE_TIME=7,
 NET_NEIGH_UNRES_QLEN=8,
 NET_NEIGH_PROXY_QLEN=9,
 NET_NEIGH_ANYCAST_DELAY=10,
 NET_NEIGH_PROXY_DELAY=11,
 NET_NEIGH_LOCKTIME=12,
 NET_NEIGH_GC_INTERVAL=13,
 NET_NEIGH_GC_THRESH1=14,
 NET_NEIGH_GC_THRESH2=15,
 NET_NEIGH_GC_THRESH3=16,
 NET_NEIGH_RETRANS_TIME_MS=17,
 NET_NEIGH_REACHABLE_TIME_MS=18,
};


enum {
 NET_DCCP_DEFAULT=1,
};


enum {
 NET_IPX_PPROP_BROADCASTING=1,
 NET_IPX_FORWARDING=2
};


enum {
 NET_LLC2=1,
 NET_LLC_STATION=2,
};


enum {
 NET_LLC2_TIMEOUT=1,
};


enum {
 NET_LLC_STATION_ACK_TIMEOUT=1,
};


enum {
 NET_LLC2_ACK_TIMEOUT=1,
 NET_LLC2_P_TIMEOUT=2,
 NET_LLC2_REJ_TIMEOUT=3,
 NET_LLC2_BUSY_TIMEOUT=4,
};


enum {
 NET_ATALK_AARP_EXPIRY_TIME=1,
 NET_ATALK_AARP_TICK_TIME=2,
 NET_ATALK_AARP_RETRANSMIT_LIMIT=3,
 NET_ATALK_AARP_RESOLVE_TIME=4
};



enum {
 NET_NETROM_DEFAULT_PATH_QUALITY=1,
 NET_NETROM_OBSOLESCENCE_COUNT_INITIALISER=2,
 NET_NETROM_NETWORK_TTL_INITIALISER=3,
 NET_NETROM_TRANSPORT_TIMEOUT=4,
 NET_NETROM_TRANSPORT_MAXIMUM_TRIES=5,
 NET_NETROM_TRANSPORT_ACKNOWLEDGE_DELAY=6,
 NET_NETROM_TRANSPORT_BUSY_DELAY=7,
 NET_NETROM_TRANSPORT_REQUESTED_WINDOW_SIZE=8,
 NET_NETROM_TRANSPORT_NO_ACTIVITY_TIMEOUT=9,
 NET_NETROM_ROUTING_CONTROL=10,
 NET_NETROM_LINK_FAILS_COUNT=11,
 NET_NETROM_RESET=12
};


enum {
 NET_AX25_IP_DEFAULT_MODE=1,
 NET_AX25_DEFAULT_MODE=2,
 NET_AX25_BACKOFF_TYPE=3,
 NET_AX25_CONNECT_MODE=4,
 NET_AX25_STANDARD_WINDOW=5,
 NET_AX25_EXTENDED_WINDOW=6,
 NET_AX25_T1_TIMEOUT=7,
 NET_AX25_T2_TIMEOUT=8,
 NET_AX25_T3_TIMEOUT=9,
 NET_AX25_IDLE_TIMEOUT=10,
 NET_AX25_N2=11,
 NET_AX25_PACLEN=12,
 NET_AX25_PROTOCOL=13,
 NET_AX25_DAMA_SLAVE_TIMEOUT=14
};


enum {
 NET_ROSE_RESTART_REQUEST_TIMEOUT=1,
 NET_ROSE_CALL_REQUEST_TIMEOUT=2,
 NET_ROSE_RESET_REQUEST_TIMEOUT=3,
 NET_ROSE_CLEAR_REQUEST_TIMEOUT=4,
 NET_ROSE_ACK_HOLD_BACK_TIMEOUT=5,
 NET_ROSE_ROUTING_CONTROL=6,
 NET_ROSE_LINK_FAIL_TIMEOUT=7,
 NET_ROSE_MAX_VCS=8,
 NET_ROSE_WINDOW_SIZE=9,
 NET_ROSE_NO_ACTIVITY_TIMEOUT=10
};


enum {
 NET_X25_RESTART_REQUEST_TIMEOUT=1,
 NET_X25_CALL_REQUEST_TIMEOUT=2,
 NET_X25_RESET_REQUEST_TIMEOUT=3,
 NET_X25_CLEAR_REQUEST_TIMEOUT=4,
 NET_X25_ACK_HOLD_BACK_TIMEOUT=5,
 NET_X25_FORWARD=6
};


enum
{
 NET_TR_RIF_TIMEOUT=1
};


enum {
 NET_DECNET_NODE_TYPE = 1,
 NET_DECNET_NODE_ADDRESS = 2,
 NET_DECNET_NODE_NAME = 3,
 NET_DECNET_DEFAULT_DEVICE = 4,
 NET_DECNET_TIME_WAIT = 5,
 NET_DECNET_DN_COUNT = 6,
 NET_DECNET_DI_COUNT = 7,
 NET_DECNET_DR_COUNT = 8,
 NET_DECNET_DST_GC_INTERVAL = 9,
 NET_DECNET_CONF = 10,
 NET_DECNET_NO_FC_MAX_CWND = 11,
 NET_DECNET_MEM = 12,
 NET_DECNET_RMEM = 13,
 NET_DECNET_WMEM = 14,
 NET_DECNET_DEBUG_LEVEL = 255
};


enum {
 NET_DECNET_CONF_LOOPBACK = -2,
 NET_DECNET_CONF_DDCMP = -3,
 NET_DECNET_CONF_PPP = -4,
 NET_DECNET_CONF_X25 = -5,
 NET_DECNET_CONF_GRE = -6,
 NET_DECNET_CONF_ETHER = -7


};


enum {
 NET_DECNET_CONF_DEV_PRIORITY = 1,
 NET_DECNET_CONF_DEV_T1 = 2,
 NET_DECNET_CONF_DEV_T2 = 3,
 NET_DECNET_CONF_DEV_T3 = 4,
 NET_DECNET_CONF_DEV_FORWARDING = 5,
 NET_DECNET_CONF_DEV_BLKSIZE = 6,
 NET_DECNET_CONF_DEV_STATE = 7
};


enum {
 NET_SCTP_RTO_INITIAL = 1,
 NET_SCTP_RTO_MIN = 2,
 NET_SCTP_RTO_MAX = 3,
 NET_SCTP_RTO_ALPHA = 4,
 NET_SCTP_RTO_BETA = 5,
 NET_SCTP_VALID_COOKIE_LIFE = 6,
 NET_SCTP_ASSOCIATION_MAX_RETRANS = 7,
 NET_SCTP_PATH_MAX_RETRANS = 8,
 NET_SCTP_MAX_INIT_RETRANSMITS = 9,
 NET_SCTP_HB_INTERVAL = 10,
 NET_SCTP_PRESERVE_ENABLE = 11,
 NET_SCTP_MAX_BURST = 12,
 NET_SCTP_ADDIP_ENABLE = 13,
 NET_SCTP_PRSCTP_ENABLE = 14,
 NET_SCTP_SNDBUF_POLICY = 15,
 NET_SCTP_SACK_TIMEOUT = 16,
 NET_SCTP_RCVBUF_POLICY = 17,
};


enum {
 NET_BRIDGE_NF_CALL_ARPTABLES = 1,
 NET_BRIDGE_NF_CALL_IPTABLES = 2,
 NET_BRIDGE_NF_CALL_IP6TABLES = 3,
 NET_BRIDGE_NF_FILTER_VLAN_TAGGED = 4,
 NET_BRIDGE_NF_FILTER_PPPOE_TAGGED = 5,
};


enum {
 NET_IRDA_DISCOVERY=1,
 NET_IRDA_DEVNAME=2,
 NET_IRDA_DEBUG=3,
 NET_IRDA_FAST_POLL=4,
 NET_IRDA_DISCOVERY_SLOTS=5,
 NET_IRDA_DISCOVERY_TIMEOUT=6,
 NET_IRDA_SLOT_TIMEOUT=7,
 NET_IRDA_MAX_BAUD_RATE=8,
 NET_IRDA_MIN_TX_TURN_TIME=9,
 NET_IRDA_MAX_TX_DATA_SIZE=10,
 NET_IRDA_MAX_TX_WINDOW=11,
 NET_IRDA_MAX_NOREPLY_TIME=12,
 NET_IRDA_WARN_NOREPLY_TIME=13,
 NET_IRDA_LAP_KEEPALIVE_TIME=14,
};



enum
{
 FS_NRINODE=1,
 FS_STATINODE=2,
 FS_MAXINODE=3,
 FS_NRDQUOT=4,
 FS_MAXDQUOT=5,
 FS_NRFILE=6,
 FS_MAXFILE=7,
 FS_DENTRY=8,
 FS_NRSUPER=9,
 FS_MAXSUPER=10,
 FS_OVERFLOWUID=11,
 FS_OVERFLOWGID=12,
 FS_LEASES=13,
 FS_DIR_NOTIFY=14,
 FS_LEASE_TIME=15,
 FS_DQSTATS=16,
 FS_XFS=17,
 FS_AIO_NR=18,
 FS_AIO_MAX_NR=19,
 FS_INOTIFY=20,
 FS_OCFS2=988,
};


enum {
 FS_DQ_LOOKUPS = 1,
 FS_DQ_DROPS = 2,
 FS_DQ_READS = 3,
 FS_DQ_WRITES = 4,
 FS_DQ_CACHE_HITS = 5,
 FS_DQ_ALLOCATED = 6,
 FS_DQ_FREE = 7,
 FS_DQ_SYNCS = 8,
 FS_DQ_WARNINGS = 9,
};




enum {
 DEV_CDROM=1,
 DEV_HWMON=2,
 DEV_PARPORT=3,
 DEV_RAID=4,
 DEV_MAC_HID=5,
 DEV_SCSI=6,
 DEV_IPMI=7,
};


enum {
 DEV_CDROM_INFO=1,
 DEV_CDROM_AUTOCLOSE=2,
 DEV_CDROM_AUTOEJECT=3,
 DEV_CDROM_DEBUG=4,
 DEV_CDROM_LOCK=5,
 DEV_CDROM_CHECK_MEDIA=6
};


enum {
 DEV_PARPORT_DEFAULT=-3
};


enum {
 DEV_RAID_SPEED_LIMIT_MIN=1,
 DEV_RAID_SPEED_LIMIT_MAX=2
};


enum {
 DEV_PARPORT_DEFAULT_TIMESLICE=1,
 DEV_PARPORT_DEFAULT_SPINTIME=2
};


enum {
 DEV_PARPORT_SPINTIME=1,
 DEV_PARPORT_BASE_ADDR=2,
 DEV_PARPORT_IRQ=3,
 DEV_PARPORT_DMA=4,
 DEV_PARPORT_MODES=5,
 DEV_PARPORT_DEVICES=6,
 DEV_PARPORT_AUTOPROBE=16
};


enum {
 DEV_PARPORT_DEVICES_ACTIVE=-3,
};


enum {
 DEV_PARPORT_DEVICE_TIMESLICE=1,
};


enum {
 DEV_MAC_HID_KEYBOARD_SENDS_LINUX_KEYCODES=1,
 DEV_MAC_HID_KEYBOARD_LOCK_KEYCODES=2,
 DEV_MAC_HID_MOUSE_BUTTON_EMULATION=3,
 DEV_MAC_HID_MOUSE_BUTTON2_KEYCODE=4,
 DEV_MAC_HID_MOUSE_BUTTON3_KEYCODE=5,
 DEV_MAC_HID_ADB_MOUSE_SENDS_KEYCODES=6
};


enum {
 DEV_SCSI_LOGGING_LEVEL=1,
};


enum {
 DEV_IPMI_POWEROFF_POWERCYCLE=1,
};


enum
{
 ABI_DEFHANDLER_COFF=1,
 ABI_DEFHANDLER_ELF=2,
 ABI_DEFHANDLER_LCALL7=3,
 ABI_DEFHANDLER_LIBCSO=4,
 ABI_TRACE=5,
 ABI_FAKE_UTSNAME=6,
};


struct ctl_table;
struct nsproxy;
struct ctl_table_root;
struct ctl_table_header;
struct ctl_dir;

typedef int proc_handler (struct ctl_table *ctl, int write,
     void *buffer, size_t *lenp, loff_t *ppos);

extern int proc_dostring(struct ctl_table *, int,
    void *, size_t *, loff_t *);
extern int proc_dointvec(struct ctl_table *, int,
    void *, size_t *, loff_t *);
extern int proc_dointvec_minmax(struct ctl_table *, int,
    void *, size_t *, loff_t *);
extern int proc_dointvec_jiffies(struct ctl_table *, int,
     void *, size_t *, loff_t *);
extern int proc_dointvec_userhz_jiffies(struct ctl_table *, int,
     void *, size_t *, loff_t *);
extern int proc_dointvec_ms_jiffies(struct ctl_table *, int,
        void *, size_t *, loff_t *);
extern int proc_doulongvec_minmax(struct ctl_table *, int,
      void *, size_t *, loff_t *);
extern int proc_doulongvec_ms_jiffies_minmax(struct ctl_table *table, int,
          void *, size_t *, loff_t *);
extern int proc_do_large_bitmap(struct ctl_table *, int,
    void *, size_t *, loff_t *);
struct ctl_table_poll {
 atomic_t event;
 wait_queue_head_t wait;
};

static inline __attribute__((no_instrument_function)) void *proc_sys_poll_event(struct ctl_table_poll *poll)
{
 return (void *)(unsigned long)atomic_read(&poll->event);
}
struct ctl_table
{
 const char *procname;
 void *data;
 int maxlen;
 umode_t mode;
 struct ctl_table *child;
 proc_handler *proc_handler;
 struct ctl_table_poll *poll;
 void *extra1;
 void *extra2;
};

struct ctl_node {
 struct rb_node node;
 struct ctl_table_header *header;
};



struct ctl_table_header
{
 union {
  struct {
   struct ctl_table *ctl_table;
   int used;
   int count;
   int nreg;
  };
  struct callback_head rcu;
 };
 struct completion *unregistering;
 struct ctl_table *ctl_table_arg;
 struct ctl_table_root *root;
 struct ctl_table_set *set;
 struct ctl_dir *parent;
 struct ctl_node *node;
};

struct ctl_dir {

 struct ctl_table_header header;
 struct rb_root root;
};

struct ctl_table_set {
 int (*is_seen)(struct ctl_table_set *);
 struct ctl_dir dir;
};

struct ctl_table_root {
 struct ctl_table_set default_set;
 struct ctl_table_set *(*lookup)(struct ctl_table_root *root,
        struct nsproxy *namespaces);
 int (*permissions)(struct ctl_table_header *head, struct ctl_table *table);
};


struct ctl_path {
 const char *procname;
};



void proc_sys_poll_notify(struct ctl_table_poll *poll);

extern void setup_sysctl_set(struct ctl_table_set *p,
 struct ctl_table_root *root,
 int (*is_seen)(struct ctl_table_set *));
extern void retire_sysctl_set(struct ctl_table_set *set);

void register_sysctl_root(struct ctl_table_root *root);
struct ctl_table_header *__register_sysctl_table(
 struct ctl_table_set *set,
 const char *path, struct ctl_table *table);
struct ctl_table_header *__register_sysctl_paths(
 struct ctl_table_set *set,
 const struct ctl_path *path, struct ctl_table *table);
struct ctl_table_header *register_sysctl(const char *path, struct ctl_table *table);
struct ctl_table_header *register_sysctl_table(struct ctl_table * table);
struct ctl_table_header *register_sysctl_paths(const struct ctl_path *path,
      struct ctl_table *table);

void unregister_sysctl_table(struct ctl_table_header * table);

extern int sysctl_init(void);




extern char modprobe_path[];


extern __attribute__((format(printf, 2, 3)))
int __request_module(bool wait, const char *name, ...);
struct cred;
struct file;






struct subprocess_info {
 struct work_struct work;
 struct completion *complete;
 char *path;
 char **argv;
 char **envp;
 int wait;
 int retval;
 int (*init)(struct subprocess_info *info, struct cred *new);
 void (*cleanup)(struct subprocess_info *info);
 void *data;
};

extern int
call_usermodehelper(char *path, char **argv, char **envp, int wait);

extern struct subprocess_info *
call_usermodehelper_setup(char *path, char **argv, char **envp, gfp_t gfp_mask,
     int (*init)(struct subprocess_info *info, struct cred *new),
     void (*cleanup)(struct subprocess_info *), void *data);

extern int
call_usermodehelper_exec(struct subprocess_info *info, int wait);

extern struct ctl_table usermodehelper_table[];

enum umh_disable_depth {
 UMH_ENABLED = 0,
 UMH_FREEZING,
 UMH_DISABLED,
};

extern void usermodehelper_init(void);

extern int __usermodehelper_disable(enum umh_disable_depth depth);
extern void __usermodehelper_set_disable_depth(enum umh_disable_depth depth);

static inline __attribute__((no_instrument_function)) int usermodehelper_disable(void)
{
 return __usermodehelper_disable(UMH_DISABLED);
}

static inline __attribute__((no_instrument_function)) void usermodehelper_enable(void)
{
 __usermodehelper_set_disable_depth(UMH_ENABLED);
}

extern int usermodehelper_read_trylock(void);
extern long usermodehelper_read_lock_wait(long timeout);
extern void usermodehelper_read_unlock(void);









struct user_i387_struct {
 unsigned short cwd;
 unsigned short swd;
 unsigned short twd;

 unsigned short fop;
 __u64 rip;
 __u64 rdp;
 __u32 mxcsr;
 __u32 mxcsr_mask;
 __u32 st_space[32];
 __u32 xmm_space[64];
 __u32 padding[24];
};




struct user_regs_struct {
 unsigned long r15;
 unsigned long r14;
 unsigned long r13;
 unsigned long r12;
 unsigned long bp;
 unsigned long bx;
 unsigned long r11;
 unsigned long r10;
 unsigned long r9;
 unsigned long r8;
 unsigned long ax;
 unsigned long cx;
 unsigned long dx;
 unsigned long si;
 unsigned long di;
 unsigned long orig_ax;
 unsigned long ip;
 unsigned long cs;
 unsigned long flags;
 unsigned long sp;
 unsigned long ss;
 unsigned long fs_base;
 unsigned long gs_base;
 unsigned long ds;
 unsigned long es;
 unsigned long fs;
 unsigned long gs;
};





struct user {


  struct user_regs_struct regs;

  int u_fpvalid;

  int pad0;
  struct user_i387_struct i387;

  unsigned long int u_tsize;
  unsigned long int u_dsize;
  unsigned long int u_ssize;
  unsigned long start_code;
  unsigned long start_stack;



  long int signal;
  int reserved;
  int pad1;
  unsigned long u_ar0;

  struct user_i387_struct *u_fpstate;
  unsigned long magic;
  char u_comm[32];
  unsigned long u_debugreg[8];
  unsigned long error_code;
  unsigned long fault_address;
};




struct user_ymmh_regs {

 __u32 ymmh_space[64];
};

struct user_xsave_hdr {
 __u64 xstate_bv;
 __u64 reserved1[2];
 __u64 reserved2[5];
};
struct user_xstateregs {
 struct {
  __u64 fpx_space[58];
  __u64 xstate_fx_sw[6];
 } i387;
 struct user_xsave_hdr xsave_hdr;
 struct user_ymmh_regs ymmh;

};

typedef unsigned long elf_greg_t;


typedef elf_greg_t elf_gregset_t[(sizeof(struct user_regs_struct) / sizeof(elf_greg_t))];

typedef struct user_i387_struct elf_fpregset_t;






enum page_debug_flags {
 PAGE_DEBUG_FLAG_POISON,
 PAGE_DEBUG_FLAG_GUARD,
};
struct vm_area_struct;
struct mm_struct;
struct inode;
struct notifier_block;
struct page;






enum uprobe_filter_ctx {
 UPROBE_FILTER_REGISTER,
 UPROBE_FILTER_UNREGISTER,
 UPROBE_FILTER_MMAP,
};

struct uprobe_consumer {
 int (*handler)(struct uprobe_consumer *self, struct pt_regs *regs);
 int (*ret_handler)(struct uprobe_consumer *self,
    unsigned long func,
    struct pt_regs *regs);
 bool (*filter)(struct uprobe_consumer *self,
    enum uprobe_filter_ctx ctx,
    struct mm_struct *mm);

 struct uprobe_consumer *next;
};
struct uprobes_state {
};



static inline __attribute__((no_instrument_function)) int
uprobe_register(struct inode *inode, loff_t offset, struct uprobe_consumer *uc)
{
 return -38;
}
static inline __attribute__((no_instrument_function)) int
uprobe_apply(struct inode *inode, loff_t offset, struct uprobe_consumer *uc, bool add)
{
 return -38;
}
static inline __attribute__((no_instrument_function)) void
uprobe_unregister(struct inode *inode, loff_t offset, struct uprobe_consumer *uc)
{
}
static inline __attribute__((no_instrument_function)) int uprobe_mmap(struct vm_area_struct *vma)
{
 return 0;
}
static inline __attribute__((no_instrument_function)) void
uprobe_munmap(struct vm_area_struct *vma, unsigned long start, unsigned long end)
{
}
static inline __attribute__((no_instrument_function)) void uprobe_start_dup_mmap(void)
{
}
static inline __attribute__((no_instrument_function)) void uprobe_end_dup_mmap(void)
{
}
static inline __attribute__((no_instrument_function)) void
uprobe_dup_mmap(struct mm_struct *oldmm, struct mm_struct *newmm)
{
}
static inline __attribute__((no_instrument_function)) void uprobe_notify_resume(struct pt_regs *regs)
{
}
static inline __attribute__((no_instrument_function)) bool uprobe_deny_signal(void)
{
 return false;
}
static inline __attribute__((no_instrument_function)) void uprobe_free_utask(struct task_struct *t)
{
}
static inline __attribute__((no_instrument_function)) void uprobe_copy_process(struct task_struct *t, unsigned long flags)
{
}
static inline __attribute__((no_instrument_function)) void uprobe_clear_state(struct mm_struct *mm)
{
}
struct address_space;
struct page {

 unsigned long flags;

 union {
  struct address_space *mapping;






  void *s_mem;
 };


 struct {
  union {
   unsigned long index;
   void *freelist;
   bool pfmemalloc;
  };

  union {



   unsigned long counters;
   struct {

    union {
     atomic_t _mapcount;

     struct {
      unsigned inuse:16;
      unsigned objects:15;
      unsigned frozen:1;
     };
     int units;
    };
    atomic_t _count;
   };
   unsigned int active;
  };
 };


 union {
  struct list_head lru;




  struct {
   struct page *next;

   int pages;
   int pobjects;




  };

  struct slab *slab_page;
  struct callback_head callback_head;



  pgtable_t pmd_huge_pte;

 };


 union {
  unsigned long private;
  spinlock_t ptl;


  struct kmem_cache *slab_cache;
  struct page *first_page;
 };
}





 __attribute__((aligned(2 * sizeof(unsigned long))))

;

struct page_frag {
 struct page *page;

 __u32 offset;
 __u32 size;




};

typedef unsigned long vm_flags_t;






struct vm_region {
 struct rb_node vm_rb;
 vm_flags_t vm_flags;
 unsigned long vm_start;
 unsigned long vm_end;
 unsigned long vm_top;
 unsigned long vm_pgoff;
 struct file *vm_file;

 int vm_usage;
 bool vm_icache_flushed : 1;

};







struct vm_area_struct {


 unsigned long vm_start;
 unsigned long vm_end;



 struct vm_area_struct *vm_next, *vm_prev;

 struct rb_node vm_rb;







 unsigned long rb_subtree_gap;



 struct mm_struct *vm_mm;
 pgprot_t vm_page_prot;
 unsigned long vm_flags;






 union {
  struct {
   struct rb_node rb;
   unsigned long rb_subtree_last;
  } linear;
  struct list_head nonlinear;
 } shared;







 struct list_head anon_vma_chain;

 struct anon_vma *anon_vma;


 const struct vm_operations_struct *vm_ops;


 unsigned long vm_pgoff;

 struct file * vm_file;
 void * vm_private_data;





 struct mempolicy *vm_policy;

};

struct core_thread {
 struct task_struct *task;
 struct core_thread *next;
};

struct core_state {
 atomic_t nr_threads;
 struct core_thread dumper;
 struct completion startup;
};

enum {
 MM_FILEPAGES,
 MM_ANONPAGES,
 MM_SWAPENTS,
 NR_MM_COUNTERS
};




struct task_rss_stat {
 int events;
 int count[NR_MM_COUNTERS];
};


struct mm_rss_stat {
 atomic_long_t count[NR_MM_COUNTERS];
};

struct kioctx_table;
struct mm_struct {
 struct vm_area_struct *mmap;
 struct rb_root mm_rb;
 u32 vmacache_seqnum;

 unsigned long (*get_unmapped_area) (struct file *filp,
    unsigned long addr, unsigned long len,
    unsigned long pgoff, unsigned long flags);

 unsigned long mmap_base;
 unsigned long mmap_legacy_base;
 unsigned long task_size;
 unsigned long highest_vm_end;
 pgd_t * pgd;
 atomic_t mm_users;
 atomic_t mm_count;
 atomic_long_t nr_ptes;
 int map_count;

 spinlock_t page_table_lock;
 struct rw_semaphore mmap_sem;

 struct list_head mmlist;





 unsigned long hiwater_rss;
 unsigned long hiwater_vm;

 unsigned long total_vm;
 unsigned long locked_vm;
 unsigned long pinned_vm;
 unsigned long shared_vm;
 unsigned long exec_vm;
 unsigned long stack_vm;
 unsigned long def_flags;
 unsigned long start_code, end_code, start_data, end_data;
 unsigned long start_brk, brk, start_stack;
 unsigned long arg_start, arg_end, env_start, env_end;

 unsigned long saved_auxv[(2*(2 + 20 + 1))];





 struct mm_rss_stat rss_stat;

 struct linux_binfmt *binfmt;

 cpumask_var_t cpu_vm_mask_var;


 mm_context_t context;

 unsigned long flags;

 struct core_state *core_state;

 spinlock_t ioctx_lock;
 struct kioctx_table *ioctx_table;
 struct file *exe_file;

 struct mmu_notifier_mm *mmu_notifier_mm;
 bool tlb_flush_pending;

 struct uprobes_state uprobes_state;
};

static inline __attribute__((no_instrument_function)) void mm_init_cpumask(struct mm_struct *mm)
{



 cpumask_clear(mm->cpu_vm_mask_var);
}


static inline __attribute__((no_instrument_function)) cpumask_t *mm_cpumask(struct mm_struct *mm)
{
 return mm->cpu_vm_mask_var;
}
static inline __attribute__((no_instrument_function)) bool mm_tlb_flush_pending(struct mm_struct *mm)
{
 __asm__ __volatile__("": : :"memory");
 return mm->tlb_flush_pending;
}
static inline __attribute__((no_instrument_function)) void set_tlb_flush_pending(struct mm_struct *mm)
{
 mm->tlb_flush_pending = true;





 __asm__ __volatile__("": : :"memory");
}

static inline __attribute__((no_instrument_function)) void clear_tlb_flush_pending(struct mm_struct *mm)
{
 __asm__ __volatile__("": : :"memory");
 mm->tlb_flush_pending = false;
}
struct vm_special_mapping
{
 const char *name;
 struct page **pages;
};

enum tlb_flush_reason {
 TLB_FLUSH_ON_TASK_SWITCH,
 TLB_REMOTE_SHOOTDOWN,
 TLB_LOCAL_SHOOTDOWN,
 TLB_LOCAL_MM_SHOOTDOWN,
 NR_TLB_FLUSH_REASONS,
};

struct vdso_image {
 void *data;
 unsigned long size;


 struct vm_special_mapping text_mapping;

 unsigned long alt, alt_len;

 long sym_vvar_start;

 long sym_vvar_page;
 long sym_hpet_page;
 long sym_VDSO32_NOTE_MASK;
 long sym___kernel_sigreturn;
 long sym___kernel_rt_sigreturn;
 long sym___kernel_vsyscall;
 long sym_VDSO32_SYSENTER_RETURN;
};


extern const struct vdso_image vdso_image_64;







extern const struct vdso_image vdso_image_32_int80;

extern const struct vdso_image vdso_image_32_syscall;

extern const struct vdso_image vdso_image_32_sysenter;

extern const struct vdso_image *selected_vdso32;


extern void __attribute__ ((__section__(".init.text"))) __attribute__((__cold__)) __attribute__((no_instrument_function)) init_vdso_image(const struct vdso_image *image);


extern unsigned int vdso64_enabled;


extern unsigned int vdso32_enabled;
static inline __attribute__((no_instrument_function)) void elf_common_init(struct thread_struct *t,
       struct pt_regs *regs, const u16 ds)
{
 regs->ax = regs->bx = regs->cx = regs->dx = 0;
 regs->si = regs->di = regs->bp = 0;
 regs->r8 = regs->r9 = regs->r10 = regs->r11 = 0;
 regs->r12 = regs->r13 = regs->r14 = regs->r15 = 0;
 t->fs = t->gs = 0;
 t->fsindex = t->gsindex = 0;
 t->ds = t->es = ds;
}







void start_thread_ia32(struct pt_regs *regs, u32 new_ip, u32 new_sp);


void set_personality_ia32(bool);
extern void set_personality_64bit(void);
extern unsigned int sysctl_vsyscall32;
extern int force_personality32;
struct task_struct;
struct linux_binprm;


extern int arch_setup_additional_pages(struct linux_binprm *bprm,
           int uses_interp);
extern int compat_arch_setup_additional_pages(struct linux_binprm *bprm,
           int uses_interp);


extern unsigned long arch_randomize_brk(struct mm_struct *mm);





static inline __attribute__((no_instrument_function)) int mmap_is_ia32(void)
{




 if (test_ti_thread_flag(current_thread_info(), 29))
  return 1;

 return 0;
}


enum align_flags {
 ALIGN_VA_32 = (1UL << (0)),
 ALIGN_VA_64 = (1UL << (1)),
};

struct va_alignment {
 int flags;
 unsigned long mask;
} __attribute__((__aligned__((1 << (6)))));

extern struct va_alignment va_align;
extern unsigned long align_vdso_addr(unsigned long);






typedef __u32 Elf32_Addr;
typedef __u16 Elf32_Half;
typedef __u32 Elf32_Off;
typedef __s32 Elf32_Sword;
typedef __u32 Elf32_Word;


typedef __u64 Elf64_Addr;
typedef __u16 Elf64_Half;
typedef __s16 Elf64_SHalf;
typedef __u64 Elf64_Off;
typedef __s32 Elf64_Sword;
typedef __u32 Elf64_Word;
typedef __u64 Elf64_Xword;
typedef __s64 Elf64_Sxword;
typedef struct dynamic{
  Elf32_Sword d_tag;
  union{
    Elf32_Sword d_val;
    Elf32_Addr d_ptr;
  } d_un;
} Elf32_Dyn;

typedef struct {
  Elf64_Sxword d_tag;
  union {
    Elf64_Xword d_val;
    Elf64_Addr d_ptr;
  } d_un;
} Elf64_Dyn;
typedef struct elf32_rel {
  Elf32_Addr r_offset;
  Elf32_Word r_info;
} Elf32_Rel;

typedef struct elf64_rel {
  Elf64_Addr r_offset;
  Elf64_Xword r_info;
} Elf64_Rel;

typedef struct elf32_rela{
  Elf32_Addr r_offset;
  Elf32_Word r_info;
  Elf32_Sword r_addend;
} Elf32_Rela;

typedef struct elf64_rela {
  Elf64_Addr r_offset;
  Elf64_Xword r_info;
  Elf64_Sxword r_addend;
} Elf64_Rela;

typedef struct elf32_sym{
  Elf32_Word st_name;
  Elf32_Addr st_value;
  Elf32_Word st_size;
  unsigned char st_info;
  unsigned char st_other;
  Elf32_Half st_shndx;
} Elf32_Sym;

typedef struct elf64_sym {
  Elf64_Word st_name;
  unsigned char st_info;
  unsigned char st_other;
  Elf64_Half st_shndx;
  Elf64_Addr st_value;
  Elf64_Xword st_size;
} Elf64_Sym;




typedef struct elf32_hdr{
  unsigned char e_ident[16];
  Elf32_Half e_type;
  Elf32_Half e_machine;
  Elf32_Word e_version;
  Elf32_Addr e_entry;
  Elf32_Off e_phoff;
  Elf32_Off e_shoff;
  Elf32_Word e_flags;
  Elf32_Half e_ehsize;
  Elf32_Half e_phentsize;
  Elf32_Half e_phnum;
  Elf32_Half e_shentsize;
  Elf32_Half e_shnum;
  Elf32_Half e_shstrndx;
} Elf32_Ehdr;

typedef struct elf64_hdr {
  unsigned char e_ident[16];
  Elf64_Half e_type;
  Elf64_Half e_machine;
  Elf64_Word e_version;
  Elf64_Addr e_entry;
  Elf64_Off e_phoff;
  Elf64_Off e_shoff;
  Elf64_Word e_flags;
  Elf64_Half e_ehsize;
  Elf64_Half e_phentsize;
  Elf64_Half e_phnum;
  Elf64_Half e_shentsize;
  Elf64_Half e_shnum;
  Elf64_Half e_shstrndx;
} Elf64_Ehdr;







typedef struct elf32_phdr{
  Elf32_Word p_type;
  Elf32_Off p_offset;
  Elf32_Addr p_vaddr;
  Elf32_Addr p_paddr;
  Elf32_Word p_filesz;
  Elf32_Word p_memsz;
  Elf32_Word p_flags;
  Elf32_Word p_align;
} Elf32_Phdr;

typedef struct elf64_phdr {
  Elf64_Word p_type;
  Elf64_Word p_flags;
  Elf64_Off p_offset;
  Elf64_Addr p_vaddr;
  Elf64_Addr p_paddr;
  Elf64_Xword p_filesz;
  Elf64_Xword p_memsz;
  Elf64_Xword p_align;
} Elf64_Phdr;
typedef struct elf32_shdr {
  Elf32_Word sh_name;
  Elf32_Word sh_type;
  Elf32_Word sh_flags;
  Elf32_Addr sh_addr;
  Elf32_Off sh_offset;
  Elf32_Word sh_size;
  Elf32_Word sh_link;
  Elf32_Word sh_info;
  Elf32_Word sh_addralign;
  Elf32_Word sh_entsize;
} Elf32_Shdr;

typedef struct elf64_shdr {
  Elf64_Word sh_name;
  Elf64_Word sh_type;
  Elf64_Xword sh_flags;
  Elf64_Addr sh_addr;
  Elf64_Off sh_offset;
  Elf64_Xword sh_size;
  Elf64_Word sh_link;
  Elf64_Word sh_info;
  Elf64_Xword sh_addralign;
  Elf64_Xword sh_entsize;
} Elf64_Shdr;
typedef struct elf32_note {
  Elf32_Word n_namesz;
  Elf32_Word n_descsz;
  Elf32_Word n_type;
} Elf32_Nhdr;


typedef struct elf64_note {
  Elf64_Word n_namesz;
  Elf64_Word n_descsz;
  Elf64_Word n_type;
} Elf64_Nhdr;
extern Elf64_Dyn _DYNAMIC [];
struct file;
struct coredump_params;


static inline __attribute__((no_instrument_function)) int elf_coredump_extra_notes_size(void) { return 0; }
static inline __attribute__((no_instrument_function)) int elf_coredump_extra_notes_write(struct coredump_params *cprm) { return 0; }

struct idr_layer {
 int prefix;
 int layer;
 struct idr_layer *ary[1<<8];
 int count;
 union {

  unsigned long bitmap[((((1 << 8)) + (8 * sizeof(long)) - 1) / (8 * sizeof(long)))];
  struct callback_head callback_head;
 };
};

struct idr {
 struct idr_layer *hint;
 struct idr_layer *top;
 int layers;
 int cur;
 spinlock_t lock;
 int id_free_cnt;
 struct idr_layer *id_free;
};
void *idr_find_slowpath(struct idr *idp, int id);
void idr_preload(gfp_t gfp_mask);
int idr_alloc(struct idr *idp, void *ptr, int start, int end, gfp_t gfp_mask);
int idr_alloc_cyclic(struct idr *idr, void *ptr, int start, int end, gfp_t gfp_mask);
int idr_for_each(struct idr *idp,
   int (*fn)(int id, void *p, void *data), void *data);
void *idr_get_next(struct idr *idp, int *nextid);
void *idr_replace(struct idr *idp, void *ptr, int id);
void idr_remove(struct idr *idp, int id);
void idr_destroy(struct idr *idp);
void idr_init(struct idr *idp);
bool idr_is_empty(struct idr *idp);







static inline __attribute__((no_instrument_function)) void idr_preload_end(void)
{
 __asm__ __volatile__("": : :"memory");
}
static inline __attribute__((no_instrument_function)) void *idr_find(struct idr *idr, int id)
{
 struct idr_layer *hint = ({ typeof(*(idr->hint)) *_________p1 = (typeof(*(idr->hint)) *)(*(volatile typeof((idr->hint)) *)&((idr->hint))); do { } while (0); ; do { } while (0); ((typeof(*(idr->hint)) *)(_________p1)); });

 if (hint && (id & ~((1 << 8)-1)) == hint->prefix)
  return ({ typeof(*(hint->ary[id & ((1 << 8)-1)])) *_________p1 = (typeof(*(hint->ary[id & ((1 << 8)-1)])) *)(*(volatile typeof((hint->ary[id & ((1 << 8)-1)])) *)&((hint->ary[id & ((1 << 8)-1)]))); do { } while (0); ; do { } while (0); ((typeof(*(hint->ary[id & ((1 << 8)-1)])) *)(_________p1)); });

 return idr_find_slowpath(idr, id);
}
struct ida_bitmap {
 long nr_busy;
 unsigned long bitmap[(128 / sizeof(long) - 1)];
};

struct ida {
 struct idr idr;
 struct ida_bitmap *free_bitmap;
};




int ida_pre_get(struct ida *ida, gfp_t gfp_mask);
int ida_get_new_above(struct ida *ida, int starting_id, int *p_id);
void ida_remove(struct ida *ida, int id);
void ida_destroy(struct ida *ida);
void ida_init(struct ida *ida);

int ida_simple_get(struct ida *ida, unsigned int start, unsigned int end,
     gfp_t gfp_mask);
void ida_simple_remove(struct ida *ida, unsigned int id);
static inline __attribute__((no_instrument_function)) int ida_get_new(struct ida *ida, int *p_id)
{
 return ida_get_new_above(ida, 0, p_id);
}

void __attribute__ ((__section__(".init.text"))) __attribute__((__cold__)) __attribute__((no_instrument_function)) idr_init_cache(void);





struct file;
struct dentry;
struct iattr;
struct seq_file;
struct vm_area_struct;
struct super_block;
struct file_system_type;

struct kernfs_open_node;
struct kernfs_iattrs;

enum kernfs_node_type {
 KERNFS_DIR = 0x0001,
 KERNFS_FILE = 0x0002,
 KERNFS_LINK = 0x0004,
};




enum kernfs_node_flag {
 KERNFS_ACTIVATED = 0x0010,
 KERNFS_NS = 0x0020,
 KERNFS_HAS_SEQ_SHOW = 0x0040,
 KERNFS_HAS_MMAP = 0x0080,
 KERNFS_LOCKDEP = 0x0100,
 KERNFS_STATIC_NAME = 0x0200,
 KERNFS_SUICIDAL = 0x0400,
 KERNFS_SUICIDED = 0x0800,
};


enum kernfs_root_flag {






 KERNFS_ROOT_CREATE_DEACTIVATED = 0x0001,
 KERNFS_ROOT_EXTRA_OPEN_PERM_CHECK = 0x0002,
};


struct kernfs_elem_dir {
 unsigned long subdirs;

 struct rb_root children;





 struct kernfs_root *root;
};

struct kernfs_elem_symlink {
 struct kernfs_node *target_kn;
};

struct kernfs_elem_attr {
 const struct kernfs_ops *ops;
 struct kernfs_open_node *open;
 loff_t size;
 struct kernfs_node *notify_next;
};
struct kernfs_node {
 atomic_t count;
 atomic_t active;
 struct kernfs_node *parent;
 const char *name;

 struct rb_node rb;

 const void *ns;
 unsigned int hash;
 union {
  struct kernfs_elem_dir dir;
  struct kernfs_elem_symlink symlink;
  struct kernfs_elem_attr attr;
 };

 void *priv;

 unsigned short flags;
 umode_t mode;
 unsigned int ino;
 struct kernfs_iattrs *iattr;
};
struct kernfs_syscall_ops {
 int (*remount_fs)(struct kernfs_root *root, int *flags, char *data);
 int (*show_options)(struct seq_file *sf, struct kernfs_root *root);

 int (*mkdir)(struct kernfs_node *parent, const char *name,
       umode_t mode);
 int (*rmdir)(struct kernfs_node *kn);
 int (*rename)(struct kernfs_node *kn, struct kernfs_node *new_parent,
        const char *new_name);
};

struct kernfs_root {

 struct kernfs_node *kn;
 unsigned int flags;


 struct ida ino_ida;
 struct kernfs_syscall_ops *syscall_ops;


 struct list_head supers;

 wait_queue_head_t deactivate_waitq;
};

struct kernfs_open_file {

 struct kernfs_node *kn;
 struct file *file;
 void *priv;


 struct mutex mutex;
 int event;
 struct list_head list;

 size_t atomic_write_len;
 bool mmapped;
 const struct vm_operations_struct *vm_ops;
};

struct kernfs_ops {
 int (*seq_show)(struct seq_file *sf, void *v);

 void *(*seq_start)(struct seq_file *sf, loff_t *ppos);
 void *(*seq_next)(struct seq_file *sf, void *v, loff_t *ppos);
 void (*seq_stop)(struct seq_file *sf, void *v);

 ssize_t (*read)(struct kernfs_open_file *of, char *buf, size_t bytes,
   loff_t off);
 size_t atomic_write_len;
 ssize_t (*write)(struct kernfs_open_file *of, char *buf, size_t bytes,
    loff_t off);

 int (*mmap)(struct kernfs_open_file *of, struct vm_area_struct *vma);




};



static inline __attribute__((no_instrument_function)) enum kernfs_node_type kernfs_type(struct kernfs_node *kn)
{
 return kn->flags & 0x000f;
}
static inline __attribute__((no_instrument_function)) void kernfs_enable_ns(struct kernfs_node *kn)
{
 ({ static bool __attribute__ ((__section__(".data.unlikely"))) __warned; int __ret_warn_once = !!(kernfs_type(kn) != KERNFS_DIR); if (__builtin_expect(!!(__ret_warn_once), 0)) if (({ int __ret_warn_on = !!(!__warned); if (__builtin_expect(!!(__ret_warn_on), 0)) warn_slowpath_null("include/linux/kernfs.h", 244); __builtin_expect(!!(__ret_warn_on), 0); })) __warned = true; __builtin_expect(!!(__ret_warn_once), 0); });
 ({ static bool __attribute__ ((__section__(".data.unlikely"))) __warned; int __ret_warn_once = !!(!((&kn->dir.children)->rb_node == ((void *)0))); if (__builtin_expect(!!(__ret_warn_once), 0)) if (({ int __ret_warn_on = !!(!__warned); if (__builtin_expect(!!(__ret_warn_on), 0)) warn_slowpath_null("include/linux/kernfs.h", 245); __builtin_expect(!!(__ret_warn_on), 0); })) __warned = true; __builtin_expect(!!(__ret_warn_once), 0); });
 kn->flags |= KERNFS_NS;
}







static inline __attribute__((no_instrument_function)) bool kernfs_ns_enabled(struct kernfs_node *kn)
{
 return kn->flags & KERNFS_NS;
}

int kernfs_name(struct kernfs_node *kn, char *buf, size_t buflen);
char * kernfs_path(struct kernfs_node *kn, char *buf,
    size_t buflen);
void pr_cont_kernfs_name(struct kernfs_node *kn);
void pr_cont_kernfs_path(struct kernfs_node *kn);
struct kernfs_node *kernfs_get_parent(struct kernfs_node *kn);
struct kernfs_node *kernfs_find_and_get_ns(struct kernfs_node *parent,
        const char *name, const void *ns);
void kernfs_get(struct kernfs_node *kn);
void kernfs_put(struct kernfs_node *kn);

struct kernfs_node *kernfs_node_from_dentry(struct dentry *dentry);
struct kernfs_root *kernfs_root_from_sb(struct super_block *sb);

struct kernfs_root *kernfs_create_root(struct kernfs_syscall_ops *scops,
           unsigned int flags, void *priv);
void kernfs_destroy_root(struct kernfs_root *root);

struct kernfs_node *kernfs_create_dir_ns(struct kernfs_node *parent,
      const char *name, umode_t mode,
      void *priv, const void *ns);
struct kernfs_node *__kernfs_create_file(struct kernfs_node *parent,
      const char *name,
      umode_t mode, loff_t size,
      const struct kernfs_ops *ops,
      void *priv, const void *ns,
      bool name_is_static,
      struct lock_class_key *key);
struct kernfs_node *kernfs_create_link(struct kernfs_node *parent,
           const char *name,
           struct kernfs_node *target);
void kernfs_activate(struct kernfs_node *kn);
void kernfs_remove(struct kernfs_node *kn);
void kernfs_break_active_protection(struct kernfs_node *kn);
void kernfs_unbreak_active_protection(struct kernfs_node *kn);
bool kernfs_remove_self(struct kernfs_node *kn);
int kernfs_remove_by_name_ns(struct kernfs_node *parent, const char *name,
        const void *ns);
int kernfs_rename_ns(struct kernfs_node *kn, struct kernfs_node *new_parent,
       const char *new_name, const void *new_ns);
int kernfs_setattr(struct kernfs_node *kn, const struct iattr *iattr);
void kernfs_notify(struct kernfs_node *kn);

const void *kernfs_super_ns(struct super_block *sb);
struct dentry *kernfs_mount_ns(struct file_system_type *fs_type, int flags,
          struct kernfs_root *root, unsigned long magic,
          bool *new_sb_created, const void *ns);
void kernfs_kill_sb(struct super_block *sb);
struct super_block *kernfs_pin_sb(struct kernfs_root *root, const void *ns);

void kernfs_init(void);
static inline __attribute__((no_instrument_function)) struct kernfs_node *
kernfs_find_and_get(struct kernfs_node *kn, const char *name)
{
 return kernfs_find_and_get_ns(kn, name, ((void *)0));
}

static inline __attribute__((no_instrument_function)) struct kernfs_node *
kernfs_create_dir(struct kernfs_node *parent, const char *name, umode_t mode,
    void *priv)
{
 return kernfs_create_dir_ns(parent, name, mode, priv, ((void *)0));
}

static inline __attribute__((no_instrument_function)) struct kernfs_node *
kernfs_create_file_ns(struct kernfs_node *parent, const char *name,
        umode_t mode, loff_t size, const struct kernfs_ops *ops,
        void *priv, const void *ns)
{
 struct lock_class_key *key = ((void *)0);




 return __kernfs_create_file(parent, name, mode, size, ops, priv, ns,
        false, key);
}

static inline __attribute__((no_instrument_function)) struct kernfs_node *
kernfs_create_file(struct kernfs_node *parent, const char *name, umode_t mode,
     loff_t size, const struct kernfs_ops *ops, void *priv)
{
 return kernfs_create_file_ns(parent, name, mode, size, ops, priv, ((void *)0));
}

static inline __attribute__((no_instrument_function)) int kernfs_remove_by_name(struct kernfs_node *parent,
     const char *name)
{
 return kernfs_remove_by_name_ns(parent, name, ((void *)0));
}

static inline __attribute__((no_instrument_function)) int kernfs_rename(struct kernfs_node *kn,
    struct kernfs_node *new_parent,
    const char *new_name)
{
 return kernfs_rename_ns(kn, new_parent, new_name, ((void *)0));
}

static inline __attribute__((no_instrument_function)) struct dentry *
kernfs_mount(struct file_system_type *fs_type, int flags,
  struct kernfs_root *root, unsigned long magic,
  bool *new_sb_created)
{
 return kernfs_mount_ns(fs_type, flags, root,
    magic, new_sb_created, ((void *)0));
}




struct sock;
struct kobject;





enum kobj_ns_type {
 KOBJ_NS_TYPE_NONE = 0,
 KOBJ_NS_TYPE_NET,
 KOBJ_NS_TYPES
};
struct kobj_ns_type_operations {
 enum kobj_ns_type type;
 bool (*current_may_mount)(void);
 void *(*grab_current_ns)(void);
 const void *(*netlink_ns)(struct sock *sk);
 const void *(*initial_ns)(void);
 void (*drop_ns)(void *);
};

int kobj_ns_type_register(const struct kobj_ns_type_operations *ops);
int kobj_ns_type_registered(enum kobj_ns_type type);
const struct kobj_ns_type_operations *kobj_child_ns_ops(struct kobject *parent);
const struct kobj_ns_type_operations *kobj_ns_ops(struct kobject *kobj);

bool kobj_ns_current_may_mount(enum kobj_ns_type type);
void *kobj_ns_grab_current(enum kobj_ns_type type);
const void *kobj_ns_netlink(enum kobj_ns_type type, struct sock *sk);
const void *kobj_ns_initial(enum kobj_ns_type type);
void kobj_ns_drop(enum kobj_ns_type type, void *ns);



struct kobject;
struct module;
struct bin_attribute;
enum kobj_ns_type;

struct attribute {
 const char *name;
 umode_t mode;





};
struct attribute_group {
 const char *name;
 umode_t (*is_visible)(struct kobject *,
           struct attribute *, int);
 struct attribute **attrs;
 struct bin_attribute **bin_attrs;
};
struct file;
struct vm_area_struct;

struct bin_attribute {
 struct attribute attr;
 size_t size;
 void *private;
 ssize_t (*read)(struct file *, struct kobject *, struct bin_attribute *,
   char *, loff_t, size_t);
 ssize_t (*write)(struct file *, struct kobject *, struct bin_attribute *,
    char *, loff_t, size_t);
 int (*mmap)(struct file *, struct kobject *, struct bin_attribute *attr,
      struct vm_area_struct *vma);
};
struct sysfs_ops {
 ssize_t (*show)(struct kobject *, struct attribute *, char *);
 ssize_t (*store)(struct kobject *, struct attribute *, const char *, size_t);
};



int sysfs_create_dir_ns(struct kobject *kobj, const void *ns);
void sysfs_remove_dir(struct kobject *kobj);
int sysfs_rename_dir_ns(struct kobject *kobj, const char *new_name,
         const void *new_ns);
int sysfs_move_dir_ns(struct kobject *kobj,
       struct kobject *new_parent_kobj,
       const void *new_ns);

int sysfs_create_file_ns(struct kobject *kobj,
          const struct attribute *attr,
          const void *ns);
int sysfs_create_files(struct kobject *kobj,
       const struct attribute **attr);
int sysfs_chmod_file(struct kobject *kobj,
      const struct attribute *attr, umode_t mode);
void sysfs_remove_file_ns(struct kobject *kobj, const struct attribute *attr,
     const void *ns);
bool sysfs_remove_file_self(struct kobject *kobj, const struct attribute *attr);
void sysfs_remove_files(struct kobject *kobj, const struct attribute **attr);

int sysfs_create_bin_file(struct kobject *kobj,
           const struct bin_attribute *attr);
void sysfs_remove_bin_file(struct kobject *kobj,
      const struct bin_attribute *attr);

int sysfs_create_link(struct kobject *kobj, struct kobject *target,
       const char *name);
int sysfs_create_link_nowarn(struct kobject *kobj,
       struct kobject *target,
       const char *name);
void sysfs_remove_link(struct kobject *kobj, const char *name);

int sysfs_rename_link_ns(struct kobject *kobj, struct kobject *target,
    const char *old_name, const char *new_name,
    const void *new_ns);

void sysfs_delete_link(struct kobject *dir, struct kobject *targ,
   const char *name);

int sysfs_create_group(struct kobject *kobj,
        const struct attribute_group *grp);
int sysfs_create_groups(struct kobject *kobj,
         const struct attribute_group **groups);
int sysfs_update_group(struct kobject *kobj,
         const struct attribute_group *grp);
void sysfs_remove_group(struct kobject *kobj,
   const struct attribute_group *grp);
void sysfs_remove_groups(struct kobject *kobj,
    const struct attribute_group **groups);
int sysfs_add_file_to_group(struct kobject *kobj,
   const struct attribute *attr, const char *group);
void sysfs_remove_file_from_group(struct kobject *kobj,
   const struct attribute *attr, const char *group);
int sysfs_merge_group(struct kobject *kobj,
         const struct attribute_group *grp);
void sysfs_unmerge_group(struct kobject *kobj,
         const struct attribute_group *grp);
int sysfs_add_link_to_group(struct kobject *kobj, const char *group_name,
       struct kobject *target, const char *link_name);
void sysfs_remove_link_from_group(struct kobject *kobj, const char *group_name,
      const char *link_name);

void sysfs_notify(struct kobject *kobj, const char *dir, const char *attr);

int sysfs_init(void);

static inline __attribute__((no_instrument_function)) void sysfs_enable_ns(struct kernfs_node *kn)
{
 return kernfs_enable_ns(kn);
}
static inline __attribute__((no_instrument_function)) int sysfs_create_file(struct kobject *kobj,
       const struct attribute *attr)
{
 return sysfs_create_file_ns(kobj, attr, ((void *)0));
}

static inline __attribute__((no_instrument_function)) void sysfs_remove_file(struct kobject *kobj,
         const struct attribute *attr)
{
 sysfs_remove_file_ns(kobj, attr, ((void *)0));
}

static inline __attribute__((no_instrument_function)) int sysfs_rename_link(struct kobject *kobj, struct kobject *target,
        const char *old_name, const char *new_name)
{
 return sysfs_rename_link_ns(kobj, target, old_name, new_name, ((void *)0));
}

static inline __attribute__((no_instrument_function)) void sysfs_notify_dirent(struct kernfs_node *kn)
{
 kernfs_notify(kn);
}

static inline __attribute__((no_instrument_function)) struct kernfs_node *sysfs_get_dirent(struct kernfs_node *parent,
         const unsigned char *name)
{
 return kernfs_find_and_get(parent, name);
}

static inline __attribute__((no_instrument_function)) struct kernfs_node *sysfs_get(struct kernfs_node *kn)
{
 kernfs_get(kn);
 return kn;
}

static inline __attribute__((no_instrument_function)) void sysfs_put(struct kernfs_node *kn)
{
 kernfs_put(kn);
}


struct kref {
 atomic_t refcount;
};





static inline __attribute__((no_instrument_function)) void kref_init(struct kref *kref)
{
 atomic_set(&kref->refcount, 1);
}





static inline __attribute__((no_instrument_function)) void kref_get(struct kref *kref)
{




 ({ static bool __attribute__ ((__section__(".data.unlikely"))) __warned; int __ret_warn_once = !!((atomic_add_return(1, &kref->refcount)) < 2); if (__builtin_expect(!!(__ret_warn_once), 0)) if (({ int __ret_warn_on = !!(!__warned); if (__builtin_expect(!!(__ret_warn_on), 0)) warn_slowpath_null("include/linux/kref.h", 47); __builtin_expect(!!(__ret_warn_on), 0); })) __warned = true; __builtin_expect(!!(__ret_warn_once), 0); });
}
static inline __attribute__((no_instrument_function)) int kref_sub(struct kref *kref, unsigned int count,
      void (*release)(struct kref *kref))
{
 ({ int __ret_warn_on = !!(release == ((void *)0)); if (__builtin_expect(!!(__ret_warn_on), 0)) warn_slowpath_null("include/linux/kref.h", 71); __builtin_expect(!!(__ret_warn_on), 0); });

 if (atomic_sub_and_test((int) count, &kref->refcount)) {
  release(kref);
  return 1;
 }
 return 0;
}
static inline __attribute__((no_instrument_function)) int kref_put(struct kref *kref, void (*release)(struct kref *kref))
{
 return kref_sub(kref, 1, release);
}
static inline __attribute__((no_instrument_function)) int kref_put_spinlock_irqsave(struct kref *kref,
  void (*release)(struct kref *kref),
  spinlock_t *lock)
{
 unsigned long flags;

 ({ int __ret_warn_on = !!(release == ((void *)0)); if (__builtin_expect(!!(__ret_warn_on), 0)) warn_slowpath_null("include/linux/kref.h", 121); __builtin_expect(!!(__ret_warn_on), 0); });
 if (atomic_add_unless(&kref->refcount, -1, 1))
  return 0;
 do { do { ({ unsigned long __dummy; typeof(flags) __dummy2; (void)(&__dummy == &__dummy2); 1; }); flags = _raw_spin_lock_irqsave(spinlock_check(lock)); } while (0); } while (0);
 if (atomic_dec_and_test(&kref->refcount)) {
  release(kref);
  do { if (({ ({ unsigned long __dummy; typeof(flags) __dummy2; (void)(&__dummy == &__dummy2); 1; }); arch_irqs_disabled_flags(flags); })) { do { ({ unsigned long __dummy; typeof(flags) __dummy2; (void)(&__dummy == &__dummy2); 1; }); arch_local_irq_restore(flags); } while (0); do { } while (0); } else { do { } while (0); do { ({ unsigned long __dummy; typeof(flags) __dummy2; (void)(&__dummy == &__dummy2); 1; }); arch_local_irq_restore(flags); } while (0); } } while (0);
  return 1;
 }
 spin_unlock_irqrestore(lock, flags);
 return 0;
}

static inline __attribute__((no_instrument_function)) int kref_put_mutex(struct kref *kref,
     void (*release)(struct kref *kref),
     struct mutex *lock)
{
 ({ int __ret_warn_on = !!(release == ((void *)0)); if (__builtin_expect(!!(__ret_warn_on), 0)) warn_slowpath_null("include/linux/kref.h", 138); __builtin_expect(!!(__ret_warn_on), 0); });
 if (__builtin_expect(!!(!atomic_add_unless(&kref->refcount, -1, 1)), 0)) {
  mutex_lock(lock);
  if (__builtin_expect(!!(!atomic_dec_and_test(&kref->refcount)), 0)) {
   mutex_unlock(lock);
   return 0;
  }
  release(kref);
  return 1;
 }
 return 0;
}
static inline __attribute__((no_instrument_function)) int kref_get_unless_zero(struct kref *kref)
{
 return atomic_add_unless(&kref->refcount, 1, 0);
}
extern char uevent_helper[];



extern u64 uevent_seqnum;
enum kobject_action {
 KOBJ_ADD,
 KOBJ_REMOVE,
 KOBJ_CHANGE,
 KOBJ_MOVE,
 KOBJ_ONLINE,
 KOBJ_OFFLINE,
 KOBJ_MAX
};

struct kobject {
 const char *name;
 struct list_head entry;
 struct kobject *parent;
 struct kset *kset;
 struct kobj_type *ktype;
 struct kernfs_node *sd;
 struct kref kref;



 unsigned int state_initialized:1;
 unsigned int state_in_sysfs:1;
 unsigned int state_add_uevent_sent:1;
 unsigned int state_remove_uevent_sent:1;
 unsigned int uevent_suppress:1;
};

extern __attribute__((format(printf, 2, 3)))
int kobject_set_name(struct kobject *kobj, const char *name, ...);
extern int kobject_set_name_vargs(struct kobject *kobj, const char *fmt,
      va_list vargs);

static inline __attribute__((no_instrument_function)) const char *kobject_name(const struct kobject *kobj)
{
 return kobj->name;
}

extern void kobject_init(struct kobject *kobj, struct kobj_type *ktype);
extern __attribute__((format(printf, 3, 4)))
int kobject_add(struct kobject *kobj, struct kobject *parent,
  const char *fmt, ...);
extern __attribute__((format(printf, 4, 5)))
int kobject_init_and_add(struct kobject *kobj,
    struct kobj_type *ktype, struct kobject *parent,
    const char *fmt, ...);

extern void kobject_del(struct kobject *kobj);

extern struct kobject * kobject_create(void);
extern struct kobject * kobject_create_and_add(const char *name,
      struct kobject *parent);

extern int kobject_rename(struct kobject *, const char *new_name);
extern int kobject_move(struct kobject *, struct kobject *);

extern struct kobject *kobject_get(struct kobject *kobj);
extern void kobject_put(struct kobject *kobj);

extern const void *kobject_namespace(struct kobject *kobj);
extern char *kobject_get_path(struct kobject *kobj, gfp_t flag);

struct kobj_type {
 void (*release)(struct kobject *kobj);
 const struct sysfs_ops *sysfs_ops;
 struct attribute **default_attrs;
 const struct kobj_ns_type_operations *(*child_ns_type)(struct kobject *kobj);
 const void *(*namespace)(struct kobject *kobj);
};

struct kobj_uevent_env {
 char *argv[3];
 char *envp[32];
 int envp_idx;
 char buf[2048];
 int buflen;
};

struct kset_uevent_ops {
 int (* const filter)(struct kset *kset, struct kobject *kobj);
 const char *(* const name)(struct kset *kset, struct kobject *kobj);
 int (* const uevent)(struct kset *kset, struct kobject *kobj,
        struct kobj_uevent_env *env);
};

struct kobj_attribute {
 struct attribute attr;
 ssize_t (*show)(struct kobject *kobj, struct kobj_attribute *attr,
   char *buf);
 ssize_t (*store)(struct kobject *kobj, struct kobj_attribute *attr,
    const char *buf, size_t count);
};

extern const struct sysfs_ops kobj_sysfs_ops;

struct sock;
struct kset {
 struct list_head list;
 spinlock_t list_lock;
 struct kobject kobj;
 const struct kset_uevent_ops *uevent_ops;
};

extern void kset_init(struct kset *kset);
extern int kset_register(struct kset *kset);
extern void kset_unregister(struct kset *kset);
extern struct kset * kset_create_and_add(const char *name,
      const struct kset_uevent_ops *u,
      struct kobject *parent_kobj);

static inline __attribute__((no_instrument_function)) struct kset *to_kset(struct kobject *kobj)
{
 return kobj ? ({ const typeof( ((struct kset *)0)->kobj ) *__mptr = (kobj); (struct kset *)( (char *)__mptr - __builtin_offsetof(struct kset,kobj) );}) : ((void *)0);
}

static inline __attribute__((no_instrument_function)) struct kset *kset_get(struct kset *k)
{
 return k ? to_kset(kobject_get(&k->kobj)) : ((void *)0);
}

static inline __attribute__((no_instrument_function)) void kset_put(struct kset *k)
{
 kobject_put(&k->kobj);
}

static inline __attribute__((no_instrument_function)) struct kobj_type *get_ktype(struct kobject *kobj)
{
 return kobj->ktype;
}

extern struct kobject *kset_find_obj(struct kset *, const char *);


extern struct kobject *kernel_kobj;

extern struct kobject *mm_kobj;

extern struct kobject *hypervisor_kobj;

extern struct kobject *power_kobj;

extern struct kobject *firmware_kobj;

int kobject_uevent(struct kobject *kobj, enum kobject_action action);
int kobject_uevent_env(struct kobject *kobj, enum kobject_action action,
   char *envp[]);

__attribute__((format(printf, 2, 3)))
int add_uevent_var(struct kobj_uevent_env *env, const char *format, ...);

int kobject_action_type(const char *buf, size_t count,
   enum kobject_action *type);
struct kernel_param;






enum {
 KERNEL_PARAM_FL_NOARG = (1 << 0)
};

struct kernel_param_ops {

 unsigned int flags;

 int (*set)(const char *val, const struct kernel_param *kp);

 int (*get)(char *buffer, const struct kernel_param *kp);

 void (*free)(void *arg);
};

struct kernel_param {
 const char *name;
 const struct kernel_param_ops *ops;
 u16 perm;
 s16 level;
 union {
  void *arg;
  const struct kparam_string *str;
  const struct kparam_array *arr;
 };
};


struct kparam_string {
 unsigned int maxlen;
 char *string;
};


struct kparam_array
{
 unsigned int max;
 unsigned int elemsize;
 unsigned int *num;
 const struct kernel_param_ops *ops;
 void *elem;
};
static inline __attribute__((no_instrument_function)) int
__check_old_set_param(int (*oldset)(const char *, struct kernel_param *))
{
 return 0;
}
extern void __kernel_param_lock(void);
extern void __kernel_param_unlock(void);
extern bool parameq(const char *name1, const char *name2);
extern bool parameqn(const char *name1, const char *name2, size_t n);


extern char *parse_args(const char *name,
        char *args,
        const struct kernel_param *params,
        unsigned num,
        s16 level_min,
        s16 level_max,
        int (*unknown)(char *param, char *val,
         const char *doing));



extern void destroy_params(const struct kernel_param *params, unsigned num);
extern struct kernel_param_ops param_ops_byte;
extern int param_set_byte(const char *val, const struct kernel_param *kp);
extern int param_get_byte(char *buffer, const struct kernel_param *kp);


extern struct kernel_param_ops param_ops_short;
extern int param_set_short(const char *val, const struct kernel_param *kp);
extern int param_get_short(char *buffer, const struct kernel_param *kp);


extern struct kernel_param_ops param_ops_ushort;
extern int param_set_ushort(const char *val, const struct kernel_param *kp);
extern int param_get_ushort(char *buffer, const struct kernel_param *kp);


extern struct kernel_param_ops param_ops_int;
extern int param_set_int(const char *val, const struct kernel_param *kp);
extern int param_get_int(char *buffer, const struct kernel_param *kp);


extern struct kernel_param_ops param_ops_uint;
extern int param_set_uint(const char *val, const struct kernel_param *kp);
extern int param_get_uint(char *buffer, const struct kernel_param *kp);


extern struct kernel_param_ops param_ops_long;
extern int param_set_long(const char *val, const struct kernel_param *kp);
extern int param_get_long(char *buffer, const struct kernel_param *kp);


extern struct kernel_param_ops param_ops_ulong;
extern int param_set_ulong(const char *val, const struct kernel_param *kp);
extern int param_get_ulong(char *buffer, const struct kernel_param *kp);


extern struct kernel_param_ops param_ops_ullong;
extern int param_set_ullong(const char *val, const struct kernel_param *kp);
extern int param_get_ullong(char *buffer, const struct kernel_param *kp);


extern struct kernel_param_ops param_ops_charp;
extern int param_set_charp(const char *val, const struct kernel_param *kp);
extern int param_get_charp(char *buffer, const struct kernel_param *kp);



extern struct kernel_param_ops param_ops_bool;
extern int param_set_bool(const char *val, const struct kernel_param *kp);
extern int param_get_bool(char *buffer, const struct kernel_param *kp);


extern struct kernel_param_ops param_ops_invbool;
extern int param_set_invbool(const char *val, const struct kernel_param *kp);
extern int param_get_invbool(char *buffer, const struct kernel_param *kp);



extern struct kernel_param_ops param_ops_bint;
extern int param_set_bint(const char *val, const struct kernel_param *kp);
extern struct kernel_param_ops param_array_ops;

extern struct kernel_param_ops param_ops_string;
extern int param_set_copystring(const char *val, const struct kernel_param *);
extern int param_get_string(char *buffer, const struct kernel_param *kp);



struct module;


extern int module_param_sysfs_setup(struct module *mod,
        const struct kernel_param *kparam,
        unsigned int num_params);

extern void module_param_sysfs_remove(struct module *mod);







struct mod_arch_specific
{
};
struct modversion_info {
 unsigned long crc;
 char name[(64 - sizeof(unsigned long))];
};

struct module;

struct module_kobject {
 struct kobject kobj;
 struct module *mod;
 struct kobject *drivers_dir;
 struct module_param_attrs *mp;
 struct completion *kobj_completion;
};

struct module_attribute {
 struct attribute attr;
 ssize_t (*show)(struct module_attribute *, struct module_kobject *,
   char *);
 ssize_t (*store)(struct module_attribute *, struct module_kobject *,
    const char *, size_t count);
 void (*setup)(struct module *, const char *);
 int (*test)(struct module *);
 void (*free)(struct module *);
};

struct module_version_attribute {
 struct module_attribute mattr;
 const char *module_name;
 const char *version;
} __attribute__ ((__aligned__(sizeof(void *))));

extern ssize_t __modver_version_show(struct module_attribute *,
         struct module_kobject *, char *);

extern struct module_attribute module_uevent;


extern int init_module(void);
extern void cleanup_module(void);


struct exception_table_entry;

const struct exception_table_entry *
search_extable(const struct exception_table_entry *first,
        const struct exception_table_entry *last,
        unsigned long value);
void sort_extable(struct exception_table_entry *start,
    struct exception_table_entry *finish);
void sort_main_extable(void);
void trim_init_extable(struct module *m);
const struct exception_table_entry *search_exception_tables(unsigned long add);

struct notifier_block;



extern int modules_disabled;

void *__symbol_get(const char *symbol);
void *__symbol_get_gpl(const char *symbol);



struct module_use {
 struct list_head source_list;
 struct list_head target_list;
 struct module *source, *target;
};

enum module_state {
 MODULE_STATE_LIVE,
 MODULE_STATE_COMING,
 MODULE_STATE_GOING,
 MODULE_STATE_UNFORMED,
};
struct module_ref {
 unsigned long incs;
 unsigned long decs;
} __attribute((aligned(2 * sizeof(unsigned long))));

struct module {
 enum module_state state;


 struct list_head list;


 char name[(64 - sizeof(unsigned long))];


 struct module_kobject mkobj;
 struct module_attribute *modinfo_attrs;
 const char *version;
 const char *srcversion;
 struct kobject *holders_dir;


 const struct kernel_symbol *syms;
 const unsigned long *crcs;
 unsigned int num_syms;


 struct kernel_param *kp;
 unsigned int num_kp;


 unsigned int num_gpl_syms;
 const struct kernel_symbol *gpl_syms;
 const unsigned long *gpl_crcs;



 const struct kernel_symbol *unused_syms;
 const unsigned long *unused_crcs;
 unsigned int num_unused_syms;


 unsigned int num_unused_gpl_syms;
 const struct kernel_symbol *unused_gpl_syms;
 const unsigned long *unused_gpl_crcs;
 const struct kernel_symbol *gpl_future_syms;
 const unsigned long *gpl_future_crcs;
 unsigned int num_gpl_future_syms;


 unsigned int num_exentries;
 struct exception_table_entry *extable;


 int (*init)(void);


 void *module_init;


 void *module_core;


 unsigned int init_size, core_size;


 unsigned int init_text_size, core_text_size;


 unsigned int init_ro_size, core_ro_size;


 struct mod_arch_specific arch;

 unsigned int taints;



 unsigned num_bugs;
 struct list_head bug_list;
 struct bug_entry *bug_table;
 Elf64_Sym *symtab, *core_symtab;
 unsigned int num_symtab, core_num_syms;
 char *strtab, *core_strtab;


 struct module_sect_attrs *sect_attrs;


 struct module_notes_attrs *notes_attrs;




 char *args;



 void *percpu;
 unsigned int percpu_size;



 unsigned int num_tracepoints;
 struct tracepoint * const *tracepoints_ptrs;






 unsigned int num_trace_bprintk_fmt;
 const char **trace_bprintk_fmt_start;


 struct ftrace_event_call **trace_events;
 unsigned int num_trace_events;


 unsigned int num_ftrace_callsites;
 unsigned long *ftrace_callsites;




 struct list_head source_list;

 struct list_head target_list;


 void (*exit)(void);

 struct module_ref *refptr;







};




extern struct mutex module_mutex;




static inline __attribute__((no_instrument_function)) int module_is_live(struct module *mod)
{
 return mod->state != MODULE_STATE_GOING;
}

struct module *__module_text_address(unsigned long addr);
struct module *__module_address(unsigned long addr);
bool is_module_address(unsigned long addr);
bool is_module_percpu_address(unsigned long addr);
bool is_module_text_address(unsigned long addr);

static inline __attribute__((no_instrument_function)) bool within_module_core(unsigned long addr,
          const struct module *mod)
{
 return (unsigned long)mod->module_core <= addr &&
        addr < (unsigned long)mod->module_core + mod->core_size;
}

static inline __attribute__((no_instrument_function)) bool within_module_init(unsigned long addr,
          const struct module *mod)
{
 return (unsigned long)mod->module_init <= addr &&
        addr < (unsigned long)mod->module_init + mod->init_size;
}

static inline __attribute__((no_instrument_function)) bool within_module(unsigned long addr, const struct module *mod)
{
 return within_module_init(addr, mod) || within_module_core(addr, mod);
}


struct module *find_module(const char *name);

struct symsearch {
 const struct kernel_symbol *start, *stop;
 const unsigned long *crcs;
 enum {
  NOT_GPL_ONLY,
  GPL_ONLY,
  WILL_BE_GPL_ONLY,
 } licence;
 bool unused;
};


const struct kernel_symbol *find_symbol(const char *name,
     struct module **owner,
     const unsigned long **crc,
     bool gplok,
     bool warn);


bool each_symbol_section(bool (*fn)(const struct symsearch *arr,
        struct module *owner,
        void *data), void *data);



int module_get_kallsym(unsigned int symnum, unsigned long *value, char *type,
   char *name, char *module_name, int *exported);


unsigned long module_kallsyms_lookup_name(const char *name);

int module_kallsyms_on_each_symbol(int (*fn)(void *, const char *,
          struct module *, unsigned long),
       void *data);

extern void __module_put_and_exit(struct module *mod, long code)
 __attribute__((noreturn));



unsigned long module_refcount(struct module *mod);
void __symbol_put(const char *symbol);

void symbol_put_addr(void *addr);



extern void __module_get(struct module *module);



extern bool try_module_get(struct module *module);

extern void module_put(struct module *module);
int ref_module(struct module *a, struct module *b);
const char *module_address_lookup(unsigned long addr,
       unsigned long *symbolsize,
       unsigned long *offset,
       char **modname,
       char *namebuf);
int lookup_module_symbol_name(unsigned long addr, char *symname);
int lookup_module_symbol_attrs(unsigned long addr, unsigned long *size, unsigned long *offset, char *modname, char *name);


const struct exception_table_entry *search_module_extables(unsigned long addr);

int register_module_notifier(struct notifier_block *nb);
int unregister_module_notifier(struct notifier_block *nb);

extern void print_modules(void);
extern struct kset *module_kset;
extern struct kobj_type module_ktype;
extern int module_sysfs_initialized;
extern void set_all_modules_text_rw(void);
extern void set_all_modules_text_ro(void);






void module_bug_finalize(const Elf64_Ehdr *, const Elf64_Shdr *,
    struct module *);
void module_bug_cleanup(struct module *);
static inline __attribute__((no_instrument_function)) void kmemleak_init(void)
{
}
static inline __attribute__((no_instrument_function)) void kmemleak_alloc(const void *ptr, size_t size, int min_count,
      gfp_t gfp)
{
}
static inline __attribute__((no_instrument_function)) void kmemleak_alloc_recursive(const void *ptr, size_t size,
         int min_count, unsigned long flags,
         gfp_t gfp)
{
}
static inline __attribute__((no_instrument_function)) void kmemleak_alloc_percpu(const void *ptr, size_t size)
{
}
static inline __attribute__((no_instrument_function)) void kmemleak_free(const void *ptr)
{
}
static inline __attribute__((no_instrument_function)) void kmemleak_free_part(const void *ptr, size_t size)
{
}
static inline __attribute__((no_instrument_function)) void kmemleak_free_recursive(const void *ptr, unsigned long flags)
{
}
static inline __attribute__((no_instrument_function)) void kmemleak_free_percpu(const void *ptr)
{
}
static inline __attribute__((no_instrument_function)) void kmemleak_update_trace(const void *ptr)
{
}
static inline __attribute__((no_instrument_function)) void kmemleak_not_leak(const void *ptr)
{
}
static inline __attribute__((no_instrument_function)) void kmemleak_ignore(const void *ptr)
{
}
static inline __attribute__((no_instrument_function)) void kmemleak_scan_area(const void *ptr, size_t size, gfp_t gfp)
{
}
static inline __attribute__((no_instrument_function)) void kmemleak_erase(void **ptr)
{
}
static inline __attribute__((no_instrument_function)) void kmemleak_no_scan(const void *ptr)
{
}

struct mem_cgroup;



void __attribute__ ((__section__(".init.text"))) __attribute__((__cold__)) __attribute__((no_instrument_function)) kmem_cache_init(void);
int slab_is_available(void);

struct kmem_cache *kmem_cache_create(const char *, size_t, size_t,
   unsigned long,
   void (*)(void *));





void kmem_cache_destroy(struct kmem_cache *);
int kmem_cache_shrink(struct kmem_cache *);
void kmem_cache_free(struct kmem_cache *, void *);
void * __krealloc(const void *, size_t, gfp_t);
void * krealloc(const void *, size_t, gfp_t);
void kfree(const void *);
void kzfree(const void *);
size_t ksize(const void *);
extern struct kmem_cache *kmalloc_caches[(12 + 1) + 1];

extern struct kmem_cache *kmalloc_dma_caches[(12 + 1) + 1];
static inline __attribute__((no_instrument_function)) __attribute__((always_inline)) int kmalloc_index(size_t size)
{
 if (!size)
  return 0;

 if (size <= (1 << 3))
  return 3;

 if ((1 << 3) <= 32 && size > 64 && size <= 96)
  return 1;
 if ((1 << 3) <= 64 && size > 128 && size <= 192)
  return 2;
 if (size <= 8) return 3;
 if (size <= 16) return 4;
 if (size <= 32) return 5;
 if (size <= 64) return 6;
 if (size <= 128) return 7;
 if (size <= 256) return 8;
 if (size <= 512) return 9;
 if (size <= 1024) return 10;
 if (size <= 2 * 1024) return 11;
 if (size <= 4 * 1024) return 12;
 if (size <= 8 * 1024) return 13;
 if (size <= 16 * 1024) return 14;
 if (size <= 32 * 1024) return 15;
 if (size <= 64 * 1024) return 16;
 if (size <= 128 * 1024) return 17;
 if (size <= 256 * 1024) return 18;
 if (size <= 512 * 1024) return 19;
 if (size <= 1024 * 1024) return 20;
 if (size <= 2 * 1024 * 1024) return 21;
 if (size <= 4 * 1024 * 1024) return 22;
 if (size <= 8 * 1024 * 1024) return 23;
 if (size <= 16 * 1024 * 1024) return 24;
 if (size <= 32 * 1024 * 1024) return 25;
 if (size <= 64 * 1024 * 1024) return 26;
 do { asm volatile("1:\tud2\n" ".pushsection __bug_table,\"a\"\n" "2:\t.long 1b - 2b, %c0 - 2b\n" "\t.word %c1, 0\n" "\t.org 2b+%c2\n" ".popsection" : : "i" ("include/linux/slab.h"), "i" (308), "i" (sizeof(struct bug_entry))); __builtin_unreachable(); } while (0);


 return -1;
}


void *__kmalloc(size_t size, gfp_t flags);
void *kmem_cache_alloc(struct kmem_cache *, gfp_t flags);


void *__kmalloc_node(size_t size, gfp_t flags, int node);
void *kmem_cache_alloc_node(struct kmem_cache *, gfp_t flags, int node);
extern void *kmem_cache_alloc_trace(struct kmem_cache *, gfp_t, size_t);


extern void *kmem_cache_alloc_node_trace(struct kmem_cache *s,
        gfp_t gfpflags,
        int node, size_t size);
enum stat_item {
 ALLOC_FASTPATH,
 ALLOC_SLOWPATH,
 FREE_FASTPATH,
 FREE_SLOWPATH,
 FREE_FROZEN,
 FREE_ADD_PARTIAL,
 FREE_REMOVE_PARTIAL,
 ALLOC_FROM_PARTIAL,
 ALLOC_SLAB,
 ALLOC_REFILL,
 ALLOC_NODE_MISMATCH,
 FREE_SLAB,
 CPUSLAB_FLUSH,
 DEACTIVATE_FULL,
 DEACTIVATE_EMPTY,
 DEACTIVATE_TO_HEAD,
 DEACTIVATE_TO_TAIL,
 DEACTIVATE_REMOTE_FREES,
 DEACTIVATE_BYPASS,
 ORDER_FALLBACK,
 CMPXCHG_DOUBLE_CPU_FAIL,
 CMPXCHG_DOUBLE_FAIL,
 CPU_PARTIAL_ALLOC,
 CPU_PARTIAL_FREE,
 CPU_PARTIAL_NODE,
 CPU_PARTIAL_DRAIN,
 NR_SLUB_STAT_ITEMS };

struct kmem_cache_cpu {
 void **freelist;
 unsigned long tid;
 struct page *page;
 struct page *partial;



};






struct kmem_cache_order_objects {
 unsigned long x;
};




struct kmem_cache {
 struct kmem_cache_cpu *cpu_slab;

 unsigned long flags;
 unsigned long min_partial;
 int size;
 int object_size;
 int offset;
 int cpu_partial;
 struct kmem_cache_order_objects oo;


 struct kmem_cache_order_objects max;
 struct kmem_cache_order_objects min;
 gfp_t allocflags;
 int refcount;
 void (*ctor)(void *);
 int inuse;
 int align;
 int reserved;
 const char *name;
 struct list_head list;

 struct kobject kobj;
 int remote_node_defrag_ratio;

 struct kmem_cache_node *node[(1 << 6)];
};



void sysfs_slab_remove(struct kmem_cache *);


extern void *kmalloc_order(size_t size, gfp_t flags, unsigned int order);


extern void *kmalloc_order_trace(size_t size, gfp_t flags, unsigned int order);
static inline __attribute__((no_instrument_function)) __attribute__((always_inline)) void *kmalloc_large(size_t size, gfp_t flags)
{
 unsigned int order = ( __builtin_constant_p(size) ? ( ((size) == 0UL) ? 64 - 12 : (((size) < (1UL << 12)) ? 0 : ( __builtin_constant_p((size) - 1) ? ( ((size) - 1) < 1 ? ____ilog2_NaN() : ((size) - 1) & (1ULL << 63) ? 63 : ((size) - 1) & (1ULL << 62) ? 62 : ((size) - 1) & (1ULL << 61) ? 61 : ((size) - 1) & (1ULL << 60) ? 60 : ((size) - 1) & (1ULL << 59) ? 59 : ((size) - 1) & (1ULL << 58) ? 58 : ((size) - 1) & (1ULL << 57) ? 57 : ((size) - 1) & (1ULL << 56) ? 56 : ((size) - 1) & (1ULL << 55) ? 55 : ((size) - 1) & (1ULL << 54) ? 54 : ((size) - 1) & (1ULL << 53) ? 53 : ((size) - 1) & (1ULL << 52) ? 52 : ((size) - 1) & (1ULL << 51) ? 51 : ((size) - 1) & (1ULL << 50) ? 50 : ((size) - 1) & (1ULL << 49) ? 49 : ((size) - 1) & (1ULL << 48) ? 48 : ((size) - 1) & (1ULL << 47) ? 47 : ((size) - 1) & (1ULL << 46) ? 46 : ((size) - 1) & (1ULL << 45) ? 45 : ((size) - 1) & (1ULL << 44) ? 44 : ((size) - 1) & (1ULL << 43) ? 43 : ((size) - 1) & (1ULL << 42) ? 42 : ((size) - 1) & (1ULL << 41) ? 41 : ((size) - 1) & (1ULL << 40) ? 40 : ((size) - 1) & (1ULL << 39) ? 39 : ((size) - 1) & (1ULL << 38) ? 38 : ((size) - 1) & (1ULL << 37) ? 37 : ((size) - 1) & (1ULL << 36) ? 36 : ((size) - 1) & (1ULL << 35) ? 35 : ((size) - 1) & (1ULL << 34) ? 34 : ((size) - 1) & (1ULL << 33) ? 33 : ((size) - 1) & (1ULL << 32) ? 32 : ((size) - 1) & (1ULL << 31) ? 31 : ((size) - 1) & (1ULL << 30) ? 30 : ((size) - 1) & (1ULL << 29) ? 29 : ((size) - 1) & (1ULL << 28) ? 28 : ((size) - 1) & (1ULL << 27) ? 27 : ((size) - 1) & (1ULL << 26) ? 26 : ((size) - 1) & (1ULL << 25) ? 25 : ((size) - 1) & (1ULL << 24) ? 24 : ((size) - 1) & (1ULL << 23) ? 23 : ((size) - 1) & (1ULL << 22) ? 22 : ((size) - 1) & (1ULL << 21) ? 21 : ((size) - 1) & (1ULL << 20) ? 20 : ((size) - 1) & (1ULL << 19) ? 19 : ((size) - 1) & (1ULL << 18) ? 18 : ((size) - 1) & (1ULL << 17) ? 17 : ((size) - 1) & (1ULL << 16) ? 16 : ((size) - 1) & (1ULL << 15) ? 15 : ((size) - 1) & (1ULL << 14) ? 14 : ((size) - 1) & (1ULL << 13) ? 13 : ((size) - 1) & (1ULL << 12) ? 12 : ((size) - 1) & (1ULL << 11) ? 11 : ((size) - 1) & (1ULL << 10) ? 10 : ((size) - 1) & (1ULL << 9) ? 9 : ((size) - 1) & (1ULL << 8) ? 8 : ((size) - 1) & (1ULL << 7) ? 7 : ((size) - 1) & (1ULL << 6) ? 6 : ((size) - 1) & (1ULL << 5) ? 5 : ((size) - 1) & (1ULL << 4) ? 4 : ((size) - 1) & (1ULL << 3) ? 3 : ((size) - 1) & (1ULL << 2) ? 2 : ((size) - 1) & (1ULL << 1) ? 1 : ((size) - 1) & (1ULL << 0) ? 0 : ____ilog2_NaN() ) : (sizeof((size) - 1) <= 4) ? __ilog2_u32((size) - 1) : __ilog2_u64((size) - 1) ) - 12 + 1) ) : __get_order(size) );
 return kmalloc_order_trace(size, flags, order);
}
static inline __attribute__((no_instrument_function)) __attribute__((always_inline)) void *kmalloc(size_t size, gfp_t flags)
{
 if (__builtin_constant_p(size)) {
  if (size > (1UL << (12 + 1)))
   return kmalloc_large(size, flags);

  if (!(flags & (( gfp_t)0x01u))) {
   int index = kmalloc_index(size);

   if (!index)
    return ((void *)16);

   return kmem_cache_alloc_trace(kmalloc_caches[index],
     flags, size);
  }

 }
 return __kmalloc(size, flags);
}






static inline __attribute__((no_instrument_function)) __attribute__((always_inline)) int kmalloc_size(int n)
{

 if (n > 2)
  return 1 << n;

 if (n == 1 && (1 << 3) <= 32)
  return 96;

 if (n == 2 && (1 << 3) <= 64)
  return 192;

 return 0;
}

static inline __attribute__((no_instrument_function)) __attribute__((always_inline)) void *kmalloc_node(size_t size, gfp_t flags, int node)
{

 if (__builtin_constant_p(size) &&
  size <= (1UL << (12 + 1)) && !(flags & (( gfp_t)0x01u))) {
  int i = kmalloc_index(size);

  if (!i)
   return ((void *)16);

  return kmem_cache_alloc_node_trace(kmalloc_caches[i],
      flags, node, size);
 }

 return __kmalloc_node(size, flags, node);
}
struct memcg_cache_params {
 bool is_root_cache;
 union {
  struct {
   struct callback_head callback_head;
   struct kmem_cache *memcg_caches[0];
  };
  struct {
   struct mem_cgroup *memcg;
   struct list_head list;
   struct kmem_cache *root_cache;
   atomic_t nr_pages;
  };
 };
};

int memcg_update_all_caches(int num_memcgs);

struct seq_file;
int cache_show(struct kmem_cache *s, struct seq_file *m);
void print_slabinfo_header(struct seq_file *m);







static inline __attribute__((no_instrument_function)) void *kmalloc_array(size_t n, size_t size, gfp_t flags)
{
 if (size != 0 && n > (~(size_t)0) / size)
  return ((void *)0);
 return __kmalloc(n * size, flags);
}







static inline __attribute__((no_instrument_function)) void *kcalloc(size_t n, size_t size, gfp_t flags)
{
 return kmalloc_array(n, size, flags | (( gfp_t)0x8000u));
}
extern void *__kmalloc_track_caller(size_t, gfp_t, unsigned long);
extern void *__kmalloc_node_track_caller(size_t, gfp_t, int, unsigned long);
static inline __attribute__((no_instrument_function)) void *kmem_cache_zalloc(struct kmem_cache *k, gfp_t flags)
{
 return kmem_cache_alloc(k, flags | (( gfp_t)0x8000u));
}






static inline __attribute__((no_instrument_function)) void *kzalloc(size_t size, gfp_t flags)
{
 return kmalloc(size, flags | (( gfp_t)0x8000u));
}







static inline __attribute__((no_instrument_function)) void *kzalloc_node(size_t size, gfp_t flags, int node)
{
 return kmalloc_node(size, flags | (( gfp_t)0x8000u), node);
}




static inline __attribute__((no_instrument_function)) unsigned int kmem_cache_size(struct kmem_cache *s)
{
 return s->object_size;
}

void __attribute__ ((__section__(".init.text"))) __attribute__((__cold__)) __attribute__((no_instrument_function)) kmem_cache_init_late(void);
enum irqreturn {
 IRQ_NONE = (0 << 0),
 IRQ_HANDLED = (1 << 0),
 IRQ_WAKE_THREAD = (1 << 1),
};

typedef enum irqreturn irqreturn_t;





extern int nr_irqs;
extern struct irq_desc *irq_to_desc(unsigned int irq);
unsigned int irq_get_next_irq(unsigned int offset);





static inline __attribute__((no_instrument_function)) void ftrace_nmi_enter(void) { }
static inline __attribute__((no_instrument_function)) void ftrace_nmi_exit(void) { }








struct context_tracking {






 bool active;
 enum ctx_state {
  IN_KERNEL = 0,
  IN_USER,
 } state;
};
static inline __attribute__((no_instrument_function)) bool context_tracking_in_user(void) { return false; }
static inline __attribute__((no_instrument_function)) bool context_tracking_active(void) { return false; }





struct task_struct;
static inline __attribute__((no_instrument_function)) bool vtime_accounting_enabled(void) { return false; }
static inline __attribute__((no_instrument_function)) void vtime_task_switch(struct task_struct *prev) { }
static inline __attribute__((no_instrument_function)) void vtime_account_system(struct task_struct *tsk) { }
static inline __attribute__((no_instrument_function)) void vtime_account_user(struct task_struct *tsk) { }
static inline __attribute__((no_instrument_function)) void vtime_account_irq_enter(struct task_struct *tsk) { }
static inline __attribute__((no_instrument_function)) void vtime_account_irq_exit(struct task_struct *tsk)
{

 vtime_account_system(tsk);
}
static inline __attribute__((no_instrument_function)) void vtime_user_enter(struct task_struct *tsk) { }
static inline __attribute__((no_instrument_function)) void vtime_user_exit(struct task_struct *tsk) { }
static inline __attribute__((no_instrument_function)) void vtime_guest_enter(struct task_struct *tsk) { }
static inline __attribute__((no_instrument_function)) void vtime_guest_exit(struct task_struct *tsk) { }
static inline __attribute__((no_instrument_function)) void vtime_init_idle(struct task_struct *tsk, int cpu) { }





static inline __attribute__((no_instrument_function)) void irqtime_account_irq(struct task_struct *tsk) { }


static inline __attribute__((no_instrument_function)) void account_irq_enter_time(struct task_struct *tsk)
{
 vtime_account_irq_enter(tsk);
 irqtime_account_irq(tsk);
}

static inline __attribute__((no_instrument_function)) void account_irq_exit_time(struct task_struct *tsk)
{
 vtime_account_irq_exit(tsk);
 irqtime_account_irq(tsk);
}




static inline __attribute__((no_instrument_function)) int irq_canonicalize(int irq)
{
 return ((irq == 2) ? 9 : irq);
}
extern int check_irq_vectors_for_cpu_disable(void);
extern void fixup_irqs(void);
extern void irq_force_complete_move(int);


extern void (*x86_platform_ipi_callback)(void);
extern void native_init_IRQ(void);
extern bool handle_irq(unsigned irq, struct pt_regs *regs);

extern __attribute__((externally_visible)) unsigned int do_IRQ(struct pt_regs *regs);


extern unsigned long used_vectors[(((256) + (8 * sizeof(long)) - 1) / (8 * sizeof(long)))];
extern int vector_used_by_percpu_irq(unsigned int vector);

extern void init_ISA_irqs(void);


void arch_trigger_all_cpu_backtrace(bool);

extern __attribute__((section(".data..percpu" ""))) __typeof__(struct pt_regs *) irq_regs;

static inline __attribute__((no_instrument_function)) struct pt_regs *get_irq_regs(void)
{
 return ({ typeof(irq_regs) pscr_ret__; do { const void *__vpp_verify = (typeof((&(irq_regs)) + 0))((void *)0); (void)__vpp_verify; } while (0); switch(sizeof(irq_regs)) { case 1: pscr_ret__ = ({ typeof((irq_regs)) pfo_ret__; switch (sizeof((irq_regs))) { case 1: asm("mov" "b ""%%""gs"":" "%P" "1"",%0" : "=q" (pfo_ret__) : "m"(irq_regs)); break; case 2: asm("mov" "w ""%%""gs"":" "%P" "1"",%0" : "=r" (pfo_ret__) : "m"(irq_regs)); break; case 4: asm("mov" "l ""%%""gs"":" "%P" "1"",%0" : "=r" (pfo_ret__) : "m"(irq_regs)); break; case 8: asm("mov" "q ""%%""gs"":" "%P" "1"",%0" : "=r" (pfo_ret__) : "m"(irq_regs)); break; default: __bad_percpu_size(); } pfo_ret__; }); break; case 2: pscr_ret__ = ({ typeof((irq_regs)) pfo_ret__; switch (sizeof((irq_regs))) { case 1: asm("mov" "b ""%%""gs"":" "%P" "1"",%0" : "=q" (pfo_ret__) : "m"(irq_regs)); break; case 2: asm("mov" "w ""%%""gs"":" "%P" "1"",%0" : "=r" (pfo_ret__) : "m"(irq_regs)); break; case 4: asm("mov" "l ""%%""gs"":" "%P" "1"",%0" : "=r" (pfo_ret__) : "m"(irq_regs)); break; case 8: asm("mov" "q ""%%""gs"":" "%P" "1"",%0" : "=r" (pfo_ret__) : "m"(irq_regs)); break; default: __bad_percpu_size(); } pfo_ret__; }); break; case 4: pscr_ret__ = ({ typeof((irq_regs)) pfo_ret__; switch (sizeof((irq_regs))) { case 1: asm("mov" "b ""%%""gs"":" "%P" "1"",%0" : "=q" (pfo_ret__) : "m"(irq_regs)); break; case 2: asm("mov" "w ""%%""gs"":" "%P" "1"",%0" : "=r" (pfo_ret__) : "m"(irq_regs)); break; case 4: asm("mov" "l ""%%""gs"":" "%P" "1"",%0" : "=r" (pfo_ret__) : "m"(irq_regs)); break; case 8: asm("mov" "q ""%%""gs"":" "%P" "1"",%0" : "=r" (pfo_ret__) : "m"(irq_regs)); break; default: __bad_percpu_size(); } pfo_ret__; }); break; case 8: pscr_ret__ = ({ typeof((irq_regs)) pfo_ret__; switch (sizeof((irq_regs))) { case 1: asm("mov" "b ""%%""gs"":" "%P" "1"",%0" : "=q" (pfo_ret__) : "m"(irq_regs)); break; case 2: asm("mov" "w ""%%""gs"":" "%P" "1"",%0" : "=r" (pfo_ret__) : "m"(irq_regs)); break; case 4: asm("mov" "l ""%%""gs"":" "%P" "1"",%0" : "=r" (pfo_ret__) : "m"(irq_regs)); break; case 8: asm("mov" "q ""%%""gs"":" "%P" "1"",%0" : "=r" (pfo_ret__) : "m"(irq_regs)); break; default: __bad_percpu_size(); } pfo_ret__; }); break; default: __bad_size_call_parameter(); break; } pscr_ret__; });
}

static inline __attribute__((no_instrument_function)) struct pt_regs *set_irq_regs(struct pt_regs *new_regs)
{
 struct pt_regs *old_regs;

 old_regs = get_irq_regs();
 do { do { const void *__vpp_verify = (typeof((&(irq_regs)) + 0))((void *)0); (void)__vpp_verify; } while (0); switch(sizeof(irq_regs)) { case 1: do { typedef typeof((irq_regs)) pto_T__; if (0) { pto_T__ pto_tmp__; pto_tmp__ = (new_regs); (void)pto_tmp__; } switch (sizeof((irq_regs))) { case 1: asm("mov" "b %1,""%%""gs"":" "%P" "0" : "+m" ((irq_regs)) : "qi" ((pto_T__)(new_regs))); break; case 2: asm("mov" "w %1,""%%""gs"":" "%P" "0" : "+m" ((irq_regs)) : "ri" ((pto_T__)(new_regs))); break; case 4: asm("mov" "l %1,""%%""gs"":" "%P" "0" : "+m" ((irq_regs)) : "ri" ((pto_T__)(new_regs))); break; case 8: asm("mov" "q %1,""%%""gs"":" "%P" "0" : "+m" ((irq_regs)) : "re" ((pto_T__)(new_regs))); break; default: __bad_percpu_size(); } } while (0);break; case 2: do { typedef typeof((irq_regs)) pto_T__; if (0) { pto_T__ pto_tmp__; pto_tmp__ = (new_regs); (void)pto_tmp__; } switch (sizeof((irq_regs))) { case 1: asm("mov" "b %1,""%%""gs"":" "%P" "0" : "+m" ((irq_regs)) : "qi" ((pto_T__)(new_regs))); break; case 2: asm("mov" "w %1,""%%""gs"":" "%P" "0" : "+m" ((irq_regs)) : "ri" ((pto_T__)(new_regs))); break; case 4: asm("mov" "l %1,""%%""gs"":" "%P" "0" : "+m" ((irq_regs)) : "ri" ((pto_T__)(new_regs))); break; case 8: asm("mov" "q %1,""%%""gs"":" "%P" "0" : "+m" ((irq_regs)) : "re" ((pto_T__)(new_regs))); break; default: __bad_percpu_size(); } } while (0);break; case 4: do { typedef typeof((irq_regs)) pto_T__; if (0) { pto_T__ pto_tmp__; pto_tmp__ = (new_regs); (void)pto_tmp__; } switch (sizeof((irq_regs))) { case 1: asm("mov" "b %1,""%%""gs"":" "%P" "0" : "+m" ((irq_regs)) : "qi" ((pto_T__)(new_regs))); break; case 2: asm("mov" "w %1,""%%""gs"":" "%P" "0" : "+m" ((irq_regs)) : "ri" ((pto_T__)(new_regs))); break; case 4: asm("mov" "l %1,""%%""gs"":" "%P" "0" : "+m" ((irq_regs)) : "ri" ((pto_T__)(new_regs))); break; case 8: asm("mov" "q %1,""%%""gs"":" "%P" "0" : "+m" ((irq_regs)) : "re" ((pto_T__)(new_regs))); break; default: __bad_percpu_size(); } } while (0);break; case 8: do { typedef typeof((irq_regs)) pto_T__; if (0) { pto_T__ pto_tmp__; pto_tmp__ = (new_regs); (void)pto_tmp__; } switch (sizeof((irq_regs))) { case 1: asm("mov" "b %1,""%%""gs"":" "%P" "0" : "+m" ((irq_regs)) : "qi" ((pto_T__)(new_regs))); break; case 2: asm("mov" "w %1,""%%""gs"":" "%P" "0" : "+m" ((irq_regs)) : "ri" ((pto_T__)(new_regs))); break; case 4: asm("mov" "l %1,""%%""gs"":" "%P" "0" : "+m" ((irq_regs)) : "ri" ((pto_T__)(new_regs))); break; case 8: asm("mov" "q %1,""%%""gs"":" "%P" "0" : "+m" ((irq_regs)) : "re" ((pto_T__)(new_regs))); break; default: __bad_percpu_size(); } } while (0);break; default: __bad_size_call_parameter();break; } } while (0);

 return old_regs;
}

struct seq_file;
struct module;
struct irq_desc;
struct irq_data;
typedef void (*irq_flow_handler_t)(unsigned int irq,
         struct irq_desc *desc);
typedef void (*irq_preflow_handler_t)(struct irq_data *data);
enum {
 IRQ_TYPE_NONE = 0x00000000,
 IRQ_TYPE_EDGE_RISING = 0x00000001,
 IRQ_TYPE_EDGE_FALLING = 0x00000002,
 IRQ_TYPE_EDGE_BOTH = (IRQ_TYPE_EDGE_FALLING | IRQ_TYPE_EDGE_RISING),
 IRQ_TYPE_LEVEL_HIGH = 0x00000004,
 IRQ_TYPE_LEVEL_LOW = 0x00000008,
 IRQ_TYPE_LEVEL_MASK = (IRQ_TYPE_LEVEL_LOW | IRQ_TYPE_LEVEL_HIGH),
 IRQ_TYPE_SENSE_MASK = 0x0000000f,
 IRQ_TYPE_DEFAULT = IRQ_TYPE_SENSE_MASK,

 IRQ_TYPE_PROBE = 0x00000010,

 IRQ_LEVEL = (1 << 8),
 IRQ_PER_CPU = (1 << 9),
 IRQ_NOPROBE = (1 << 10),
 IRQ_NOREQUEST = (1 << 11),
 IRQ_NOAUTOEN = (1 << 12),
 IRQ_NO_BALANCING = (1 << 13),
 IRQ_MOVE_PCNTXT = (1 << 14),
 IRQ_NESTED_THREAD = (1 << 15),
 IRQ_NOTHREAD = (1 << 16),
 IRQ_PER_CPU_DEVID = (1 << 17),
 IRQ_IS_POLLED = (1 << 18),
};
enum {
 IRQ_SET_MASK_OK = 0,
 IRQ_SET_MASK_OK_NOCOPY,
};

struct msi_desc;
struct irq_domain;
struct irq_data {
 u32 mask;
 unsigned int irq;
 unsigned long hwirq;
 unsigned int node;
 unsigned int state_use_accessors;
 struct irq_chip *chip;
 struct irq_domain *domain;
 void *handler_data;
 void *chip_data;
 struct msi_desc *msi_desc;
 cpumask_var_t affinity;
};
enum {
 IRQD_TRIGGER_MASK = 0xf,
 IRQD_SETAFFINITY_PENDING = (1 << 8),
 IRQD_NO_BALANCING = (1 << 10),
 IRQD_PER_CPU = (1 << 11),
 IRQD_AFFINITY_SET = (1 << 12),
 IRQD_LEVEL = (1 << 13),
 IRQD_WAKEUP_STATE = (1 << 14),
 IRQD_MOVE_PCNTXT = (1 << 15),
 IRQD_IRQ_DISABLED = (1 << 16),
 IRQD_IRQ_MASKED = (1 << 17),
 IRQD_IRQ_INPROGRESS = (1 << 18),
};

static inline __attribute__((no_instrument_function)) bool irqd_is_setaffinity_pending(struct irq_data *d)
{
 return d->state_use_accessors & IRQD_SETAFFINITY_PENDING;
}

static inline __attribute__((no_instrument_function)) bool irqd_is_per_cpu(struct irq_data *d)
{
 return d->state_use_accessors & IRQD_PER_CPU;
}

static inline __attribute__((no_instrument_function)) bool irqd_can_balance(struct irq_data *d)
{
 return !(d->state_use_accessors & (IRQD_PER_CPU | IRQD_NO_BALANCING));
}

static inline __attribute__((no_instrument_function)) bool irqd_affinity_was_set(struct irq_data *d)
{
 return d->state_use_accessors & IRQD_AFFINITY_SET;
}

static inline __attribute__((no_instrument_function)) void irqd_mark_affinity_was_set(struct irq_data *d)
{
 d->state_use_accessors |= IRQD_AFFINITY_SET;
}

static inline __attribute__((no_instrument_function)) u32 irqd_get_trigger_type(struct irq_data *d)
{
 return d->state_use_accessors & IRQD_TRIGGER_MASK;
}




static inline __attribute__((no_instrument_function)) void irqd_set_trigger_type(struct irq_data *d, u32 type)
{
 d->state_use_accessors &= ~IRQD_TRIGGER_MASK;
 d->state_use_accessors |= type & IRQD_TRIGGER_MASK;
}

static inline __attribute__((no_instrument_function)) bool irqd_is_level_type(struct irq_data *d)
{
 return d->state_use_accessors & IRQD_LEVEL;
}

static inline __attribute__((no_instrument_function)) bool irqd_is_wakeup_set(struct irq_data *d)
{
 return d->state_use_accessors & IRQD_WAKEUP_STATE;
}

static inline __attribute__((no_instrument_function)) bool irqd_can_move_in_process_context(struct irq_data *d)
{
 return d->state_use_accessors & IRQD_MOVE_PCNTXT;
}

static inline __attribute__((no_instrument_function)) bool irqd_irq_disabled(struct irq_data *d)
{
 return d->state_use_accessors & IRQD_IRQ_DISABLED;
}

static inline __attribute__((no_instrument_function)) bool irqd_irq_masked(struct irq_data *d)
{
 return d->state_use_accessors & IRQD_IRQ_MASKED;
}

static inline __attribute__((no_instrument_function)) bool irqd_irq_inprogress(struct irq_data *d)
{
 return d->state_use_accessors & IRQD_IRQ_INPROGRESS;
}






static inline __attribute__((no_instrument_function)) void irqd_set_chained_irq_inprogress(struct irq_data *d)
{
 d->state_use_accessors |= IRQD_IRQ_INPROGRESS;
}

static inline __attribute__((no_instrument_function)) void irqd_clr_chained_irq_inprogress(struct irq_data *d)
{
 d->state_use_accessors &= ~IRQD_IRQ_INPROGRESS;
}

static inline __attribute__((no_instrument_function)) irq_hw_number_t irqd_to_hwirq(struct irq_data *d)
{
 return d->hwirq;
}
struct irq_chip {
 const char *name;
 unsigned int (*irq_startup)(struct irq_data *data);
 void (*irq_shutdown)(struct irq_data *data);
 void (*irq_enable)(struct irq_data *data);
 void (*irq_disable)(struct irq_data *data);

 void (*irq_ack)(struct irq_data *data);
 void (*irq_mask)(struct irq_data *data);
 void (*irq_mask_ack)(struct irq_data *data);
 void (*irq_unmask)(struct irq_data *data);
 void (*irq_eoi)(struct irq_data *data);

 int (*irq_set_affinity)(struct irq_data *data, const struct cpumask *dest, bool force);
 int (*irq_retrigger)(struct irq_data *data);
 int (*irq_set_type)(struct irq_data *data, unsigned int flow_type);
 int (*irq_set_wake)(struct irq_data *data, unsigned int on);

 void (*irq_bus_lock)(struct irq_data *data);
 void (*irq_bus_sync_unlock)(struct irq_data *data);

 void (*irq_cpu_online)(struct irq_data *data);
 void (*irq_cpu_offline)(struct irq_data *data);

 void (*irq_suspend)(struct irq_data *data);
 void (*irq_resume)(struct irq_data *data);
 void (*irq_pm_shutdown)(struct irq_data *data);

 void (*irq_calc_mask)(struct irq_data *data);

 void (*irq_print_chip)(struct irq_data *data, struct seq_file *p);
 int (*irq_request_resources)(struct irq_data *data);
 void (*irq_release_resources)(struct irq_data *data);

 unsigned long flags;
};
enum {
 IRQCHIP_SET_TYPE_MASKED = (1 << 0),
 IRQCHIP_EOI_IF_HANDLED = (1 << 1),
 IRQCHIP_MASK_ON_SUSPEND = (1 << 2),
 IRQCHIP_ONOFFLINE_ENABLED = (1 << 3),
 IRQCHIP_SKIP_SET_WAKE = (1 << 4),
 IRQCHIP_ONESHOT_SAFE = (1 << 5),
 IRQCHIP_EOI_THREADED = (1 << 6),
};


struct irq_affinity_notify;
struct proc_dir_entry;
struct module;
struct irq_desc;
struct irq_desc {
 struct irq_data irq_data;
 unsigned int *kstat_irqs;
 irq_flow_handler_t handle_irq;



 struct irqaction *action;
 unsigned int status_use_accessors;
 unsigned int core_internal_state__do_not_mess_with_it;
 unsigned int depth;
 unsigned int wake_depth;
 unsigned int irq_count;
 unsigned long last_unhandled;
 unsigned int irqs_unhandled;
 atomic_t threads_handled;
 int threads_handled_last;
 raw_spinlock_t lock;
 struct cpumask *percpu_enabled;

 const struct cpumask *affinity_hint;
 struct irq_affinity_notify *affinity_notify;

 cpumask_var_t pending_mask;


 unsigned long threads_oneshot;
 atomic_t threads_active;
 wait_queue_head_t wait_for_threads;

 struct proc_dir_entry *dir;

 int parent_irq;
 struct module *owner;
 const char *name;
} __attribute__((__aligned__(1 << (6))));





static inline __attribute__((no_instrument_function)) struct irq_data *irq_desc_get_irq_data(struct irq_desc *desc)
{
 return &desc->irq_data;
}

static inline __attribute__((no_instrument_function)) struct irq_chip *irq_desc_get_chip(struct irq_desc *desc)
{
 return desc->irq_data.chip;
}

static inline __attribute__((no_instrument_function)) void *irq_desc_get_chip_data(struct irq_desc *desc)
{
 return desc->irq_data.chip_data;
}

static inline __attribute__((no_instrument_function)) void *irq_desc_get_handler_data(struct irq_desc *desc)
{
 return desc->irq_data.handler_data;
}

static inline __attribute__((no_instrument_function)) struct msi_desc *irq_desc_get_msi_desc(struct irq_desc *desc)
{
 return desc->irq_data.msi_desc;
}







static inline __attribute__((no_instrument_function)) void generic_handle_irq_desc(unsigned int irq, struct irq_desc *desc)
{
 desc->handle_irq(irq, desc);
}

int generic_handle_irq(unsigned int irq);


static inline __attribute__((no_instrument_function)) int irq_has_action(unsigned int irq)
{
 struct irq_desc *desc = irq_to_desc(irq);
 return desc->action != ((void *)0);
}


static inline __attribute__((no_instrument_function)) void __irq_set_handler_locked(unsigned int irq,
         irq_flow_handler_t handler)
{
 struct irq_desc *desc;

 desc = irq_to_desc(irq);
 desc->handle_irq = handler;
}


static inline __attribute__((no_instrument_function)) void
__irq_set_chip_handler_name_locked(unsigned int irq, struct irq_chip *chip,
       irq_flow_handler_t handler, const char *name)
{
 struct irq_desc *desc;

 desc = irq_to_desc(irq);
 irq_desc_get_irq_data(desc)->chip = chip;
 desc->handle_irq = handler;
 desc->name = name;
}

static inline __attribute__((no_instrument_function)) int irq_balancing_disabled(unsigned int irq)
{
 struct irq_desc *desc;

 desc = irq_to_desc(irq);
 return desc->status_use_accessors & (IRQ_PER_CPU | IRQ_NO_BALANCING);
}

static inline __attribute__((no_instrument_function)) int irq_is_percpu(unsigned int irq)
{
 struct irq_desc *desc;

 desc = irq_to_desc(irq);
 return desc->status_use_accessors & IRQ_PER_CPU;
}

static inline __attribute__((no_instrument_function)) void
irq_set_lockdep_class(unsigned int irq, struct lock_class_key *class)
{
 struct irq_desc *desc = irq_to_desc(irq);

 if (desc)
  do { (void)(class); } while (0);
}










struct proc_dir_entry;
struct pt_regs;
struct notifier_block;


void create_prof_cpu_mask(void);
int create_proc_profile(void);
enum profile_type {
 PROFILE_TASK_EXIT,
 PROFILE_MUNMAP
};



extern int prof_on __attribute__((__section__(".data..read_mostly")));


int profile_init(void);
int profile_setup(char *str);
void profile_tick(int type);
int setup_profiling_timer(unsigned int multiplier);




void profile_hits(int type, void *ip, unsigned int nr_hits);




static inline __attribute__((no_instrument_function)) void profile_hit(int type, void *ip)
{



 if (__builtin_expect(!!(prof_on == type), 0))
  profile_hits(type, ip, 1);
}

struct task_struct;
struct mm_struct;


void profile_task_exit(struct task_struct * task);




int profile_handoff_task(struct task_struct * task);


void profile_munmap(unsigned long addr);

int task_handoff_register(struct notifier_block * n);
int task_handoff_unregister(struct notifier_block * n);

int profile_event_register(enum profile_type, struct notifier_block * n);
int profile_event_unregister(enum profile_type, struct notifier_block * n);

struct pt_regs;







extern char _text[], _stext[], _etext[];
extern char _data[], _sdata[], _edata[];
extern char __bss_start[], __bss_stop[];
extern char __init_begin[], __init_end[];
extern char _sinittext[], _einittext[];
extern char _end[];
extern char __per_cpu_load[], __per_cpu_start[], __per_cpu_end[];
extern char __kprobes_text_start[], __kprobes_text_end[];
extern char __entry_text_start[], __entry_text_end[];
extern char __start_rodata[], __end_rodata[];


extern char __ctors_start[], __ctors_end[];
static inline __attribute__((no_instrument_function)) int arch_is_kernel_text(unsigned long addr)
{
 return 0;
}



static inline __attribute__((no_instrument_function)) int arch_is_kernel_data(unsigned long addr)
{
 return 0;
}
static inline __attribute__((no_instrument_function)) __attribute__((always_inline)) void clac(void)
{

 asm volatile ("661:\n\t" ".byte " "0x66,0x66,0x90" "\n" "\n662:\n" ".pushsection .altinstructions,\"a\"\n" " .long 661b - .\n" " .long " "663""1""f - .\n" " .word " "( 9*32+20)" "\n" " .byte " "662b-661b" "\n" " .byte " "664""1""f-""663""1""f" "\n" ".popsection\n" ".pushsection .discard,\"aw\",@progbits\n" " .byte 0xff + (" "664""1""f-""663""1""f" ") - (" "662b-661b" ")\n" ".popsection\n" ".pushsection .altinstr_replacement, \"ax\"\n" "663""1"":\n\t" ".byte 0x0f,0x01,0xca" "\n" "664""1" ":\n\t" ".popsection" : : : "memory");
}

static inline __attribute__((no_instrument_function)) __attribute__((always_inline)) void stac(void)
{

 asm volatile ("661:\n\t" ".byte " "0x66,0x66,0x90" "\n" "\n662:\n" ".pushsection .altinstructions,\"a\"\n" " .long 661b - .\n" " .long " "663""1""f - .\n" " .word " "( 9*32+20)" "\n" " .byte " "662b-661b" "\n" " .byte " "664""1""f-""663""1""f" "\n" ".popsection\n" ".pushsection .discard,\"aw\",@progbits\n" " .byte 0xff + (" "664""1""f-""663""1""f" ") - (" "662b-661b" ")\n" ".popsection\n" ".pushsection .altinstr_replacement, \"ax\"\n" "663""1"":\n\t" ".byte 0x0f,0x01,0xcb" "\n" "664""1" ":\n\t" ".popsection" : : : "memory");
}
static inline __attribute__((no_instrument_function)) bool __chk_range_not_ok(unsigned long addr, unsigned long size, unsigned long limit)
{







 if (__builtin_constant_p(size))
  return addr > limit - size;


 addr += size;
 if (addr < size)
  return true;
 return addr > limit;
}
struct exception_table_entry {
 int insn, fixup;
};




extern int fixup_exception(struct pt_regs *regs);
extern int early_fixup_exception(unsigned long *ip);
extern int __get_user_1(void);
extern int __get_user_2(void);
extern int __get_user_4(void);
extern int __get_user_8(void);
extern int __get_user_bad(void);
extern void __put_user_bad(void);





extern void __put_user_1(void);
extern void __put_user_2(void);
extern void __put_user_4(void);
extern void __put_user_8(void);
struct __large_struct { unsigned long buf[100]; };
extern unsigned long
copy_from_user_nmi(void *to, const void *from, unsigned long n);
extern long
strncpy_from_user(char *dst, const char *src, long count);

extern long strlen_user(const char *str);
extern long strnlen_user(const char *str, long n);

unsigned long clear_user(void *mem, unsigned long len);
unsigned long __clear_user(void *mem, unsigned long len);

extern void __cmpxchg_wrong_size(void)
 __attribute__((error("Bad argument size for cmpxchg")));
 unsigned long
copy_user_enhanced_fast_string(void *to, const void *from, unsigned len);
 unsigned long
copy_user_generic_string(void *to, const void *from, unsigned len);
 unsigned long
copy_user_generic_unrolled(void *to, const void *from, unsigned len);

static inline __attribute__((no_instrument_function)) __attribute__((always_inline)) unsigned long
copy_user_generic(void *to, const void *from, unsigned len)
{
 unsigned ret;






 asm volatile ("661:\n\t" "call %P[old]" "\n662:\n" ".pushsection .altinstructions,\"a\"\n" " .long 661b - .\n" " .long " "663""1""f - .\n" " .word " "( 3*32+16)" "\n" " .byte " "662b-661b" "\n" " .byte " "664""1""f-""663""1""f" "\n" " .long 661b - .\n" " .long " "663""2""f - .\n" " .word " "( 9*32+ 9)" "\n" " .byte " "662b-661b" "\n" " .byte " "664""2""f-""663""2""f" "\n" ".popsection\n" ".pushsection .discard,\"aw\",@progbits\n" " .byte 0xff + (" "664""1""f-""663""1""f" ") - (" "662b-661b" ")\n" " .byte 0xff + (" "664""2""f-""663""2""f" ") - (" "662b-661b" ")\n" ".popsection\n" ".pushsection .altinstr_replacement, \"ax\"\n" "663""1"":\n\t" "call %P[new1]" "\n" "664""1" ":\n\t" "663""2"":\n\t" "call %P[new2]" "\n" "664""2" ":\n\t" ".popsection" : "=a" (ret), "=D" (to), "=S" (from), "=d" (len) : [old] "i" (copy_user_generic_unrolled), [new1] "i" (copy_user_generic_string), [new2] "i" (copy_user_enhanced_fast_string), "1" (to), "2" (from), "3" (len) : "memory", "rcx", "r8", "r9", "r10", "r11")







                                                ;
 return ret;
}

 unsigned long
copy_in_user(void *to, const void *from, unsigned len);

static inline __attribute__((no_instrument_function)) __attribute__((always_inline))
int __copy_from_user_nocheck(void *dst, const void *src, unsigned size)
{
 int ret = 0;

 if (!__builtin_constant_p(size))
  return copy_user_generic(dst, ( void *)src, size);
 switch (size) {
 case 1:asm volatile("661:\n\t" ".byte " "0x66,0x66,0x90" "\n" "\n662:\n" ".pushsection .altinstructions,\"a\"\n" " .long 661b - .\n" " .long " "663""1""f - .\n" " .word " "( 9*32+20)" "\n" " .byte " "662b-661b" "\n" " .byte " "664""1""f-""663""1""f" "\n" ".popsection\n" ".pushsection .discard,\"aw\",@progbits\n" " .byte 0xff + (" "664""1""f-""663""1""f" ") - (" "662b-661b" ")\n" ".popsection\n" ".pushsection .altinstr_replacement, \"ax\"\n" "663""1"":\n\t" ".byte 0x0f,0x01,0xcb" "\n" "664""1" ":\n\t" ".popsection" "\n" "1:	mov""b"" %2,%""b""1\n" "2: " "661:\n\t" ".byte " "0x66,0x66,0x90" "\n" "\n662:\n" ".pushsection .altinstructions,\"a\"\n" " .long 661b - .\n" " .long " "663""1""f - .\n" " .word " "( 9*32+20)" "\n" " .byte " "662b-661b" "\n" " .byte " "664""1""f-""663""1""f" "\n" ".popsection\n" ".pushsection .discard,\"aw\",@progbits\n" " .byte 0xff + (" "664""1""f-""663""1""f" ") - (" "662b-661b" ")\n" ".popsection\n" ".pushsection .altinstr_replacement, \"ax\"\n" "663""1"":\n\t" ".byte 0x0f,0x01,0xca" "\n" "664""1" ":\n\t" ".popsection" "\n" ".section .fixup,\"ax\"\n" "3:	mov %3,%0\n" "	xor""b"" %""b""1,%""b""1\n" "	jmp 2b\n" ".previous\n" " .pushsection \"__ex_table\",\"a\"\n" " .balign 8\n" " .long (" "1b" ") - .\n" " .long (" "3b" ") - .\n" " .popsection\n" : "=r" (ret), "=q"(*(u8 *)dst) : "m" ((*(struct __large_struct *)((u8 *)src))), "i" (1), "0" (ret))
                                ;
  return ret;
 case 2:asm volatile("661:\n\t" ".byte " "0x66,0x66,0x90" "\n" "\n662:\n" ".pushsection .altinstructions,\"a\"\n" " .long 661b - .\n" " .long " "663""1""f - .\n" " .word " "( 9*32+20)" "\n" " .byte " "662b-661b" "\n" " .byte " "664""1""f-""663""1""f" "\n" ".popsection\n" ".pushsection .discard,\"aw\",@progbits\n" " .byte 0xff + (" "664""1""f-""663""1""f" ") - (" "662b-661b" ")\n" ".popsection\n" ".pushsection .altinstr_replacement, \"ax\"\n" "663""1"":\n\t" ".byte 0x0f,0x01,0xcb" "\n" "664""1" ":\n\t" ".popsection" "\n" "1:	mov""w"" %2,%""w""1\n" "2: " "661:\n\t" ".byte " "0x66,0x66,0x90" "\n" "\n662:\n" ".pushsection .altinstructions,\"a\"\n" " .long 661b - .\n" " .long " "663""1""f - .\n" " .word " "( 9*32+20)" "\n" " .byte " "662b-661b" "\n" " .byte " "664""1""f-""663""1""f" "\n" ".popsection\n" ".pushsection .discard,\"aw\",@progbits\n" " .byte 0xff + (" "664""1""f-""663""1""f" ") - (" "662b-661b" ")\n" ".popsection\n" ".pushsection .altinstr_replacement, \"ax\"\n" "663""1"":\n\t" ".byte 0x0f,0x01,0xca" "\n" "664""1" ":\n\t" ".popsection" "\n" ".section .fixup,\"ax\"\n" "3:	mov %3,%0\n" "	xor""w"" %""w""1,%""w""1\n" "	jmp 2b\n" ".previous\n" " .pushsection \"__ex_table\",\"a\"\n" " .balign 8\n" " .long (" "1b" ") - .\n" " .long (" "3b" ") - .\n" " .popsection\n" : "=r" (ret), "=r"(*(u16 *)dst) : "m" ((*(struct __large_struct *)((u16 *)src))), "i" (2), "0" (ret))
                                ;
  return ret;
 case 4:asm volatile("661:\n\t" ".byte " "0x66,0x66,0x90" "\n" "\n662:\n" ".pushsection .altinstructions,\"a\"\n" " .long 661b - .\n" " .long " "663""1""f - .\n" " .word " "( 9*32+20)" "\n" " .byte " "662b-661b" "\n" " .byte " "664""1""f-""663""1""f" "\n" ".popsection\n" ".pushsection .discard,\"aw\",@progbits\n" " .byte 0xff + (" "664""1""f-""663""1""f" ") - (" "662b-661b" ")\n" ".popsection\n" ".pushsection .altinstr_replacement, \"ax\"\n" "663""1"":\n\t" ".byte 0x0f,0x01,0xcb" "\n" "664""1" ":\n\t" ".popsection" "\n" "1:	mov""l"" %2,%""k""1\n" "2: " "661:\n\t" ".byte " "0x66,0x66,0x90" "\n" "\n662:\n" ".pushsection .altinstructions,\"a\"\n" " .long 661b - .\n" " .long " "663""1""f - .\n" " .word " "( 9*32+20)" "\n" " .byte " "662b-661b" "\n" " .byte " "664""1""f-""663""1""f" "\n" ".popsection\n" ".pushsection .discard,\"aw\",@progbits\n" " .byte 0xff + (" "664""1""f-""663""1""f" ") - (" "662b-661b" ")\n" ".popsection\n" ".pushsection .altinstr_replacement, \"ax\"\n" "663""1"":\n\t" ".byte 0x0f,0x01,0xca" "\n" "664""1" ":\n\t" ".popsection" "\n" ".section .fixup,\"ax\"\n" "3:	mov %3,%0\n" "	xor""l"" %""k""1,%""k""1\n" "	jmp 2b\n" ".previous\n" " .pushsection \"__ex_table\",\"a\"\n" " .balign 8\n" " .long (" "1b" ") - .\n" " .long (" "3b" ") - .\n" " .popsection\n" : "=r" (ret), "=r"(*(u32 *)dst) : "m" ((*(struct __large_struct *)((u32 *)src))), "i" (4), "0" (ret))
                                ;
  return ret;
 case 8:asm volatile("661:\n\t" ".byte " "0x66,0x66,0x90" "\n" "\n662:\n" ".pushsection .altinstructions,\"a\"\n" " .long 661b - .\n" " .long " "663""1""f - .\n" " .word " "( 9*32+20)" "\n" " .byte " "662b-661b" "\n" " .byte " "664""1""f-""663""1""f" "\n" ".popsection\n" ".pushsection .discard,\"aw\",@progbits\n" " .byte 0xff + (" "664""1""f-""663""1""f" ") - (" "662b-661b" ")\n" ".popsection\n" ".pushsection .altinstr_replacement, \"ax\"\n" "663""1"":\n\t" ".byte 0x0f,0x01,0xcb" "\n" "664""1" ":\n\t" ".popsection" "\n" "1:	mov""q"" %2,%""""1\n" "2: " "661:\n\t" ".byte " "0x66,0x66,0x90" "\n" "\n662:\n" ".pushsection .altinstructions,\"a\"\n" " .long 661b - .\n" " .long " "663""1""f - .\n" " .word " "( 9*32+20)" "\n" " .byte " "662b-661b" "\n" " .byte " "664""1""f-""663""1""f" "\n" ".popsection\n" ".pushsection .discard,\"aw\",@progbits\n" " .byte 0xff + (" "664""1""f-""663""1""f" ") - (" "662b-661b" ")\n" ".popsection\n" ".pushsection .altinstr_replacement, \"ax\"\n" "663""1"":\n\t" ".byte 0x0f,0x01,0xca" "\n" "664""1" ":\n\t" ".popsection" "\n" ".section .fixup,\"ax\"\n" "3:	mov %3,%0\n" "	xor""q"" %""""1,%""""1\n" "	jmp 2b\n" ".previous\n" " .pushsection \"__ex_table\",\"a\"\n" " .balign 8\n" " .long (" "1b" ") - .\n" " .long (" "3b" ") - .\n" " .popsection\n" : "=r" (ret), "=r"(*(u64 *)dst) : "m" ((*(struct __large_struct *)((u64 *)src))), "i" (8), "0" (ret))
                               ;
  return ret;
 case 10:
  asm volatile("661:\n\t" ".byte " "0x66,0x66,0x90" "\n" "\n662:\n" ".pushsection .altinstructions,\"a\"\n" " .long 661b - .\n" " .long " "663""1""f - .\n" " .word " "( 9*32+20)" "\n" " .byte " "662b-661b" "\n" " .byte " "664""1""f-""663""1""f" "\n" ".popsection\n" ".pushsection .discard,\"aw\",@progbits\n" " .byte 0xff + (" "664""1""f-""663""1""f" ") - (" "662b-661b" ")\n" ".popsection\n" ".pushsection .altinstr_replacement, \"ax\"\n" "663""1"":\n\t" ".byte 0x0f,0x01,0xcb" "\n" "664""1" ":\n\t" ".popsection" "\n" "1:	mov""q"" %2,%""""1\n" "2: " "661:\n\t" ".byte " "0x66,0x66,0x90" "\n" "\n662:\n" ".pushsection .altinstructions,\"a\"\n" " .long 661b - .\n" " .long " "663""1""f - .\n" " .word " "( 9*32+20)" "\n" " .byte " "662b-661b" "\n" " .byte " "664""1""f-""663""1""f" "\n" ".popsection\n" ".pushsection .discard,\"aw\",@progbits\n" " .byte 0xff + (" "664""1""f-""663""1""f" ") - (" "662b-661b" ")\n" ".popsection\n" ".pushsection .altinstr_replacement, \"ax\"\n" "663""1"":\n\t" ".byte 0x0f,0x01,0xca" "\n" "664""1" ":\n\t" ".popsection" "\n" ".section .fixup,\"ax\"\n" "3:	mov %3,%0\n" "	xor""q"" %""""1,%""""1\n" "	jmp 2b\n" ".previous\n" " .pushsection \"__ex_table\",\"a\"\n" " .balign 8\n" " .long (" "1b" ") - .\n" " .long (" "3b" ") - .\n" " .popsection\n" : "=r" (ret), "=r"(*(u64 *)dst) : "m" ((*(struct __large_struct *)((u64 *)src))), "i" (10), "0" (ret))
                                 ;
  if (__builtin_expect(!!(ret), 0))
   return ret;
  asm volatile("661:\n\t" ".byte " "0x66,0x66,0x90" "\n" "\n662:\n" ".pushsection .altinstructions,\"a\"\n" " .long 661b - .\n" " .long " "663""1""f - .\n" " .word " "( 9*32+20)" "\n" " .byte " "662b-661b" "\n" " .byte " "664""1""f-""663""1""f" "\n" ".popsection\n" ".pushsection .discard,\"aw\",@progbits\n" " .byte 0xff + (" "664""1""f-""663""1""f" ") - (" "662b-661b" ")\n" ".popsection\n" ".pushsection .altinstr_replacement, \"ax\"\n" "663""1"":\n\t" ".byte 0x0f,0x01,0xcb" "\n" "664""1" ":\n\t" ".popsection" "\n" "1:	mov""w"" %2,%""w""1\n" "2: " "661:\n\t" ".byte " "0x66,0x66,0x90" "\n" "\n662:\n" ".pushsection .altinstructions,\"a\"\n" " .long 661b - .\n" " .long " "663""1""f - .\n" " .word " "( 9*32+20)" "\n" " .byte " "662b-661b" "\n" " .byte " "664""1""f-""663""1""f" "\n" ".popsection\n" ".pushsection .discard,\"aw\",@progbits\n" " .byte 0xff + (" "664""1""f-""663""1""f" ") - (" "662b-661b" ")\n" ".popsection\n" ".pushsection .altinstr_replacement, \"ax\"\n" "663""1"":\n\t" ".byte 0x0f,0x01,0xca" "\n" "664""1" ":\n\t" ".popsection" "\n" ".section .fixup,\"ax\"\n" "3:	mov %3,%0\n" "	xor""w"" %""w""1,%""w""1\n" "	jmp 2b\n" ".previous\n" " .pushsection \"__ex_table\",\"a\"\n" " .balign 8\n" " .long (" "1b" ") - .\n" " .long (" "3b" ") - .\n" " .popsection\n" : "=r" (ret), "=r"(*(u16 *)(8 + (char *)dst)) : "m" ((*(struct __large_struct *)((u16 *)(8 + (char *)src)))), "i" (2), "0" (ret))

                                 ;
  return ret;
 case 16:
  asm volatile("661:\n\t" ".byte " "0x66,0x66,0x90" "\n" "\n662:\n" ".pushsection .altinstructions,\"a\"\n" " .long 661b - .\n" " .long " "663""1""f - .\n" " .word " "( 9*32+20)" "\n" " .byte " "662b-661b" "\n" " .byte " "664""1""f-""663""1""f" "\n" ".popsection\n" ".pushsection .discard,\"aw\",@progbits\n" " .byte 0xff + (" "664""1""f-""663""1""f" ") - (" "662b-661b" ")\n" ".popsection\n" ".pushsection .altinstr_replacement, \"ax\"\n" "663""1"":\n\t" ".byte 0x0f,0x01,0xcb" "\n" "664""1" ":\n\t" ".popsection" "\n" "1:	mov""q"" %2,%""""1\n" "2: " "661:\n\t" ".byte " "0x66,0x66,0x90" "\n" "\n662:\n" ".pushsection .altinstructions,\"a\"\n" " .long 661b - .\n" " .long " "663""1""f - .\n" " .word " "( 9*32+20)" "\n" " .byte " "662b-661b" "\n" " .byte " "664""1""f-""663""1""f" "\n" ".popsection\n" ".pushsection .discard,\"aw\",@progbits\n" " .byte 0xff + (" "664""1""f-""663""1""f" ") - (" "662b-661b" ")\n" ".popsection\n" ".pushsection .altinstr_replacement, \"ax\"\n" "663""1"":\n\t" ".byte 0x0f,0x01,0xca" "\n" "664""1" ":\n\t" ".popsection" "\n" ".section .fixup,\"ax\"\n" "3:	mov %3,%0\n" "	xor""q"" %""""1,%""""1\n" "	jmp 2b\n" ".previous\n" " .pushsection \"__ex_table\",\"a\"\n" " .balign 8\n" " .long (" "1b" ") - .\n" " .long (" "3b" ") - .\n" " .popsection\n" : "=r" (ret), "=r"(*(u64 *)dst) : "m" ((*(struct __large_struct *)((u64 *)src))), "i" (16), "0" (ret))
                                 ;
  if (__builtin_expect(!!(ret), 0))
   return ret;
  asm volatile("661:\n\t" ".byte " "0x66,0x66,0x90" "\n" "\n662:\n" ".pushsection .altinstructions,\"a\"\n" " .long 661b - .\n" " .long " "663""1""f - .\n" " .word " "( 9*32+20)" "\n" " .byte " "662b-661b" "\n" " .byte " "664""1""f-""663""1""f" "\n" ".popsection\n" ".pushsection .discard,\"aw\",@progbits\n" " .byte 0xff + (" "664""1""f-""663""1""f" ") - (" "662b-661b" ")\n" ".popsection\n" ".pushsection .altinstr_replacement, \"ax\"\n" "663""1"":\n\t" ".byte 0x0f,0x01,0xcb" "\n" "664""1" ":\n\t" ".popsection" "\n" "1:	mov""q"" %2,%""""1\n" "2: " "661:\n\t" ".byte " "0x66,0x66,0x90" "\n" "\n662:\n" ".pushsection .altinstructions,\"a\"\n" " .long 661b - .\n" " .long " "663""1""f - .\n" " .word " "( 9*32+20)" "\n" " .byte " "662b-661b" "\n" " .byte " "664""1""f-""663""1""f" "\n" ".popsection\n" ".pushsection .discard,\"aw\",@progbits\n" " .byte 0xff + (" "664""1""f-""663""1""f" ") - (" "662b-661b" ")\n" ".popsection\n" ".pushsection .altinstr_replacement, \"ax\"\n" "663""1"":\n\t" ".byte 0x0f,0x01,0xca" "\n" "664""1" ":\n\t" ".popsection" "\n" ".section .fixup,\"ax\"\n" "3:	mov %3,%0\n" "	xor""q"" %""""1,%""""1\n" "	jmp 2b\n" ".previous\n" " .pushsection \"__ex_table\",\"a\"\n" " .balign 8\n" " .long (" "1b" ") - .\n" " .long (" "3b" ") - .\n" " .popsection\n" : "=r" (ret), "=r"(*(u64 *)(8 + (char *)dst)) : "m" ((*(struct __large_struct *)((u64 *)(8 + (char *)src)))), "i" (8), "0" (ret))

                                ;
  return ret;
 default:
  return copy_user_generic(dst, ( void *)src, size);
 }
}

static inline __attribute__((no_instrument_function)) __attribute__((always_inline))
int __copy_from_user(void *dst, const void *src, unsigned size)
{
 might_fault();
 return __copy_from_user_nocheck(dst, src, size);
}

static inline __attribute__((no_instrument_function)) __attribute__((always_inline))
int __copy_to_user_nocheck(void *dst, const void *src, unsigned size)
{
 int ret = 0;

 if (!__builtin_constant_p(size))
  return copy_user_generic(( void *)dst, src, size);
 switch (size) {
 case 1:asm volatile("661:\n\t" ".byte " "0x66,0x66,0x90" "\n" "\n662:\n" ".pushsection .altinstructions,\"a\"\n" " .long 661b - .\n" " .long " "663""1""f - .\n" " .word " "( 9*32+20)" "\n" " .byte " "662b-661b" "\n" " .byte " "664""1""f-""663""1""f" "\n" ".popsection\n" ".pushsection .discard,\"aw\",@progbits\n" " .byte 0xff + (" "664""1""f-""663""1""f" ") - (" "662b-661b" ")\n" ".popsection\n" ".pushsection .altinstr_replacement, \"ax\"\n" "663""1"":\n\t" ".byte 0x0f,0x01,0xcb" "\n" "664""1" ":\n\t" ".popsection" "\n" "1:	mov""b"" %""b""1,%2\n" "2: " "661:\n\t" ".byte " "0x66,0x66,0x90" "\n" "\n662:\n" ".pushsection .altinstructions,\"a\"\n" " .long 661b - .\n" " .long " "663""1""f - .\n" " .word " "( 9*32+20)" "\n" " .byte " "662b-661b" "\n" " .byte " "664""1""f-""663""1""f" "\n" ".popsection\n" ".pushsection .discard,\"aw\",@progbits\n" " .byte 0xff + (" "664""1""f-""663""1""f" ") - (" "662b-661b" ")\n" ".popsection\n" ".pushsection .altinstr_replacement, \"ax\"\n" "663""1"":\n\t" ".byte 0x0f,0x01,0xca" "\n" "664""1" ":\n\t" ".popsection" "\n" ".section .fixup,\"ax\"\n" "3:	mov %3,%0\n" "	jmp 2b\n" ".previous\n" " .pushsection \"__ex_table\",\"a\"\n" " .balign 8\n" " .long (" "1b" ") - .\n" " .long (" "3b" ") - .\n" " .popsection\n" : "=r"(ret) : "iq"(*(u8 *)src), "m" ((*(struct __large_struct *)((u8 *)dst))), "i" (1), "0" (ret))
                                ;
  return ret;
 case 2:asm volatile("661:\n\t" ".byte " "0x66,0x66,0x90" "\n" "\n662:\n" ".pushsection .altinstructions,\"a\"\n" " .long 661b - .\n" " .long " "663""1""f - .\n" " .word " "( 9*32+20)" "\n" " .byte " "662b-661b" "\n" " .byte " "664""1""f-""663""1""f" "\n" ".popsection\n" ".pushsection .discard,\"aw\",@progbits\n" " .byte 0xff + (" "664""1""f-""663""1""f" ") - (" "662b-661b" ")\n" ".popsection\n" ".pushsection .altinstr_replacement, \"ax\"\n" "663""1"":\n\t" ".byte 0x0f,0x01,0xcb" "\n" "664""1" ":\n\t" ".popsection" "\n" "1:	mov""w"" %""w""1,%2\n" "2: " "661:\n\t" ".byte " "0x66,0x66,0x90" "\n" "\n662:\n" ".pushsection .altinstructions,\"a\"\n" " .long 661b - .\n" " .long " "663""1""f - .\n" " .word " "( 9*32+20)" "\n" " .byte " "662b-661b" "\n" " .byte " "664""1""f-""663""1""f" "\n" ".popsection\n" ".pushsection .discard,\"aw\",@progbits\n" " .byte 0xff + (" "664""1""f-""663""1""f" ") - (" "662b-661b" ")\n" ".popsection\n" ".pushsection .altinstr_replacement, \"ax\"\n" "663""1"":\n\t" ".byte 0x0f,0x01,0xca" "\n" "664""1" ":\n\t" ".popsection" "\n" ".section .fixup,\"ax\"\n" "3:	mov %3,%0\n" "	jmp 2b\n" ".previous\n" " .pushsection \"__ex_table\",\"a\"\n" " .balign 8\n" " .long (" "1b" ") - .\n" " .long (" "3b" ") - .\n" " .popsection\n" : "=r"(ret) : "ir"(*(u16 *)src), "m" ((*(struct __large_struct *)((u16 *)dst))), "i" (2), "0" (ret))
                                ;
  return ret;
 case 4:asm volatile("661:\n\t" ".byte " "0x66,0x66,0x90" "\n" "\n662:\n" ".pushsection .altinstructions,\"a\"\n" " .long 661b - .\n" " .long " "663""1""f - .\n" " .word " "( 9*32+20)" "\n" " .byte " "662b-661b" "\n" " .byte " "664""1""f-""663""1""f" "\n" ".popsection\n" ".pushsection .discard,\"aw\",@progbits\n" " .byte 0xff + (" "664""1""f-""663""1""f" ") - (" "662b-661b" ")\n" ".popsection\n" ".pushsection .altinstr_replacement, \"ax\"\n" "663""1"":\n\t" ".byte 0x0f,0x01,0xcb" "\n" "664""1" ":\n\t" ".popsection" "\n" "1:	mov""l"" %""k""1,%2\n" "2: " "661:\n\t" ".byte " "0x66,0x66,0x90" "\n" "\n662:\n" ".pushsection .altinstructions,\"a\"\n" " .long 661b - .\n" " .long " "663""1""f - .\n" " .word " "( 9*32+20)" "\n" " .byte " "662b-661b" "\n" " .byte " "664""1""f-""663""1""f" "\n" ".popsection\n" ".pushsection .discard,\"aw\",@progbits\n" " .byte 0xff + (" "664""1""f-""663""1""f" ") - (" "662b-661b" ")\n" ".popsection\n" ".pushsection .altinstr_replacement, \"ax\"\n" "663""1"":\n\t" ".byte 0x0f,0x01,0xca" "\n" "664""1" ":\n\t" ".popsection" "\n" ".section .fixup,\"ax\"\n" "3:	mov %3,%0\n" "	jmp 2b\n" ".previous\n" " .pushsection \"__ex_table\",\"a\"\n" " .balign 8\n" " .long (" "1b" ") - .\n" " .long (" "3b" ") - .\n" " .popsection\n" : "=r"(ret) : "ir"(*(u32 *)src), "m" ((*(struct __large_struct *)((u32 *)dst))), "i" (4), "0" (ret))
                                ;
  return ret;
 case 8:asm volatile("661:\n\t" ".byte " "0x66,0x66,0x90" "\n" "\n662:\n" ".pushsection .altinstructions,\"a\"\n" " .long 661b - .\n" " .long " "663""1""f - .\n" " .word " "( 9*32+20)" "\n" " .byte " "662b-661b" "\n" " .byte " "664""1""f-""663""1""f" "\n" ".popsection\n" ".pushsection .discard,\"aw\",@progbits\n" " .byte 0xff + (" "664""1""f-""663""1""f" ") - (" "662b-661b" ")\n" ".popsection\n" ".pushsection .altinstr_replacement, \"ax\"\n" "663""1"":\n\t" ".byte 0x0f,0x01,0xcb" "\n" "664""1" ":\n\t" ".popsection" "\n" "1:	mov""q"" %""""1,%2\n" "2: " "661:\n\t" ".byte " "0x66,0x66,0x90" "\n" "\n662:\n" ".pushsection .altinstructions,\"a\"\n" " .long 661b - .\n" " .long " "663""1""f - .\n" " .word " "( 9*32+20)" "\n" " .byte " "662b-661b" "\n" " .byte " "664""1""f-""663""1""f" "\n" ".popsection\n" ".pushsection .discard,\"aw\",@progbits\n" " .byte 0xff + (" "664""1""f-""663""1""f" ") - (" "662b-661b" ")\n" ".popsection\n" ".pushsection .altinstr_replacement, \"ax\"\n" "663""1"":\n\t" ".byte 0x0f,0x01,0xca" "\n" "664""1" ":\n\t" ".popsection" "\n" ".section .fixup,\"ax\"\n" "3:	mov %3,%0\n" "	jmp 2b\n" ".previous\n" " .pushsection \"__ex_table\",\"a\"\n" " .balign 8\n" " .long (" "1b" ") - .\n" " .long (" "3b" ") - .\n" " .popsection\n" : "=r"(ret) : "er"(*(u64 *)src), "m" ((*(struct __large_struct *)((u64 *)dst))), "i" (8), "0" (ret))
                               ;
  return ret;
 case 10:
  asm volatile("661:\n\t" ".byte " "0x66,0x66,0x90" "\n" "\n662:\n" ".pushsection .altinstructions,\"a\"\n" " .long 661b - .\n" " .long " "663""1""f - .\n" " .word " "( 9*32+20)" "\n" " .byte " "662b-661b" "\n" " .byte " "664""1""f-""663""1""f" "\n" ".popsection\n" ".pushsection .discard,\"aw\",@progbits\n" " .byte 0xff + (" "664""1""f-""663""1""f" ") - (" "662b-661b" ")\n" ".popsection\n" ".pushsection .altinstr_replacement, \"ax\"\n" "663""1"":\n\t" ".byte 0x0f,0x01,0xcb" "\n" "664""1" ":\n\t" ".popsection" "\n" "1:	mov""q"" %""""1,%2\n" "2: " "661:\n\t" ".byte " "0x66,0x66,0x90" "\n" "\n662:\n" ".pushsection .altinstructions,\"a\"\n" " .long 661b - .\n" " .long " "663""1""f - .\n" " .word " "( 9*32+20)" "\n" " .byte " "662b-661b" "\n" " .byte " "664""1""f-""663""1""f" "\n" ".popsection\n" ".pushsection .discard,\"aw\",@progbits\n" " .byte 0xff + (" "664""1""f-""663""1""f" ") - (" "662b-661b" ")\n" ".popsection\n" ".pushsection .altinstr_replacement, \"ax\"\n" "663""1"":\n\t" ".byte 0x0f,0x01,0xca" "\n" "664""1" ":\n\t" ".popsection" "\n" ".section .fixup,\"ax\"\n" "3:	mov %3,%0\n" "	jmp 2b\n" ".previous\n" " .pushsection \"__ex_table\",\"a\"\n" " .balign 8\n" " .long (" "1b" ") - .\n" " .long (" "3b" ") - .\n" " .popsection\n" : "=r"(ret) : "er"(*(u64 *)src), "m" ((*(struct __large_struct *)((u64 *)dst))), "i" (10), "0" (ret))
                                 ;
  if (__builtin_expect(!!(ret), 0))
   return ret;
  asm("":::"memory");
  asm volatile("661:\n\t" ".byte " "0x66,0x66,0x90" "\n" "\n662:\n" ".pushsection .altinstructions,\"a\"\n" " .long 661b - .\n" " .long " "663""1""f - .\n" " .word " "( 9*32+20)" "\n" " .byte " "662b-661b" "\n" " .byte " "664""1""f-""663""1""f" "\n" ".popsection\n" ".pushsection .discard,\"aw\",@progbits\n" " .byte 0xff + (" "664""1""f-""663""1""f" ") - (" "662b-661b" ")\n" ".popsection\n" ".pushsection .altinstr_replacement, \"ax\"\n" "663""1"":\n\t" ".byte 0x0f,0x01,0xcb" "\n" "664""1" ":\n\t" ".popsection" "\n" "1:	mov""w"" %""w""1,%2\n" "2: " "661:\n\t" ".byte " "0x66,0x66,0x90" "\n" "\n662:\n" ".pushsection .altinstructions,\"a\"\n" " .long 661b - .\n" " .long " "663""1""f - .\n" " .word " "( 9*32+20)" "\n" " .byte " "662b-661b" "\n" " .byte " "664""1""f-""663""1""f" "\n" ".popsection\n" ".pushsection .discard,\"aw\",@progbits\n" " .byte 0xff + (" "664""1""f-""663""1""f" ") - (" "662b-661b" ")\n" ".popsection\n" ".pushsection .altinstr_replacement, \"ax\"\n" "663""1"":\n\t" ".byte 0x0f,0x01,0xca" "\n" "664""1" ":\n\t" ".popsection" "\n" ".section .fixup,\"ax\"\n" "3:	mov %3,%0\n" "	jmp 2b\n" ".previous\n" " .pushsection \"__ex_table\",\"a\"\n" " .balign 8\n" " .long (" "1b" ") - .\n" " .long (" "3b" ") - .\n" " .popsection\n" : "=r"(ret) : "ir"(4[(u16 *)src]), "m" ((*(struct __large_struct *)(4 + (u16 *)dst))), "i" (2), "0" (ret))
                                 ;
  return ret;
 case 16:
  asm volatile("661:\n\t" ".byte " "0x66,0x66,0x90" "\n" "\n662:\n" ".pushsection .altinstructions,\"a\"\n" " .long 661b - .\n" " .long " "663""1""f - .\n" " .word " "( 9*32+20)" "\n" " .byte " "662b-661b" "\n" " .byte " "664""1""f-""663""1""f" "\n" ".popsection\n" ".pushsection .discard,\"aw\",@progbits\n" " .byte 0xff + (" "664""1""f-""663""1""f" ") - (" "662b-661b" ")\n" ".popsection\n" ".pushsection .altinstr_replacement, \"ax\"\n" "663""1"":\n\t" ".byte 0x0f,0x01,0xcb" "\n" "664""1" ":\n\t" ".popsection" "\n" "1:	mov""q"" %""""1,%2\n" "2: " "661:\n\t" ".byte " "0x66,0x66,0x90" "\n" "\n662:\n" ".pushsection .altinstructions,\"a\"\n" " .long 661b - .\n" " .long " "663""1""f - .\n" " .word " "( 9*32+20)" "\n" " .byte " "662b-661b" "\n" " .byte " "664""1""f-""663""1""f" "\n" ".popsection\n" ".pushsection .discard,\"aw\",@progbits\n" " .byte 0xff + (" "664""1""f-""663""1""f" ") - (" "662b-661b" ")\n" ".popsection\n" ".pushsection .altinstr_replacement, \"ax\"\n" "663""1"":\n\t" ".byte 0x0f,0x01,0xca" "\n" "664""1" ":\n\t" ".popsection" "\n" ".section .fixup,\"ax\"\n" "3:	mov %3,%0\n" "	jmp 2b\n" ".previous\n" " .pushsection \"__ex_table\",\"a\"\n" " .balign 8\n" " .long (" "1b" ") - .\n" " .long (" "3b" ") - .\n" " .popsection\n" : "=r"(ret) : "er"(*(u64 *)src), "m" ((*(struct __large_struct *)((u64 *)dst))), "i" (16), "0" (ret))
                                 ;
  if (__builtin_expect(!!(ret), 0))
   return ret;
  asm("":::"memory");
  asm volatile("661:\n\t" ".byte " "0x66,0x66,0x90" "\n" "\n662:\n" ".pushsection .altinstructions,\"a\"\n" " .long 661b - .\n" " .long " "663""1""f - .\n" " .word " "( 9*32+20)" "\n" " .byte " "662b-661b" "\n" " .byte " "664""1""f-""663""1""f" "\n" ".popsection\n" ".pushsection .discard,\"aw\",@progbits\n" " .byte 0xff + (" "664""1""f-""663""1""f" ") - (" "662b-661b" ")\n" ".popsection\n" ".pushsection .altinstr_replacement, \"ax\"\n" "663""1"":\n\t" ".byte 0x0f,0x01,0xcb" "\n" "664""1" ":\n\t" ".popsection" "\n" "1:	mov""q"" %""""1,%2\n" "2: " "661:\n\t" ".byte " "0x66,0x66,0x90" "\n" "\n662:\n" ".pushsection .altinstructions,\"a\"\n" " .long 661b - .\n" " .long " "663""1""f - .\n" " .word " "( 9*32+20)" "\n" " .byte " "662b-661b" "\n" " .byte " "664""1""f-""663""1""f" "\n" ".popsection\n" ".pushsection .discard,\"aw\",@progbits\n" " .byte 0xff + (" "664""1""f-""663""1""f" ") - (" "662b-661b" ")\n" ".popsection\n" ".pushsection .altinstr_replacement, \"ax\"\n" "663""1"":\n\t" ".byte 0x0f,0x01,0xca" "\n" "664""1" ":\n\t" ".popsection" "\n" ".section .fixup,\"ax\"\n" "3:	mov %3,%0\n" "	jmp 2b\n" ".previous\n" " .pushsection \"__ex_table\",\"a\"\n" " .balign 8\n" " .long (" "1b" ") - .\n" " .long (" "3b" ") - .\n" " .popsection\n" : "=r"(ret) : "er"(1[(u64 *)src]), "m" ((*(struct __large_struct *)(1 + (u64 *)dst))), "i" (8), "0" (ret))
                                ;
  return ret;
 default:
  return copy_user_generic(( void *)dst, src, size);
 }
}

static inline __attribute__((no_instrument_function)) __attribute__((always_inline))
int __copy_to_user(void *dst, const void *src, unsigned size)
{
 might_fault();
 return __copy_to_user_nocheck(dst, src, size);
}

static inline __attribute__((no_instrument_function)) __attribute__((always_inline))
int __copy_in_user(void *dst, const void *src, unsigned size)
{
 int ret = 0;

 might_fault();
 if (!__builtin_constant_p(size))
  return copy_user_generic(( void *)dst,
      ( void *)src, size);
 switch (size) {
 case 1: {
  u8 tmp;
  asm volatile("661:\n\t" ".byte " "0x66,0x66,0x90" "\n" "\n662:\n" ".pushsection .altinstructions,\"a\"\n" " .long 661b - .\n" " .long " "663""1""f - .\n" " .word " "( 9*32+20)" "\n" " .byte " "662b-661b" "\n" " .byte " "664""1""f-""663""1""f" "\n" ".popsection\n" ".pushsection .discard,\"aw\",@progbits\n" " .byte 0xff + (" "664""1""f-""663""1""f" ") - (" "662b-661b" ")\n" ".popsection\n" ".pushsection .altinstr_replacement, \"ax\"\n" "663""1"":\n\t" ".byte 0x0f,0x01,0xcb" "\n" "664""1" ":\n\t" ".popsection" "\n" "1:	mov""b"" %2,%""b""1\n" "2: " "661:\n\t" ".byte " "0x66,0x66,0x90" "\n" "\n662:\n" ".pushsection .altinstructions,\"a\"\n" " .long 661b - .\n" " .long " "663""1""f - .\n" " .word " "( 9*32+20)" "\n" " .byte " "662b-661b" "\n" " .byte " "664""1""f-""663""1""f" "\n" ".popsection\n" ".pushsection .discard,\"aw\",@progbits\n" " .byte 0xff + (" "664""1""f-""663""1""f" ") - (" "662b-661b" ")\n" ".popsection\n" ".pushsection .altinstr_replacement, \"ax\"\n" "663""1"":\n\t" ".byte 0x0f,0x01,0xca" "\n" "664""1" ":\n\t" ".popsection" "\n" ".section .fixup,\"ax\"\n" "3:	mov %3,%0\n" "	xor""b"" %""b""1,%""b""1\n" "	jmp 2b\n" ".previous\n" " .pushsection \"__ex_table\",\"a\"\n" " .balign 8\n" " .long (" "1b" ") - .\n" " .long (" "3b" ") - .\n" " .popsection\n" : "=r" (ret), "=q"(tmp) : "m" ((*(struct __large_struct *)((u8 *)src))), "i" (1), "0" (ret))
                                 ;
  if (__builtin_expect(!!(!ret), 1))
   asm volatile("661:\n\t" ".byte " "0x66,0x66,0x90" "\n" "\n662:\n" ".pushsection .altinstructions,\"a\"\n" " .long 661b - .\n" " .long " "663""1""f - .\n" " .word " "( 9*32+20)" "\n" " .byte " "662b-661b" "\n" " .byte " "664""1""f-""663""1""f" "\n" ".popsection\n" ".pushsection .discard,\"aw\",@progbits\n" " .byte 0xff + (" "664""1""f-""663""1""f" ") - (" "662b-661b" ")\n" ".popsection\n" ".pushsection .altinstr_replacement, \"ax\"\n" "663""1"":\n\t" ".byte 0x0f,0x01,0xcb" "\n" "664""1" ":\n\t" ".popsection" "\n" "1:	mov""b"" %""b""1,%2\n" "2: " "661:\n\t" ".byte " "0x66,0x66,0x90" "\n" "\n662:\n" ".pushsection .altinstructions,\"a\"\n" " .long 661b - .\n" " .long " "663""1""f - .\n" " .word " "( 9*32+20)" "\n" " .byte " "662b-661b" "\n" " .byte " "664""1""f-""663""1""f" "\n" ".popsection\n" ".pushsection .discard,\"aw\",@progbits\n" " .byte 0xff + (" "664""1""f-""663""1""f" ") - (" "662b-661b" ")\n" ".popsection\n" ".pushsection .altinstr_replacement, \"ax\"\n" "663""1"":\n\t" ".byte 0x0f,0x01,0xca" "\n" "664""1" ":\n\t" ".popsection" "\n" ".section .fixup,\"ax\"\n" "3:	mov %3,%0\n" "	jmp 2b\n" ".previous\n" " .pushsection \"__ex_table\",\"a\"\n" " .balign 8\n" " .long (" "1b" ") - .\n" " .long (" "3b" ") - .\n" " .popsection\n" : "=r"(ret) : "iq"(tmp), "m" ((*(struct __large_struct *)((u8 *)dst))), "i" (1), "0" (ret))
                                  ;
  return ret;
 }
 case 2: {
  u16 tmp;
  asm volatile("661:\n\t" ".byte " "0x66,0x66,0x90" "\n" "\n662:\n" ".pushsection .altinstructions,\"a\"\n" " .long 661b - .\n" " .long " "663""1""f - .\n" " .word " "( 9*32+20)" "\n" " .byte " "662b-661b" "\n" " .byte " "664""1""f-""663""1""f" "\n" ".popsection\n" ".pushsection .discard,\"aw\",@progbits\n" " .byte 0xff + (" "664""1""f-""663""1""f" ") - (" "662b-661b" ")\n" ".popsection\n" ".pushsection .altinstr_replacement, \"ax\"\n" "663""1"":\n\t" ".byte 0x0f,0x01,0xcb" "\n" "664""1" ":\n\t" ".popsection" "\n" "1:	mov""w"" %2,%""w""1\n" "2: " "661:\n\t" ".byte " "0x66,0x66,0x90" "\n" "\n662:\n" ".pushsection .altinstructions,\"a\"\n" " .long 661b - .\n" " .long " "663""1""f - .\n" " .word " "( 9*32+20)" "\n" " .byte " "662b-661b" "\n" " .byte " "664""1""f-""663""1""f" "\n" ".popsection\n" ".pushsection .discard,\"aw\",@progbits\n" " .byte 0xff + (" "664""1""f-""663""1""f" ") - (" "662b-661b" ")\n" ".popsection\n" ".pushsection .altinstr_replacement, \"ax\"\n" "663""1"":\n\t" ".byte 0x0f,0x01,0xca" "\n" "664""1" ":\n\t" ".popsection" "\n" ".section .fixup,\"ax\"\n" "3:	mov %3,%0\n" "	xor""w"" %""w""1,%""w""1\n" "	jmp 2b\n" ".previous\n" " .pushsection \"__ex_table\",\"a\"\n" " .balign 8\n" " .long (" "1b" ") - .\n" " .long (" "3b" ") - .\n" " .popsection\n" : "=r" (ret), "=r"(tmp) : "m" ((*(struct __large_struct *)((u16 *)src))), "i" (2), "0" (ret))
                                 ;
  if (__builtin_expect(!!(!ret), 1))
   asm volatile("661:\n\t" ".byte " "0x66,0x66,0x90" "\n" "\n662:\n" ".pushsection .altinstructions,\"a\"\n" " .long 661b - .\n" " .long " "663""1""f - .\n" " .word " "( 9*32+20)" "\n" " .byte " "662b-661b" "\n" " .byte " "664""1""f-""663""1""f" "\n" ".popsection\n" ".pushsection .discard,\"aw\",@progbits\n" " .byte 0xff + (" "664""1""f-""663""1""f" ") - (" "662b-661b" ")\n" ".popsection\n" ".pushsection .altinstr_replacement, \"ax\"\n" "663""1"":\n\t" ".byte 0x0f,0x01,0xcb" "\n" "664""1" ":\n\t" ".popsection" "\n" "1:	mov""w"" %""w""1,%2\n" "2: " "661:\n\t" ".byte " "0x66,0x66,0x90" "\n" "\n662:\n" ".pushsection .altinstructions,\"a\"\n" " .long 661b - .\n" " .long " "663""1""f - .\n" " .word " "( 9*32+20)" "\n" " .byte " "662b-661b" "\n" " .byte " "664""1""f-""663""1""f" "\n" ".popsection\n" ".pushsection .discard,\"aw\",@progbits\n" " .byte 0xff + (" "664""1""f-""663""1""f" ") - (" "662b-661b" ")\n" ".popsection\n" ".pushsection .altinstr_replacement, \"ax\"\n" "663""1"":\n\t" ".byte 0x0f,0x01,0xca" "\n" "664""1" ":\n\t" ".popsection" "\n" ".section .fixup,\"ax\"\n" "3:	mov %3,%0\n" "	jmp 2b\n" ".previous\n" " .pushsection \"__ex_table\",\"a\"\n" " .balign 8\n" " .long (" "1b" ") - .\n" " .long (" "3b" ") - .\n" " .popsection\n" : "=r"(ret) : "ir"(tmp), "m" ((*(struct __large_struct *)((u16 *)dst))), "i" (2), "0" (ret))
                                  ;
  return ret;
 }

 case 4: {
  u32 tmp;
  asm volatile("661:\n\t" ".byte " "0x66,0x66,0x90" "\n" "\n662:\n" ".pushsection .altinstructions,\"a\"\n" " .long 661b - .\n" " .long " "663""1""f - .\n" " .word " "( 9*32+20)" "\n" " .byte " "662b-661b" "\n" " .byte " "664""1""f-""663""1""f" "\n" ".popsection\n" ".pushsection .discard,\"aw\",@progbits\n" " .byte 0xff + (" "664""1""f-""663""1""f" ") - (" "662b-661b" ")\n" ".popsection\n" ".pushsection .altinstr_replacement, \"ax\"\n" "663""1"":\n\t" ".byte 0x0f,0x01,0xcb" "\n" "664""1" ":\n\t" ".popsection" "\n" "1:	mov""l"" %2,%""k""1\n" "2: " "661:\n\t" ".byte " "0x66,0x66,0x90" "\n" "\n662:\n" ".pushsection .altinstructions,\"a\"\n" " .long 661b - .\n" " .long " "663""1""f - .\n" " .word " "( 9*32+20)" "\n" " .byte " "662b-661b" "\n" " .byte " "664""1""f-""663""1""f" "\n" ".popsection\n" ".pushsection .discard,\"aw\",@progbits\n" " .byte 0xff + (" "664""1""f-""663""1""f" ") - (" "662b-661b" ")\n" ".popsection\n" ".pushsection .altinstr_replacement, \"ax\"\n" "663""1"":\n\t" ".byte 0x0f,0x01,0xca" "\n" "664""1" ":\n\t" ".popsection" "\n" ".section .fixup,\"ax\"\n" "3:	mov %3,%0\n" "	xor""l"" %""k""1,%""k""1\n" "	jmp 2b\n" ".previous\n" " .pushsection \"__ex_table\",\"a\"\n" " .balign 8\n" " .long (" "1b" ") - .\n" " .long (" "3b" ") - .\n" " .popsection\n" : "=r" (ret), "=r"(tmp) : "m" ((*(struct __large_struct *)((u32 *)src))), "i" (4), "0" (ret))
                                 ;
  if (__builtin_expect(!!(!ret), 1))
   asm volatile("661:\n\t" ".byte " "0x66,0x66,0x90" "\n" "\n662:\n" ".pushsection .altinstructions,\"a\"\n" " .long 661b - .\n" " .long " "663""1""f - .\n" " .word " "( 9*32+20)" "\n" " .byte " "662b-661b" "\n" " .byte " "664""1""f-""663""1""f" "\n" ".popsection\n" ".pushsection .discard,\"aw\",@progbits\n" " .byte 0xff + (" "664""1""f-""663""1""f" ") - (" "662b-661b" ")\n" ".popsection\n" ".pushsection .altinstr_replacement, \"ax\"\n" "663""1"":\n\t" ".byte 0x0f,0x01,0xcb" "\n" "664""1" ":\n\t" ".popsection" "\n" "1:	mov""l"" %""k""1,%2\n" "2: " "661:\n\t" ".byte " "0x66,0x66,0x90" "\n" "\n662:\n" ".pushsection .altinstructions,\"a\"\n" " .long 661b - .\n" " .long " "663""1""f - .\n" " .word " "( 9*32+20)" "\n" " .byte " "662b-661b" "\n" " .byte " "664""1""f-""663""1""f" "\n" ".popsection\n" ".pushsection .discard,\"aw\",@progbits\n" " .byte 0xff + (" "664""1""f-""663""1""f" ") - (" "662b-661b" ")\n" ".popsection\n" ".pushsection .altinstr_replacement, \"ax\"\n" "663""1"":\n\t" ".byte 0x0f,0x01,0xca" "\n" "664""1" ":\n\t" ".popsection" "\n" ".section .fixup,\"ax\"\n" "3:	mov %3,%0\n" "	jmp 2b\n" ".previous\n" " .pushsection \"__ex_table\",\"a\"\n" " .balign 8\n" " .long (" "1b" ") - .\n" " .long (" "3b" ") - .\n" " .popsection\n" : "=r"(ret) : "ir"(tmp), "m" ((*(struct __large_struct *)((u32 *)dst))), "i" (4), "0" (ret))
                                  ;
  return ret;
 }
 case 8: {
  u64 tmp;
  asm volatile("661:\n\t" ".byte " "0x66,0x66,0x90" "\n" "\n662:\n" ".pushsection .altinstructions,\"a\"\n" " .long 661b - .\n" " .long " "663""1""f - .\n" " .word " "( 9*32+20)" "\n" " .byte " "662b-661b" "\n" " .byte " "664""1""f-""663""1""f" "\n" ".popsection\n" ".pushsection .discard,\"aw\",@progbits\n" " .byte 0xff + (" "664""1""f-""663""1""f" ") - (" "662b-661b" ")\n" ".popsection\n" ".pushsection .altinstr_replacement, \"ax\"\n" "663""1"":\n\t" ".byte 0x0f,0x01,0xcb" "\n" "664""1" ":\n\t" ".popsection" "\n" "1:	mov""q"" %2,%""""1\n" "2: " "661:\n\t" ".byte " "0x66,0x66,0x90" "\n" "\n662:\n" ".pushsection .altinstructions,\"a\"\n" " .long 661b - .\n" " .long " "663""1""f - .\n" " .word " "( 9*32+20)" "\n" " .byte " "662b-661b" "\n" " .byte " "664""1""f-""663""1""f" "\n" ".popsection\n" ".pushsection .discard,\"aw\",@progbits\n" " .byte 0xff + (" "664""1""f-""663""1""f" ") - (" "662b-661b" ")\n" ".popsection\n" ".pushsection .altinstr_replacement, \"ax\"\n" "663""1"":\n\t" ".byte 0x0f,0x01,0xca" "\n" "664""1" ":\n\t" ".popsection" "\n" ".section .fixup,\"ax\"\n" "3:	mov %3,%0\n" "	xor""q"" %""""1,%""""1\n" "	jmp 2b\n" ".previous\n" " .pushsection \"__ex_table\",\"a\"\n" " .balign 8\n" " .long (" "1b" ") - .\n" " .long (" "3b" ") - .\n" " .popsection\n" : "=r" (ret), "=r"(tmp) : "m" ((*(struct __large_struct *)((u64 *)src))), "i" (8), "0" (ret))
                                ;
  if (__builtin_expect(!!(!ret), 1))
   asm volatile("661:\n\t" ".byte " "0x66,0x66,0x90" "\n" "\n662:\n" ".pushsection .altinstructions,\"a\"\n" " .long 661b - .\n" " .long " "663""1""f - .\n" " .word " "( 9*32+20)" "\n" " .byte " "662b-661b" "\n" " .byte " "664""1""f-""663""1""f" "\n" ".popsection\n" ".pushsection .discard,\"aw\",@progbits\n" " .byte 0xff + (" "664""1""f-""663""1""f" ") - (" "662b-661b" ")\n" ".popsection\n" ".pushsection .altinstr_replacement, \"ax\"\n" "663""1"":\n\t" ".byte 0x0f,0x01,0xcb" "\n" "664""1" ":\n\t" ".popsection" "\n" "1:	mov""q"" %""""1,%2\n" "2: " "661:\n\t" ".byte " "0x66,0x66,0x90" "\n" "\n662:\n" ".pushsection .altinstructions,\"a\"\n" " .long 661b - .\n" " .long " "663""1""f - .\n" " .word " "( 9*32+20)" "\n" " .byte " "662b-661b" "\n" " .byte " "664""1""f-""663""1""f" "\n" ".popsection\n" ".pushsection .discard,\"aw\",@progbits\n" " .byte 0xff + (" "664""1""f-""663""1""f" ") - (" "662b-661b" ")\n" ".popsection\n" ".pushsection .altinstr_replacement, \"ax\"\n" "663""1"":\n\t" ".byte 0x0f,0x01,0xca" "\n" "664""1" ":\n\t" ".popsection" "\n" ".section .fixup,\"ax\"\n" "3:	mov %3,%0\n" "	jmp 2b\n" ".previous\n" " .pushsection \"__ex_table\",\"a\"\n" " .balign 8\n" " .long (" "1b" ") - .\n" " .long (" "3b" ") - .\n" " .popsection\n" : "=r"(ret) : "er"(tmp), "m" ((*(struct __large_struct *)((u64 *)dst))), "i" (8), "0" (ret))
                                 ;
  return ret;
 }
 default:
  return copy_user_generic(( void *)dst,
      ( void *)src, size);
 }
}

static inline __attribute__((no_instrument_function)) __attribute__((always_inline)) int
__copy_from_user_inatomic(void *dst, const void *src, unsigned size)
{
 return __copy_from_user_nocheck(dst, src, size);
}

static inline __attribute__((no_instrument_function)) __attribute__((always_inline)) int
__copy_to_user_inatomic(void *dst, const void *src, unsigned size)
{
 return __copy_to_user_nocheck(dst, src, size);
}

extern long __copy_user_nocache(void *dst, const void *src,
    unsigned size, int zerorest);

static inline __attribute__((no_instrument_function)) int
__copy_from_user_nocache(void *dst, const void *src, unsigned size)
{
 might_fault();
 return __copy_user_nocache(dst, src, size, 1);
}

static inline __attribute__((no_instrument_function)) int
__copy_from_user_inatomic_nocache(void *dst, const void *src,
      unsigned size)
{
 return __copy_user_nocache(dst, src, size, 0);
}

unsigned long
copy_user_handle_tail(char *to, char *from, unsigned len, unsigned zerorest);


unsigned long _copy_from_user(void *to, const void *from,
        unsigned n);
unsigned long _copy_to_user(void *to, const void *from,
      unsigned n);







extern void __attribute__((warning("copy_from_user() buffer size is too small")))
copy_from_user_overflow(void);
extern void __attribute__((warning("copy_to_user() buffer size is too small")))
copy_to_user_overflow(void) __asm__("copy_from_user_overflow");
static inline __attribute__((no_instrument_function)) void
__copy_from_user_overflow(int size, unsigned long count)
{
 ({ int __ret_warn_on = !!(1); if (__builtin_expect(!!(__ret_warn_on), 0)) warn_slowpath_fmt("./arch/x86/include/asm/uaccess.h", 680, "Buffer overflow detected (%d < %lu)!\n", size, count); __builtin_expect(!!(__ret_warn_on), 0); });
}





static inline __attribute__((no_instrument_function)) unsigned long
copy_from_user(void *to, const void *from, unsigned long n)
{
 int sz = -1;

 might_fault();
 if (__builtin_expect(!!(sz < 0 || sz >= n), 1))
  n = _copy_from_user(to, from, n);
 else if(__builtin_constant_p(n))
  copy_from_user_overflow();
 else
  __copy_from_user_overflow(sz, n);

 return n;
}

static inline __attribute__((no_instrument_function)) unsigned long
copy_to_user(void *to, const void *from, unsigned long n)
{
 int sz = -1;

 might_fault();


 if (__builtin_expect(!!(sz < 0 || sz >= n), 1))
  n = _copy_to_user(to, from, n);
 else if(__builtin_constant_p(n))
  copy_to_user_overflow();
 else
  __copy_from_user_overflow(sz, n);

 return n;
}

extern char __brk_base[], __brk_limit[];
extern struct exception_table_entry __stop___ex_table[];


extern char __end_rodata_hpage_align[];


extern void apic_timer_interrupt(void);
extern void x86_platform_ipi(void);
extern void kvm_posted_intr_ipi(void);
extern void error_interrupt(void);
extern void irq_work_interrupt(void);

extern void spurious_interrupt(void);
extern void thermal_interrupt(void);
extern void reschedule_interrupt(void);

extern void invalidate_interrupt(void);
extern void invalidate_interrupt0(void);
extern void invalidate_interrupt1(void);
extern void invalidate_interrupt2(void);
extern void invalidate_interrupt3(void);
extern void invalidate_interrupt4(void);
extern void invalidate_interrupt5(void);
extern void invalidate_interrupt6(void);
extern void invalidate_interrupt7(void);
extern void invalidate_interrupt8(void);
extern void invalidate_interrupt9(void);
extern void invalidate_interrupt10(void);
extern void invalidate_interrupt11(void);
extern void invalidate_interrupt12(void);
extern void invalidate_interrupt13(void);
extern void invalidate_interrupt14(void);
extern void invalidate_interrupt15(void);
extern void invalidate_interrupt16(void);
extern void invalidate_interrupt17(void);
extern void invalidate_interrupt18(void);
extern void invalidate_interrupt19(void);
extern void invalidate_interrupt20(void);
extern void invalidate_interrupt21(void);
extern void invalidate_interrupt22(void);
extern void invalidate_interrupt23(void);
extern void invalidate_interrupt24(void);
extern void invalidate_interrupt25(void);
extern void invalidate_interrupt26(void);
extern void invalidate_interrupt27(void);
extern void invalidate_interrupt28(void);
extern void invalidate_interrupt29(void);
extern void invalidate_interrupt30(void);
extern void invalidate_interrupt31(void);

extern void irq_move_cleanup_interrupt(void);
extern void reboot_interrupt(void);
extern void threshold_interrupt(void);

extern void call_function_interrupt(void);
extern void call_function_single_interrupt(void);



extern void trace_apic_timer_interrupt(void);
extern void trace_x86_platform_ipi(void);
extern void trace_error_interrupt(void);
extern void trace_irq_work_interrupt(void);
extern void trace_spurious_interrupt(void);
extern void trace_thermal_interrupt(void);
extern void trace_reschedule_interrupt(void);
extern void trace_threshold_interrupt(void);
extern void trace_call_function_interrupt(void);
extern void trace_call_function_single_interrupt(void);







extern unsigned long io_apic_irqs;

extern void setup_IO_APIC(void);
extern void disable_IO_APIC(void);

struct io_apic_irq_attr {
 int ioapic;
 int ioapic_pin;
 int trigger;
 int polarity;
};

static inline __attribute__((no_instrument_function)) void set_io_apic_irq_attr(struct io_apic_irq_attr *irq_attr,
     int ioapic, int ioapic_pin,
     int trigger, int polarity)
{
 irq_attr->ioapic = ioapic;
 irq_attr->ioapic_pin = ioapic_pin;
 irq_attr->trigger = trigger;
 irq_attr->polarity = polarity;
}


struct irq_2_iommu {
 struct intel_iommu *iommu;
 u16 irte_index;
 u16 sub_handle;
 u8 irte_mask;
};


struct irq_2_irte {
 u16 devid;
 u16 index;
};






struct irq_cfg {
 struct irq_pin_list *irq_2_pin;
 cpumask_var_t domain;
 cpumask_var_t old_domain;
 u8 vector;
 u8 move_in_progress : 1;

 u8 remapped : 1;
 union {
  struct irq_2_iommu irq_2_iommu;
  struct irq_2_irte irq_2_irte;
 };

};

extern int assign_irq_vector(int, struct irq_cfg *, const struct cpumask *);
extern void send_cleanup_vector(struct irq_cfg *);

struct irq_data;
int __ioapic_set_affinity(struct irq_data *, const struct cpumask *,
     unsigned int *dest_id);
extern int IO_APIC_get_PCI_irq_vector(int bus, int devfn, int pin, struct io_apic_irq_attr *irq_attr);
extern void setup_ioapic_dest(void);

extern void enable_IO_APIC(void);


extern atomic_t irq_err_count;
extern atomic_t irq_mis_count;


extern void eisa_set_level_irq(unsigned int irq);


extern __attribute__((externally_visible)) void smp_apic_timer_interrupt(struct pt_regs *);
extern __attribute__((externally_visible)) void smp_spurious_interrupt(struct pt_regs *);
extern __attribute__((externally_visible)) void smp_x86_platform_ipi(struct pt_regs *);
extern __attribute__((externally_visible)) void smp_error_interrupt(struct pt_regs *);

extern void smp_irq_move_cleanup_interrupt(void);


extern __attribute__((externally_visible)) void smp_reschedule_interrupt(struct pt_regs *);
extern __attribute__((externally_visible)) void smp_call_function_interrupt(struct pt_regs *);
extern __attribute__((externally_visible)) void smp_call_function_single_interrupt(struct pt_regs *);
extern __attribute__((externally_visible)) void smp_invalidate_interrupt(struct pt_regs *);


extern void (*__attribute__ ((__section__(".init.rodata"))) interrupt[256 -0x20])(void);







typedef int vector_irq_t[256];
extern __attribute__((section(".data..percpu" ""))) __typeof__(vector_irq_t) vector_irq;
extern void setup_vector_irq(int cpu);


extern void lock_vector_lock(void);
extern void unlock_vector_lock(void);
extern void __setup_vector_irq(int cpu);
struct irqaction;
extern int setup_irq(unsigned int irq, struct irqaction *new);
extern void remove_irq(unsigned int irq, struct irqaction *act);
extern int setup_percpu_irq(unsigned int irq, struct irqaction *new);
extern void remove_percpu_irq(unsigned int irq, struct irqaction *act);

extern void irq_cpu_online(void);
extern void irq_cpu_offline(void);
extern int irq_set_affinity_locked(struct irq_data *data,
       const struct cpumask *cpumask, bool force);


void irq_move_irq(struct irq_data *data);
void irq_move_masked_irq(struct irq_data *data);





extern int no_irq_affinity;




static inline __attribute__((no_instrument_function)) int irq_set_parent(int irq, int parent_irq)
{
 return 0;
}






extern void handle_level_irq(unsigned int irq, struct irq_desc *desc);
extern void handle_fasteoi_irq(unsigned int irq, struct irq_desc *desc);
extern void handle_edge_irq(unsigned int irq, struct irq_desc *desc);
extern void handle_edge_eoi_irq(unsigned int irq, struct irq_desc *desc);
extern void handle_simple_irq(unsigned int irq, struct irq_desc *desc);
extern void handle_percpu_irq(unsigned int irq, struct irq_desc *desc);
extern void handle_percpu_devid_irq(unsigned int irq, struct irq_desc *desc);
extern void handle_bad_irq(unsigned int irq, struct irq_desc *desc);
extern void handle_nested_irq(unsigned int irq);


extern void note_interrupt(unsigned int irq, struct irq_desc *desc,
      irqreturn_t action_ret);



extern int noirqdebug_setup(char *str);


extern int can_request_irq(unsigned int irq, unsigned long irqflags);


extern struct irq_chip no_irq_chip;
extern struct irq_chip dummy_irq_chip;

extern void
irq_set_chip_and_handler_name(unsigned int irq, struct irq_chip *chip,
         irq_flow_handler_t handle, const char *name);

static inline __attribute__((no_instrument_function)) void irq_set_chip_and_handler(unsigned int irq, struct irq_chip *chip,
         irq_flow_handler_t handle)
{
 irq_set_chip_and_handler_name(irq, chip, handle, ((void *)0));
}

extern int irq_set_percpu_devid(unsigned int irq);

extern void
__irq_set_handler(unsigned int irq, irq_flow_handler_t handle, int is_chained,
    const char *name);

static inline __attribute__((no_instrument_function)) void
irq_set_handler(unsigned int irq, irq_flow_handler_t handle)
{
 __irq_set_handler(irq, handle, 0, ((void *)0));
}






static inline __attribute__((no_instrument_function)) void
irq_set_chained_handler(unsigned int irq, irq_flow_handler_t handle)
{
 __irq_set_handler(irq, handle, 1, ((void *)0));
}

void irq_modify_status(unsigned int irq, unsigned long clr, unsigned long set);

static inline __attribute__((no_instrument_function)) void irq_set_status_flags(unsigned int irq, unsigned long set)
{
 irq_modify_status(irq, 0, set);
}

static inline __attribute__((no_instrument_function)) void irq_clear_status_flags(unsigned int irq, unsigned long clr)
{
 irq_modify_status(irq, clr, 0);
}

static inline __attribute__((no_instrument_function)) void irq_set_noprobe(unsigned int irq)
{
 irq_modify_status(irq, 0, IRQ_NOPROBE);
}

static inline __attribute__((no_instrument_function)) void irq_set_probe(unsigned int irq)
{
 irq_modify_status(irq, IRQ_NOPROBE, 0);
}

static inline __attribute__((no_instrument_function)) void irq_set_nothread(unsigned int irq)
{
 irq_modify_status(irq, 0, IRQ_NOTHREAD);
}

static inline __attribute__((no_instrument_function)) void irq_set_thread(unsigned int irq)
{
 irq_modify_status(irq, IRQ_NOTHREAD, 0);
}

static inline __attribute__((no_instrument_function)) void irq_set_nested_thread(unsigned int irq, bool nest)
{
 if (nest)
  irq_set_status_flags(irq, IRQ_NESTED_THREAD);
 else
  irq_clear_status_flags(irq, IRQ_NESTED_THREAD);
}

static inline __attribute__((no_instrument_function)) void irq_set_percpu_devid_flags(unsigned int irq)
{
 irq_set_status_flags(irq,
        IRQ_NOAUTOEN | IRQ_PER_CPU | IRQ_NOTHREAD |
        IRQ_NOPROBE | IRQ_PER_CPU_DEVID);
}


extern int irq_set_chip(unsigned int irq, struct irq_chip *chip);
extern int irq_set_handler_data(unsigned int irq, void *data);
extern int irq_set_chip_data(unsigned int irq, void *data);
extern int irq_set_irq_type(unsigned int irq, unsigned int type);
extern int irq_set_msi_desc(unsigned int irq, struct msi_desc *entry);
extern int irq_set_msi_desc_off(unsigned int irq_base, unsigned int irq_offset,
    struct msi_desc *entry);
extern struct irq_data *irq_get_irq_data(unsigned int irq);

static inline __attribute__((no_instrument_function)) struct irq_chip *irq_get_chip(unsigned int irq)
{
 struct irq_data *d = irq_get_irq_data(irq);
 return d ? d->chip : ((void *)0);
}

static inline __attribute__((no_instrument_function)) struct irq_chip *irq_data_get_irq_chip(struct irq_data *d)
{
 return d->chip;
}

static inline __attribute__((no_instrument_function)) void *irq_get_chip_data(unsigned int irq)
{
 struct irq_data *d = irq_get_irq_data(irq);
 return d ? d->chip_data : ((void *)0);
}

static inline __attribute__((no_instrument_function)) void *irq_data_get_irq_chip_data(struct irq_data *d)
{
 return d->chip_data;
}

static inline __attribute__((no_instrument_function)) void *irq_get_handler_data(unsigned int irq)
{
 struct irq_data *d = irq_get_irq_data(irq);
 return d ? d->handler_data : ((void *)0);
}

static inline __attribute__((no_instrument_function)) void *irq_data_get_irq_handler_data(struct irq_data *d)
{
 return d->handler_data;
}

static inline __attribute__((no_instrument_function)) struct msi_desc *irq_get_msi_desc(unsigned int irq)
{
 struct irq_data *d = irq_get_irq_data(irq);
 return d ? d->msi_desc : ((void *)0);
}

static inline __attribute__((no_instrument_function)) struct msi_desc *irq_data_get_msi(struct irq_data *d)
{
 return d->msi_desc;
}

static inline __attribute__((no_instrument_function)) u32 irq_get_trigger_type(unsigned int irq)
{
 struct irq_data *d = irq_get_irq_data(irq);
 return d ? irqd_get_trigger_type(d) : 0;
}

unsigned int arch_dynirq_lower_bound(unsigned int from);

int __irq_alloc_descs(int irq, unsigned int from, unsigned int cnt, int node,
  struct module *owner);
void irq_free_descs(unsigned int irq, unsigned int cnt);
static inline __attribute__((no_instrument_function)) void irq_free_desc(unsigned int irq)
{
 irq_free_descs(irq, 1);
}


unsigned int irq_alloc_hwirqs(int cnt, int node);
static inline __attribute__((no_instrument_function)) unsigned int irq_alloc_hwirq(int node)
{
 return irq_alloc_hwirqs(1, node);
}
void irq_free_hwirqs(unsigned int from, int cnt);
static inline __attribute__((no_instrument_function)) void irq_free_hwirq(unsigned int irq)
{
 return irq_free_hwirqs(irq, 1);
}
int arch_setup_hwirq(unsigned int irq, int node);
void arch_teardown_hwirq(unsigned int irq);
struct irq_chip_regs {
 unsigned long enable;
 unsigned long disable;
 unsigned long mask;
 unsigned long ack;
 unsigned long eoi;
 unsigned long type;
 unsigned long polarity;
};
struct irq_chip_type {
 struct irq_chip chip;
 struct irq_chip_regs regs;
 irq_flow_handler_t handler;
 u32 type;
 u32 mask_cache_priv;
 u32 *mask_cache;
};
struct irq_chip_generic {
 raw_spinlock_t lock;
 void *reg_base;
 unsigned int irq_base;
 unsigned int irq_cnt;
 u32 mask_cache;
 u32 type_cache;
 u32 polarity_cache;
 u32 wake_enabled;
 u32 wake_active;
 unsigned int num_ct;
 void *private;
 unsigned long installed;
 unsigned long unused;
 struct irq_domain *domain;
 struct list_head list;
 struct irq_chip_type chip_types[0];
};
enum irq_gc_flags {
 IRQ_GC_INIT_MASK_CACHE = 1 << 0,
 IRQ_GC_INIT_NESTED_LOCK = 1 << 1,
 IRQ_GC_MASK_CACHE_PER_TYPE = 1 << 2,
 IRQ_GC_NO_MASK = 1 << 3,
};
struct irq_domain_chip_generic {
 unsigned int irqs_per_chip;
 unsigned int num_chips;
 unsigned int irq_flags_to_clear;
 unsigned int irq_flags_to_set;
 enum irq_gc_flags gc_flags;
 struct irq_chip_generic *gc[0];
};


void irq_gc_noop(struct irq_data *d);
void irq_gc_mask_disable_reg(struct irq_data *d);
void irq_gc_mask_set_bit(struct irq_data *d);
void irq_gc_mask_clr_bit(struct irq_data *d);
void irq_gc_unmask_enable_reg(struct irq_data *d);
void irq_gc_ack_set_bit(struct irq_data *d);
void irq_gc_ack_clr_bit(struct irq_data *d);
void irq_gc_mask_disable_reg_and_ack(struct irq_data *d);
void irq_gc_eoi(struct irq_data *d);
int irq_gc_set_wake(struct irq_data *d, unsigned int on);


int irq_map_generic_chip(struct irq_domain *d, unsigned int virq,
    irq_hw_number_t hw_irq);
struct irq_chip_generic *
irq_alloc_generic_chip(const char *name, int nr_ct, unsigned int irq_base,
         void *reg_base, irq_flow_handler_t handler);
void irq_setup_generic_chip(struct irq_chip_generic *gc, u32 msk,
       enum irq_gc_flags flags, unsigned int clr,
       unsigned int set);
int irq_setup_alt_chip(struct irq_data *d, unsigned int type);
void irq_remove_generic_chip(struct irq_chip_generic *gc, u32 msk,
        unsigned int clr, unsigned int set);

struct irq_chip_generic *irq_get_domain_generic_chip(struct irq_domain *d, unsigned int hw_irq);
int irq_alloc_domain_generic_chips(struct irq_domain *d, int irqs_per_chip,
       int num_ct, const char *name,
       irq_flow_handler_t handler,
       unsigned int clr, unsigned int set,
       enum irq_gc_flags flags);


static inline __attribute__((no_instrument_function)) struct irq_chip_type *irq_data_get_chip_type(struct irq_data *d)
{
 return ({ const typeof( ((struct irq_chip_type *)0)->chip ) *__mptr = (d->chip); (struct irq_chip_type *)( (char *)__mptr - __builtin_offsetof(struct irq_chip_type,chip) );});
}




static inline __attribute__((no_instrument_function)) void irq_gc_lock(struct irq_chip_generic *gc)
{
 _raw_spin_lock(&gc->lock);
}

static inline __attribute__((no_instrument_function)) void irq_gc_unlock(struct irq_chip_generic *gc)
{
 __raw_spin_unlock(&gc->lock);
}

typedef struct {
 unsigned int __softirq_pending;
 unsigned int __nmi_count;

 unsigned int apic_timer_irqs;
 unsigned int irq_spurious_count;
 unsigned int icr_read_retry_count;


 unsigned int kvm_posted_intr_ipis;

 unsigned int x86_platform_ipis;
 unsigned int apic_perf_irqs;
 unsigned int apic_irq_work_irqs;

 unsigned int irq_resched_count;
 unsigned int irq_call_count;




 unsigned int irq_tlb_count;


 unsigned int irq_thermal_count;


 unsigned int irq_threshold_count;




} __attribute__((__aligned__((1 << (6))))) irq_cpustat_t;

extern __attribute__((section(".data..percpu" ""))) __typeof__(irq_cpustat_t) irq_stat __attribute__((__aligned__((1 << (6)))));
extern void ack_bad_irq(unsigned int irq);

extern u64 arch_irq_stat_cpu(unsigned int cpu);


extern u64 arch_irq_stat(void);


extern void synchronize_irq(unsigned int irq);
extern void synchronize_hardirq(unsigned int irq);
extern void rcu_nmi_enter(void);
extern void rcu_nmi_exit(void);
extern void irq_enter(void);
extern void irq_exit(void);








struct timerqueue_node {
 struct rb_node node;
 ktime_t expires;
};

struct timerqueue_head {
 struct rb_root head;
 struct timerqueue_node *next;
};


extern void timerqueue_add(struct timerqueue_head *head,
    struct timerqueue_node *node);
extern void timerqueue_del(struct timerqueue_head *head,
    struct timerqueue_node *node);
extern struct timerqueue_node *timerqueue_iterate_next(
      struct timerqueue_node *node);
static inline __attribute__((no_instrument_function))
struct timerqueue_node *timerqueue_getnext(struct timerqueue_head *head)
{
 return head->next;
}

static inline __attribute__((no_instrument_function)) void timerqueue_init(struct timerqueue_node *node)
{
 ((&node->node)->__rb_parent_color = (unsigned long)(&node->node));
}

static inline __attribute__((no_instrument_function)) void timerqueue_init_head(struct timerqueue_head *head)
{
 head->head = (struct rb_root) { ((void *)0), };
 head->next = ((void *)0);
}

struct hrtimer_clock_base;
struct hrtimer_cpu_base;




enum hrtimer_mode {
 HRTIMER_MODE_ABS = 0x0,
 HRTIMER_MODE_REL = 0x1,
 HRTIMER_MODE_PINNED = 0x02,
 HRTIMER_MODE_ABS_PINNED = 0x02,
 HRTIMER_MODE_REL_PINNED = 0x03,
};




enum hrtimer_restart {
 HRTIMER_NORESTART,
 HRTIMER_RESTART,
};
struct hrtimer {
 struct timerqueue_node node;
 ktime_t _softexpires;
 enum hrtimer_restart (*function)(struct hrtimer *);
 struct hrtimer_clock_base *base;
 unsigned long state;

 int start_pid;
 void *start_site;
 char start_comm[16];

};
struct hrtimer_sleeper {
 struct hrtimer timer;
 struct task_struct *task;
};
struct hrtimer_clock_base {
 struct hrtimer_cpu_base *cpu_base;
 int index;
 clockid_t clockid;
 struct timerqueue_head active;
 ktime_t resolution;
 ktime_t (*get_time)(void);
 ktime_t softirq_time;
 ktime_t offset;
};

enum hrtimer_base_type {
 HRTIMER_BASE_MONOTONIC,
 HRTIMER_BASE_REALTIME,
 HRTIMER_BASE_BOOTTIME,
 HRTIMER_BASE_TAI,
 HRTIMER_MAX_CLOCK_BASES,
};
struct hrtimer_cpu_base {
 raw_spinlock_t lock;
 unsigned int cpu;
 unsigned int active_bases;
 unsigned int clock_was_set;

 ktime_t expires_next;
 int hres_active;
 int hang_detected;
 unsigned long nr_events;
 unsigned long nr_retries;
 unsigned long nr_hangs;
 ktime_t max_hang_time;

 struct hrtimer_clock_base clock_base[HRTIMER_MAX_CLOCK_BASES];
};

static inline __attribute__((no_instrument_function)) void hrtimer_set_expires(struct hrtimer *timer, ktime_t time)
{
 timer->node.expires = time;
 timer->_softexpires = time;
}

static inline __attribute__((no_instrument_function)) void hrtimer_set_expires_range(struct hrtimer *timer, ktime_t time, ktime_t delta)
{
 timer->_softexpires = time;
 timer->node.expires = ktime_add_safe(time, delta);
}

static inline __attribute__((no_instrument_function)) void hrtimer_set_expires_range_ns(struct hrtimer *timer, ktime_t time, unsigned long delta)
{
 timer->_softexpires = time;
 timer->node.expires = ktime_add_safe(time, ns_to_ktime(delta));
}

static inline __attribute__((no_instrument_function)) void hrtimer_set_expires_tv64(struct hrtimer *timer, s64 tv64)
{
 timer->node.expires.tv64 = tv64;
 timer->_softexpires.tv64 = tv64;
}

static inline __attribute__((no_instrument_function)) void hrtimer_add_expires(struct hrtimer *timer, ktime_t time)
{
 timer->node.expires = ktime_add_safe(timer->node.expires, time);
 timer->_softexpires = ktime_add_safe(timer->_softexpires, time);
}

static inline __attribute__((no_instrument_function)) void hrtimer_add_expires_ns(struct hrtimer *timer, u64 ns)
{
 timer->node.expires = ({ (ktime_t){ .tv64 = (timer->node.expires).tv64 + (ns) }; });
 timer->_softexpires = ({ (ktime_t){ .tv64 = (timer->_softexpires).tv64 + (ns) }; });
}

static inline __attribute__((no_instrument_function)) ktime_t hrtimer_get_expires(const struct hrtimer *timer)
{
 return timer->node.expires;
}

static inline __attribute__((no_instrument_function)) ktime_t hrtimer_get_softexpires(const struct hrtimer *timer)
{
 return timer->_softexpires;
}

static inline __attribute__((no_instrument_function)) s64 hrtimer_get_expires_tv64(const struct hrtimer *timer)
{
 return timer->node.expires.tv64;
}
static inline __attribute__((no_instrument_function)) s64 hrtimer_get_softexpires_tv64(const struct hrtimer *timer)
{
 return timer->_softexpires.tv64;
}

static inline __attribute__((no_instrument_function)) s64 hrtimer_get_expires_ns(const struct hrtimer *timer)
{
 return ((timer->node.expires).tv64);
}

static inline __attribute__((no_instrument_function)) ktime_t hrtimer_expires_remaining(const struct hrtimer *timer)
{
 return ({ (ktime_t){ .tv64 = (timer->node.expires).tv64 - (timer->base->get_time()).tv64 }; });
}


struct clock_event_device;

extern void hrtimer_interrupt(struct clock_event_device *dev);




static inline __attribute__((no_instrument_function)) ktime_t hrtimer_cb_get_time(struct hrtimer *timer)
{
 return timer->base->get_time();
}

static inline __attribute__((no_instrument_function)) int hrtimer_is_hres_active(struct hrtimer *timer)
{
 return timer->base->cpu_base->hres_active;
}

extern void hrtimer_peek_ahead_timers(void);
extern void clock_was_set_delayed(void);
extern void clock_was_set(void);

extern void timerfd_clock_was_set(void);



extern void hrtimers_resume(void);

extern __attribute__((section(".data..percpu" ""))) __typeof__(struct tick_device) tick_cpu_device;





extern void hrtimer_init(struct hrtimer *timer, clockid_t which_clock,
    enum hrtimer_mode mode);







static inline __attribute__((no_instrument_function)) void hrtimer_init_on_stack(struct hrtimer *timer,
      clockid_t which_clock,
      enum hrtimer_mode mode)
{
 hrtimer_init(timer, which_clock, mode);
}
static inline __attribute__((no_instrument_function)) void destroy_hrtimer_on_stack(struct hrtimer *timer) { }



extern int hrtimer_start(struct hrtimer *timer, ktime_t tim,
    const enum hrtimer_mode mode);
extern int hrtimer_start_range_ns(struct hrtimer *timer, ktime_t tim,
   unsigned long range_ns, const enum hrtimer_mode mode);
extern int
__hrtimer_start_range_ns(struct hrtimer *timer, ktime_t tim,
    unsigned long delta_ns,
    const enum hrtimer_mode mode, int wakeup);

extern int hrtimer_cancel(struct hrtimer *timer);
extern int hrtimer_try_to_cancel(struct hrtimer *timer);

static inline __attribute__((no_instrument_function)) int hrtimer_start_expires(struct hrtimer *timer,
      enum hrtimer_mode mode)
{
 unsigned long delta;
 ktime_t soft, hard;
 soft = hrtimer_get_softexpires(timer);
 hard = hrtimer_get_expires(timer);
 delta = ((({ (ktime_t){ .tv64 = (hard).tv64 - (soft).tv64 }; })).tv64);
 return hrtimer_start_range_ns(timer, soft, delta, mode);
}

static inline __attribute__((no_instrument_function)) int hrtimer_restart(struct hrtimer *timer)
{
 return hrtimer_start_expires(timer, HRTIMER_MODE_ABS);
}


extern ktime_t hrtimer_get_remaining(const struct hrtimer *timer);
extern int hrtimer_get_res(const clockid_t which_clock, struct timespec *tp);

extern ktime_t hrtimer_get_next_event(void);






static inline __attribute__((no_instrument_function)) int hrtimer_active(const struct hrtimer *timer)
{
 return timer->state != 0x00;
}




static inline __attribute__((no_instrument_function)) int hrtimer_is_queued(struct hrtimer *timer)
{
 return timer->state & 0x01;
}





static inline __attribute__((no_instrument_function)) int hrtimer_callback_running(struct hrtimer *timer)
{
 return timer->state & 0x02;
}


extern u64
hrtimer_forward(struct hrtimer *timer, ktime_t now, ktime_t interval);


static inline __attribute__((no_instrument_function)) u64 hrtimer_forward_now(struct hrtimer *timer,
          ktime_t interval)
{
 return hrtimer_forward(timer, timer->base->get_time(), interval);
}


extern long hrtimer_nanosleep(struct timespec *rqtp,
         struct timespec *rmtp,
         const enum hrtimer_mode mode,
         const clockid_t clockid);
extern long hrtimer_nanosleep_restart(struct restart_block *restart_block);

extern void hrtimer_init_sleeper(struct hrtimer_sleeper *sl,
     struct task_struct *tsk);

extern int schedule_hrtimeout_range(ktime_t *expires, unsigned long delta,
      const enum hrtimer_mode mode);
extern int schedule_hrtimeout_range_clock(ktime_t *expires,
  unsigned long delta, const enum hrtimer_mode mode, int clock);
extern int schedule_hrtimeout(ktime_t *expires, const enum hrtimer_mode mode);


extern void hrtimer_run_queues(void);
extern void hrtimer_run_pending(void);


extern void __attribute__ ((__section__(".init.text"))) __attribute__((__cold__)) __attribute__((no_instrument_function)) hrtimers_init(void);


extern void sysrq_timer_list_show(void);
enum {
 IRQC_IS_HARDIRQ = 0,
 IRQC_IS_NESTED,
};

typedef irqreturn_t (*irq_handler_t)(int, void *);
struct irqaction {
 irq_handler_t handler;
 void *dev_id;
 void *percpu_dev_id;
 struct irqaction *next;
 irq_handler_t thread_fn;
 struct task_struct *thread;
 unsigned int irq;
 unsigned int flags;
 unsigned long thread_flags;
 unsigned long thread_mask;
 const char *name;
 struct proc_dir_entry *dir;
} __attribute__((__aligned__(1 << (6))));

extern irqreturn_t no_action(int cpl, void *dev_id);

extern int
request_threaded_irq(unsigned int irq, irq_handler_t handler,
       irq_handler_t thread_fn,
       unsigned long flags, const char *name, void *dev);

static inline __attribute__((no_instrument_function)) int
request_irq(unsigned int irq, irq_handler_t handler, unsigned long flags,
     const char *name, void *dev)
{
 return request_threaded_irq(irq, handler, ((void *)0), flags, name, dev);
}

extern int
request_any_context_irq(unsigned int irq, irq_handler_t handler,
   unsigned long flags, const char *name, void *dev_id);

extern int
request_percpu_irq(unsigned int irq, irq_handler_t handler,
     const char *devname, void *percpu_dev_id);

extern void free_irq(unsigned int, void *);
extern void free_percpu_irq(unsigned int, void *);

struct device;

extern int
devm_request_threaded_irq(struct device *dev, unsigned int irq,
     irq_handler_t handler, irq_handler_t thread_fn,
     unsigned long irqflags, const char *devname,
     void *dev_id);

static inline __attribute__((no_instrument_function)) int
devm_request_irq(struct device *dev, unsigned int irq, irq_handler_t handler,
   unsigned long irqflags, const char *devname, void *dev_id)
{
 return devm_request_threaded_irq(dev, irq, handler, ((void *)0), irqflags,
      devname, dev_id);
}

extern int
devm_request_any_context_irq(struct device *dev, unsigned int irq,
   irq_handler_t handler, unsigned long irqflags,
   const char *devname, void *dev_id);

extern void devm_free_irq(struct device *dev, unsigned int irq, void *dev_id);
extern void disable_irq_nosync(unsigned int irq);
extern void disable_irq(unsigned int irq);
extern void disable_percpu_irq(unsigned int irq);
extern void enable_irq(unsigned int irq);
extern void enable_percpu_irq(unsigned int irq, unsigned int type);
extern void irq_wake_thread(unsigned int irq, void *dev_id);


extern void suspend_device_irqs(void);
extern void resume_device_irqs(void);

extern int check_wakeup_irqs(void);
struct irq_affinity_notify {
 unsigned int irq;
 struct kref kref;
 struct work_struct work;
 void (*notify)(struct irq_affinity_notify *, const cpumask_t *mask);
 void (*release)(struct kref *ref);
};



extern cpumask_var_t irq_default_affinity;


extern int __irq_set_affinity(unsigned int irq, const struct cpumask *cpumask,
         bool force);
static inline __attribute__((no_instrument_function)) int
irq_set_affinity(unsigned int irq, const struct cpumask *cpumask)
{
 return __irq_set_affinity(irq, cpumask, false);
}
static inline __attribute__((no_instrument_function)) int
irq_force_affinity(unsigned int irq, const struct cpumask *cpumask)
{
 return __irq_set_affinity(irq, cpumask, true);
}

extern int irq_can_set_affinity(unsigned int irq);
extern int irq_select_affinity(unsigned int irq);

extern int irq_set_affinity_hint(unsigned int irq, const struct cpumask *m);

extern int
irq_set_affinity_notifier(unsigned int irq, struct irq_affinity_notify *notify);
static inline __attribute__((no_instrument_function)) void disable_irq_nosync_lockdep(unsigned int irq)
{
 disable_irq_nosync(irq);



}

static inline __attribute__((no_instrument_function)) void disable_irq_nosync_lockdep_irqsave(unsigned int irq, unsigned long *flags)
{
 disable_irq_nosync(irq);



}

static inline __attribute__((no_instrument_function)) void disable_irq_lockdep(unsigned int irq)
{
 disable_irq(irq);



}

static inline __attribute__((no_instrument_function)) void enable_irq_lockdep(unsigned int irq)
{



 enable_irq(irq);
}

static inline __attribute__((no_instrument_function)) void enable_irq_lockdep_irqrestore(unsigned int irq, unsigned long *flags)
{



 enable_irq(irq);
}


extern int irq_set_irq_wake(unsigned int irq, unsigned int on);

static inline __attribute__((no_instrument_function)) int enable_irq_wake(unsigned int irq)
{
 return irq_set_irq_wake(irq, 1);
}

static inline __attribute__((no_instrument_function)) int disable_irq_wake(unsigned int irq)
{
 return irq_set_irq_wake(irq, 0);
}



extern bool force_irqthreads;
enum
{
 HI_SOFTIRQ=0,
 TIMER_SOFTIRQ,
 NET_TX_SOFTIRQ,
 NET_RX_SOFTIRQ,
 BLOCK_SOFTIRQ,
 BLOCK_IOPOLL_SOFTIRQ,
 TASKLET_SOFTIRQ,
 SCHED_SOFTIRQ,
 HRTIMER_SOFTIRQ,
 RCU_SOFTIRQ,

 NR_SOFTIRQS
};






extern const char * const softirq_to_name[NR_SOFTIRQS];





struct softirq_action
{
 void (*action)(struct softirq_action *);
};

 void do_softirq(void);
 void __do_softirq(void);


void do_softirq_own_stack(void);







extern void open_softirq(int nr, void (*action)(struct softirq_action *));
extern void softirq_init(void);
extern void __raise_softirq_irqoff(unsigned int nr);

extern void raise_softirq_irqoff(unsigned int nr);
extern void raise_softirq(unsigned int nr);

extern __attribute__((section(".data..percpu" ""))) __typeof__(struct task_struct *) ksoftirqd;

static inline __attribute__((no_instrument_function)) struct task_struct *this_cpu_ksoftirqd(void)
{
 return ({ typeof(ksoftirqd) pscr_ret__; do { const void *__vpp_verify = (typeof((&(ksoftirqd)) + 0))((void *)0); (void)__vpp_verify; } while (0); switch(sizeof(ksoftirqd)) { case 1: pscr_ret__ = ({ typeof((ksoftirqd)) pfo_ret__; switch (sizeof((ksoftirqd))) { case 1: asm("mov" "b ""%%""gs"":" "%P" "1"",%0" : "=q" (pfo_ret__) : "m"(ksoftirqd)); break; case 2: asm("mov" "w ""%%""gs"":" "%P" "1"",%0" : "=r" (pfo_ret__) : "m"(ksoftirqd)); break; case 4: asm("mov" "l ""%%""gs"":" "%P" "1"",%0" : "=r" (pfo_ret__) : "m"(ksoftirqd)); break; case 8: asm("mov" "q ""%%""gs"":" "%P" "1"",%0" : "=r" (pfo_ret__) : "m"(ksoftirqd)); break; default: __bad_percpu_size(); } pfo_ret__; }); break; case 2: pscr_ret__ = ({ typeof((ksoftirqd)) pfo_ret__; switch (sizeof((ksoftirqd))) { case 1: asm("mov" "b ""%%""gs"":" "%P" "1"",%0" : "=q" (pfo_ret__) : "m"(ksoftirqd)); break; case 2: asm("mov" "w ""%%""gs"":" "%P" "1"",%0" : "=r" (pfo_ret__) : "m"(ksoftirqd)); break; case 4: asm("mov" "l ""%%""gs"":" "%P" "1"",%0" : "=r" (pfo_ret__) : "m"(ksoftirqd)); break; case 8: asm("mov" "q ""%%""gs"":" "%P" "1"",%0" : "=r" (pfo_ret__) : "m"(ksoftirqd)); break; default: __bad_percpu_size(); } pfo_ret__; }); break; case 4: pscr_ret__ = ({ typeof((ksoftirqd)) pfo_ret__; switch (sizeof((ksoftirqd))) { case 1: asm("mov" "b ""%%""gs"":" "%P" "1"",%0" : "=q" (pfo_ret__) : "m"(ksoftirqd)); break; case 2: asm("mov" "w ""%%""gs"":" "%P" "1"",%0" : "=r" (pfo_ret__) : "m"(ksoftirqd)); break; case 4: asm("mov" "l ""%%""gs"":" "%P" "1"",%0" : "=r" (pfo_ret__) : "m"(ksoftirqd)); break; case 8: asm("mov" "q ""%%""gs"":" "%P" "1"",%0" : "=r" (pfo_ret__) : "m"(ksoftirqd)); break; default: __bad_percpu_size(); } pfo_ret__; }); break; case 8: pscr_ret__ = ({ typeof((ksoftirqd)) pfo_ret__; switch (sizeof((ksoftirqd))) { case 1: asm("mov" "b ""%%""gs"":" "%P" "1"",%0" : "=q" (pfo_ret__) : "m"(ksoftirqd)); break; case 2: asm("mov" "w ""%%""gs"":" "%P" "1"",%0" : "=r" (pfo_ret__) : "m"(ksoftirqd)); break; case 4: asm("mov" "l ""%%""gs"":" "%P" "1"",%0" : "=r" (pfo_ret__) : "m"(ksoftirqd)); break; case 8: asm("mov" "q ""%%""gs"":" "%P" "1"",%0" : "=r" (pfo_ret__) : "m"(ksoftirqd)); break; default: __bad_percpu_size(); } pfo_ret__; }); break; default: __bad_size_call_parameter(); break; } pscr_ret__; });
}
struct tasklet_struct
{
 struct tasklet_struct *next;
 unsigned long state;
 atomic_t count;
 void (*func)(unsigned long);
 unsigned long data;
};
enum
{
 TASKLET_STATE_SCHED,
 TASKLET_STATE_RUN
};


static inline __attribute__((no_instrument_function)) int tasklet_trylock(struct tasklet_struct *t)
{
 return !test_and_set_bit(TASKLET_STATE_RUN, &(t)->state);
}

static inline __attribute__((no_instrument_function)) void tasklet_unlock(struct tasklet_struct *t)
{
 __asm__ __volatile__("": : :"memory");
 clear_bit(TASKLET_STATE_RUN, &(t)->state);
}

static inline __attribute__((no_instrument_function)) void tasklet_unlock_wait(struct tasklet_struct *t)
{
 while ((__builtin_constant_p((TASKLET_STATE_RUN)) ? constant_test_bit((TASKLET_STATE_RUN), (&(t)->state)) : variable_test_bit((TASKLET_STATE_RUN), (&(t)->state)))) { __asm__ __volatile__("": : :"memory"); }
}






extern void __tasklet_schedule(struct tasklet_struct *t);

static inline __attribute__((no_instrument_function)) void tasklet_schedule(struct tasklet_struct *t)
{
 if (!test_and_set_bit(TASKLET_STATE_SCHED, &t->state))
  __tasklet_schedule(t);
}

extern void __tasklet_hi_schedule(struct tasklet_struct *t);

static inline __attribute__((no_instrument_function)) void tasklet_hi_schedule(struct tasklet_struct *t)
{
 if (!test_and_set_bit(TASKLET_STATE_SCHED, &t->state))
  __tasklet_hi_schedule(t);
}

extern void __tasklet_hi_schedule_first(struct tasklet_struct *t);







static inline __attribute__((no_instrument_function)) void tasklet_hi_schedule_first(struct tasklet_struct *t)
{
 if (!test_and_set_bit(TASKLET_STATE_SCHED, &t->state))
  __tasklet_hi_schedule_first(t);
}


static inline __attribute__((no_instrument_function)) void tasklet_disable_nosync(struct tasklet_struct *t)
{
 atomic_inc(&t->count);
 __asm__ __volatile__("": : :"memory");
}

static inline __attribute__((no_instrument_function)) void tasklet_disable(struct tasklet_struct *t)
{
 tasklet_disable_nosync(t);
 tasklet_unlock_wait(t);
 asm volatile("mfence":::"memory");
}

static inline __attribute__((no_instrument_function)) void tasklet_enable(struct tasklet_struct *t)
{
 __asm__ __volatile__("": : :"memory");
 atomic_dec(&t->count);
}

static inline __attribute__((no_instrument_function)) void tasklet_hi_enable(struct tasklet_struct *t)
{
 __asm__ __volatile__("": : :"memory");
 atomic_dec(&t->count);
}

extern void tasklet_kill(struct tasklet_struct *t);
extern void tasklet_kill_immediate(struct tasklet_struct *t, unsigned int cpu);
extern void tasklet_init(struct tasklet_struct *t,
    void (*func)(unsigned long), unsigned long data);

struct tasklet_hrtimer {
 struct hrtimer timer;
 struct tasklet_struct tasklet;
 enum hrtimer_restart (*function)(struct hrtimer *);
};

extern void
tasklet_hrtimer_init(struct tasklet_hrtimer *ttimer,
       enum hrtimer_restart (*function)(struct hrtimer *),
       clockid_t which_clock, enum hrtimer_mode mode);

static inline __attribute__((no_instrument_function))
int tasklet_hrtimer_start(struct tasklet_hrtimer *ttimer, ktime_t time,
     const enum hrtimer_mode mode)
{
 return hrtimer_start(&ttimer->timer, time, mode);
}

static inline __attribute__((no_instrument_function))
void tasklet_hrtimer_cancel(struct tasklet_hrtimer *ttimer)
{
 hrtimer_cancel(&ttimer->timer);
 tasklet_kill(&ttimer->tasklet);
}
extern unsigned long probe_irq_on(void);
extern int probe_irq_off(unsigned long);
extern unsigned int probe_irq_mask(unsigned long);




extern void init_irq_proc(void);






struct seq_file;
int show_interrupts(struct seq_file *p, void *v);
int arch_show_interrupts(struct seq_file *p, int prec);

extern int early_irq_init(void);
extern int arch_probe_nr_irqs(void);
extern int arch_early_irq_init(void);
struct input_event {
 struct timeval time;
 __u16 type;
 __u16 code;
 __s32 value;
};
struct input_id {
 __u16 bustype;
 __u16 vendor;
 __u16 product;
 __u16 version;
};
struct input_absinfo {
 __s32 value;
 __s32 minimum;
 __s32 maximum;
 __s32 fuzz;
 __s32 flat;
 __s32 resolution;
};
struct input_keymap_entry {

 __u8 flags;
 __u8 len;
 __u16 index;
 __u32 keycode;
 __u8 scancode[32];
};
struct ff_replay {
 __u16 length;
 __u16 delay;
};






struct ff_trigger {
 __u16 button;
 __u16 interval;
};
struct ff_envelope {
 __u16 attack_length;
 __u16 attack_level;
 __u16 fade_length;
 __u16 fade_level;
};






struct ff_constant_effect {
 __s16 level;
 struct ff_envelope envelope;
};







struct ff_ramp_effect {
 __s16 start_level;
 __s16 end_level;
 struct ff_envelope envelope;
};
struct ff_condition_effect {
 __u16 right_saturation;
 __u16 left_saturation;

 __s16 right_coeff;
 __s16 left_coeff;

 __u16 deadband;
 __s16 center;
};
struct ff_periodic_effect {
 __u16 waveform;
 __u16 period;
 __s16 magnitude;
 __s16 offset;
 __u16 phase;

 struct ff_envelope envelope;

 __u32 custom_len;
 __s16 *custom_data;
};
struct ff_rumble_effect {
 __u16 strong_magnitude;
 __u16 weak_magnitude;
};
struct ff_effect {
 __u16 type;
 __s16 id;
 __u16 direction;
 struct ff_trigger trigger;
 struct ff_replay replay;

 union {
  struct ff_constant_effect constant;
  struct ff_ramp_effect ramp;
  struct ff_periodic_effect periodic;
  struct ff_condition_effect condition[2];
  struct ff_rumble_effect rumble;
 } u;
};
struct klist_node;
struct klist {
 spinlock_t k_lock;
 struct list_head k_list;
 void (*get)(struct klist_node *);
 void (*put)(struct klist_node *);
} __attribute__ ((aligned (sizeof(void *))));
extern void klist_init(struct klist *k, void (*get)(struct klist_node *),
         void (*put)(struct klist_node *));

struct klist_node {
 void *n_klist;
 struct list_head n_node;
 struct kref n_ref;
};

extern void klist_add_tail(struct klist_node *n, struct klist *k);
extern void klist_add_head(struct klist_node *n, struct klist *k);
extern void klist_add_behind(struct klist_node *n, struct klist_node *pos);
extern void klist_add_before(struct klist_node *n, struct klist_node *pos);

extern void klist_del(struct klist_node *n);
extern void klist_remove(struct klist_node *n);

extern int klist_node_attached(struct klist_node *n);


struct klist_iter {
 struct klist *i_klist;
 struct klist_node *i_cur;
};


extern void klist_iter_init(struct klist *k, struct klist_iter *i);
extern void klist_iter_init_node(struct klist *k, struct klist_iter *i,
     struct klist_node *n);
extern void klist_iter_exit(struct klist_iter *i);
extern struct klist_node *klist_next(struct klist_iter *i);





static inline __attribute__((no_instrument_function)) int pinctrl_bind_pins(struct device *dev)
{
 return 0;
}


struct ratelimit_state {
 raw_spinlock_t lock;

 int interval;
 int burst;
 int printed;
 int missed;
 unsigned long begin;
};
static inline __attribute__((no_instrument_function)) void ratelimit_state_init(struct ratelimit_state *rs,
     int interval, int burst)
{
 do { *(&rs->lock) = (raw_spinlock_t) { .raw_lock = { { 0 } }, }; } while (0);
 rs->interval = interval;
 rs->burst = burst;
 rs->printed = 0;
 rs->missed = 0;
 rs->begin = 0;
}

extern struct ratelimit_state printk_ratelimit_state;

extern int ___ratelimit(struct ratelimit_state *rs, const char *func);





struct dev_archdata {

 struct dma_map_ops *dma_ops;


 void *iommu;

};

struct pdev_archdata {
};

struct device;
struct device_private;
struct device_driver;
struct driver_private;
struct module;
struct class;
struct subsys_private;
struct bus_type;
struct device_node;
struct iommu_ops;
struct iommu_group;

struct bus_attribute {
 struct attribute attr;
 ssize_t (*show)(struct bus_type *bus, char *buf);
 ssize_t (*store)(struct bus_type *bus, const char *buf, size_t count);
};
extern int bus_create_file(struct bus_type *,
     struct bus_attribute *);
extern void bus_remove_file(struct bus_type *, struct bus_attribute *);
struct bus_type {
 const char *name;
 const char *dev_name;
 struct device *dev_root;
 struct device_attribute *dev_attrs;
 const struct attribute_group **bus_groups;
 const struct attribute_group **dev_groups;
 const struct attribute_group **drv_groups;

 int (*match)(struct device *dev, struct device_driver *drv);
 int (*uevent)(struct device *dev, struct kobj_uevent_env *env);
 int (*probe)(struct device *dev);
 int (*remove)(struct device *dev);
 void (*shutdown)(struct device *dev);

 int (*online)(struct device *dev);
 int (*offline)(struct device *dev);

 int (*suspend)(struct device *dev, pm_message_t state);
 int (*resume)(struct device *dev);

 const struct dev_pm_ops *pm;

 const struct iommu_ops *iommu_ops;

 struct subsys_private *p;
 struct lock_class_key lock_key;
};

extern int bus_register(struct bus_type *bus);

extern void bus_unregister(struct bus_type *bus);

extern int bus_rescan_devices(struct bus_type *bus);


struct subsys_dev_iter {
 struct klist_iter ki;
 const struct device_type *type;
};
void subsys_dev_iter_init(struct subsys_dev_iter *iter,
    struct bus_type *subsys,
    struct device *start,
    const struct device_type *type);
struct device *subsys_dev_iter_next(struct subsys_dev_iter *iter);
void subsys_dev_iter_exit(struct subsys_dev_iter *iter);

int bus_for_each_dev(struct bus_type *bus, struct device *start, void *data,
       int (*fn)(struct device *dev, void *data));
struct device *bus_find_device(struct bus_type *bus, struct device *start,
          void *data,
          int (*match)(struct device *dev, void *data));
struct device *bus_find_device_by_name(struct bus_type *bus,
           struct device *start,
           const char *name);
struct device *subsys_find_device_by_id(struct bus_type *bus, unsigned int id,
     struct device *hint);
int bus_for_each_drv(struct bus_type *bus, struct device_driver *start,
       void *data, int (*fn)(struct device_driver *, void *));
void bus_sort_breadthfirst(struct bus_type *bus,
      int (*compare)(const struct device *a,
       const struct device *b));






struct notifier_block;

extern int bus_register_notifier(struct bus_type *bus,
     struct notifier_block *nb);
extern int bus_unregister_notifier(struct bus_type *bus,
       struct notifier_block *nb);
extern struct kset *bus_get_kset(struct bus_type *bus);
extern struct klist *bus_get_device_klist(struct bus_type *bus);
struct device_driver {
 const char *name;
 struct bus_type *bus;

 struct module *owner;
 const char *mod_name;

 bool suppress_bind_attrs;

 const struct of_device_id *of_match_table;
 const struct acpi_device_id *acpi_match_table;

 int (*probe) (struct device *dev);
 int (*remove) (struct device *dev);
 void (*shutdown) (struct device *dev);
 int (*suspend) (struct device *dev, pm_message_t state);
 int (*resume) (struct device *dev);
 const struct attribute_group **groups;

 const struct dev_pm_ops *pm;

 struct driver_private *p;
};


extern int driver_register(struct device_driver *drv);
extern void driver_unregister(struct device_driver *drv);

extern struct device_driver *driver_find(const char *name,
      struct bus_type *bus);
extern int driver_probe_done(void);
extern void wait_for_device_probe(void);




struct driver_attribute {
 struct attribute attr;
 ssize_t (*show)(struct device_driver *driver, char *buf);
 ssize_t (*store)(struct device_driver *driver, const char *buf,
    size_t count);
};
extern int driver_create_file(struct device_driver *driver,
     const struct driver_attribute *attr);
extern void driver_remove_file(struct device_driver *driver,
          const struct driver_attribute *attr);

extern int driver_for_each_device(struct device_driver *drv,
            struct device *start,
            void *data,
            int (*fn)(struct device *dev,
        void *));
struct device *driver_find_device(struct device_driver *drv,
      struct device *start, void *data,
      int (*match)(struct device *dev, void *data));
struct subsys_interface {
 const char *name;
 struct bus_type *subsys;
 struct list_head node;
 int (*add_dev)(struct device *dev, struct subsys_interface *sif);
 int (*remove_dev)(struct device *dev, struct subsys_interface *sif);
};

int subsys_interface_register(struct subsys_interface *sif);
void subsys_interface_unregister(struct subsys_interface *sif);

int subsys_system_register(struct bus_type *subsys,
      const struct attribute_group **groups);
int subsys_virtual_register(struct bus_type *subsys,
       const struct attribute_group **groups);
struct class {
 const char *name;
 struct module *owner;

 struct class_attribute *class_attrs;
 const struct attribute_group **dev_groups;
 struct kobject *dev_kobj;

 int (*dev_uevent)(struct device *dev, struct kobj_uevent_env *env);
 char *(*devnode)(struct device *dev, umode_t *mode);

 void (*class_release)(struct class *class);
 void (*dev_release)(struct device *dev);

 int (*suspend)(struct device *dev, pm_message_t state);
 int (*resume)(struct device *dev);

 const struct kobj_ns_type_operations *ns_type;
 const void *(*namespace)(struct device *dev);

 const struct dev_pm_ops *pm;

 struct subsys_private *p;
};

struct class_dev_iter {
 struct klist_iter ki;
 const struct device_type *type;
};

extern struct kobject *sysfs_dev_block_kobj;
extern struct kobject *sysfs_dev_char_kobj;
extern int __class_register(struct class *class,
      struct lock_class_key *key);
extern void class_unregister(struct class *class);
struct class_compat;
struct class_compat *class_compat_register(const char *name);
void class_compat_unregister(struct class_compat *cls);
int class_compat_create_link(struct class_compat *cls, struct device *dev,
        struct device *device_link);
void class_compat_remove_link(struct class_compat *cls, struct device *dev,
         struct device *device_link);

extern void class_dev_iter_init(struct class_dev_iter *iter,
    struct class *class,
    struct device *start,
    const struct device_type *type);
extern struct device *class_dev_iter_next(struct class_dev_iter *iter);
extern void class_dev_iter_exit(struct class_dev_iter *iter);

extern int class_for_each_device(struct class *class, struct device *start,
     void *data,
     int (*fn)(struct device *dev, void *data));
extern struct device *class_find_device(struct class *class,
     struct device *start, const void *data,
     int (*match)(struct device *, const void *));

struct class_attribute {
 struct attribute attr;
 ssize_t (*show)(struct class *class, struct class_attribute *attr,
   char *buf);
 ssize_t (*store)(struct class *class, struct class_attribute *attr,
   const char *buf, size_t count);
};
extern int class_create_file_ns(struct class *class,
          const struct class_attribute *attr,
          const void *ns);
extern void class_remove_file_ns(struct class *class,
     const struct class_attribute *attr,
     const void *ns);

static inline __attribute__((no_instrument_function)) int class_create_file(struct class *class,
     const struct class_attribute *attr)
{
 return class_create_file_ns(class, attr, ((void *)0));
}

static inline __attribute__((no_instrument_function)) void class_remove_file(struct class *class,
         const struct class_attribute *attr)
{
 return class_remove_file_ns(class, attr, ((void *)0));
}


struct class_attribute_string {
 struct class_attribute attr;
 char *str;
};
extern ssize_t show_class_attr_string(struct class *class, struct class_attribute *attr,
                        char *buf);

struct class_interface {
 struct list_head node;
 struct class *class;

 int (*add_dev) (struct device *, struct class_interface *);
 void (*remove_dev) (struct device *, struct class_interface *);
};

extern int class_interface_register(struct class_interface *);
extern void class_interface_unregister(struct class_interface *);

extern struct class * __class_create(struct module *owner,
        const char *name,
        struct lock_class_key *key);
extern void class_destroy(struct class *cls);
struct device_type {
 const char *name;
 const struct attribute_group **groups;
 int (*uevent)(struct device *dev, struct kobj_uevent_env *env);
 char *(*devnode)(struct device *dev, umode_t *mode,
    kuid_t *uid, kgid_t *gid);
 void (*release)(struct device *dev);

 const struct dev_pm_ops *pm;
};


struct device_attribute {
 struct attribute attr;
 ssize_t (*show)(struct device *dev, struct device_attribute *attr,
   char *buf);
 ssize_t (*store)(struct device *dev, struct device_attribute *attr,
    const char *buf, size_t count);
};

struct dev_ext_attribute {
 struct device_attribute attr;
 void *var;
};

ssize_t device_show_ulong(struct device *dev, struct device_attribute *attr,
     char *buf);
ssize_t device_store_ulong(struct device *dev, struct device_attribute *attr,
      const char *buf, size_t count);
ssize_t device_show_int(struct device *dev, struct device_attribute *attr,
   char *buf);
ssize_t device_store_int(struct device *dev, struct device_attribute *attr,
    const char *buf, size_t count);
ssize_t device_show_bool(struct device *dev, struct device_attribute *attr,
   char *buf);
ssize_t device_store_bool(struct device *dev, struct device_attribute *attr,
    const char *buf, size_t count);
extern int device_create_file(struct device *device,
         const struct device_attribute *entry);
extern void device_remove_file(struct device *dev,
          const struct device_attribute *attr);
extern bool device_remove_file_self(struct device *dev,
        const struct device_attribute *attr);
extern int device_create_bin_file(struct device *dev,
     const struct bin_attribute *attr);
extern void device_remove_bin_file(struct device *dev,
       const struct bin_attribute *attr);


typedef void (*dr_release_t)(struct device *dev, void *res);
typedef int (*dr_match_t)(struct device *dev, void *res, void *match_data);







extern void *devres_alloc(dr_release_t release, size_t size, gfp_t gfp);

extern void devres_for_each_res(struct device *dev, dr_release_t release,
    dr_match_t match, void *match_data,
    void (*fn)(struct device *, void *, void *),
    void *data);
extern void devres_free(void *res);
extern void devres_add(struct device *dev, void *res);
extern void *devres_find(struct device *dev, dr_release_t release,
    dr_match_t match, void *match_data);
extern void *devres_get(struct device *dev, void *new_res,
   dr_match_t match, void *match_data);
extern void *devres_remove(struct device *dev, dr_release_t release,
      dr_match_t match, void *match_data);
extern int devres_destroy(struct device *dev, dr_release_t release,
     dr_match_t match, void *match_data);
extern int devres_release(struct device *dev, dr_release_t release,
     dr_match_t match, void *match_data);


extern void * devres_open_group(struct device *dev, void *id,
          gfp_t gfp);
extern void devres_close_group(struct device *dev, void *id);
extern void devres_remove_group(struct device *dev, void *id);
extern int devres_release_group(struct device *dev, void *id);


extern void *devm_kmalloc(struct device *dev, size_t size, gfp_t gfp);
extern char *devm_kvasprintf(struct device *dev, gfp_t gfp, const char *fmt,
        va_list ap);
extern char *devm_kasprintf(struct device *dev, gfp_t gfp,
       const char *fmt, ...);
static inline __attribute__((no_instrument_function)) void *devm_kzalloc(struct device *dev, size_t size, gfp_t gfp)
{
 return devm_kmalloc(dev, size, gfp | (( gfp_t)0x8000u));
}
static inline __attribute__((no_instrument_function)) void *devm_kmalloc_array(struct device *dev,
           size_t n, size_t size, gfp_t flags)
{
 if (size != 0 && n > (~(size_t)0) / size)
  return ((void *)0);
 return devm_kmalloc(dev, n * size, flags);
}
static inline __attribute__((no_instrument_function)) void *devm_kcalloc(struct device *dev,
     size_t n, size_t size, gfp_t flags)
{
 return devm_kmalloc_array(dev, n, size, flags | (( gfp_t)0x8000u));
}
extern void devm_kfree(struct device *dev, void *p);
extern char *devm_kstrdup(struct device *dev, const char *s, gfp_t gfp);
extern void *devm_kmemdup(struct device *dev, const void *src, size_t len,
     gfp_t gfp);

extern unsigned long devm_get_free_pages(struct device *dev,
      gfp_t gfp_mask, unsigned int order);
extern void devm_free_pages(struct device *dev, unsigned long addr);

void *devm_ioremap_resource(struct device *dev, struct resource *res);


int devm_add_action(struct device *dev, void (*action)(void *), void *data);
void devm_remove_action(struct device *dev, void (*action)(void *), void *data);

struct device_dma_parameters {




 unsigned int max_segment_size;
 unsigned long segment_boundary_mask;
};

struct acpi_device;

struct acpi_dev_node {

 struct acpi_device *companion;

};
struct device {
 struct device *parent;

 struct device_private *p;

 struct kobject kobj;
 const char *init_name;
 const struct device_type *type;

 struct mutex mutex;



 struct bus_type *bus;
 struct device_driver *driver;

 void *platform_data;

 void *driver_data;

 struct dev_pm_info power;
 struct dev_pm_domain *pm_domain;






 int numa_node;

 u64 *dma_mask;
 u64 coherent_dma_mask;




 unsigned long dma_pfn_offset;

 struct device_dma_parameters *dma_parms;

 struct list_head dma_pools;

 struct dma_coherent_mem *dma_mem;






 struct dev_archdata archdata;

 struct device_node *of_node;
 struct acpi_dev_node acpi_node;

 dev_t devt;
 u32 id;

 spinlock_t devres_lock;
 struct list_head devres_head;

 struct klist_node knode_class;
 struct class *class;
 const struct attribute_group **groups;

 void (*release)(struct device *dev);
 struct iommu_group *iommu_group;

 bool offline_disabled:1;
 bool offline:1;
};

static inline __attribute__((no_instrument_function)) struct device *kobj_to_dev(struct kobject *kobj)
{
 return ({ const typeof( ((struct device *)0)->kobj ) *__mptr = (kobj); (struct device *)( (char *)__mptr - __builtin_offsetof(struct device,kobj) );});
}


struct wakeup_source {
 const char *name;
 struct list_head entry;
 spinlock_t lock;
 struct timer_list timer;
 unsigned long timer_expires;
 ktime_t total_time;
 ktime_t max_time;
 ktime_t last_time;
 ktime_t start_prevent_time;
 ktime_t prevent_sleep_time;
 unsigned long event_count;
 unsigned long active_count;
 unsigned long relax_count;
 unsigned long expire_count;
 unsigned long wakeup_count;
 bool active:1;
 bool autosleep_enabled:1;
};







static inline __attribute__((no_instrument_function)) bool device_can_wakeup(struct device *dev)
{
 return dev->power.can_wakeup;
}

static inline __attribute__((no_instrument_function)) bool device_may_wakeup(struct device *dev)
{
 return dev->power.can_wakeup && !!dev->power.wakeup;
}


extern void wakeup_source_prepare(struct wakeup_source *ws, const char *name);
extern struct wakeup_source *wakeup_source_create(const char *name);
extern void wakeup_source_drop(struct wakeup_source *ws);
extern void wakeup_source_destroy(struct wakeup_source *ws);
extern void wakeup_source_add(struct wakeup_source *ws);
extern void wakeup_source_remove(struct wakeup_source *ws);
extern struct wakeup_source *wakeup_source_register(const char *name);
extern void wakeup_source_unregister(struct wakeup_source *ws);
extern int device_wakeup_enable(struct device *dev);
extern int device_wakeup_disable(struct device *dev);
extern void device_set_wakeup_capable(struct device *dev, bool capable);
extern int device_init_wakeup(struct device *dev, bool val);
extern int device_set_wakeup_enable(struct device *dev, bool enable);
extern void __pm_stay_awake(struct wakeup_source *ws);
extern void pm_stay_awake(struct device *dev);
extern void __pm_relax(struct wakeup_source *ws);
extern void pm_relax(struct device *dev);
extern void __pm_wakeup_event(struct wakeup_source *ws, unsigned int msec);
extern void pm_wakeup_event(struct device *dev, unsigned int msec);
static inline __attribute__((no_instrument_function)) void wakeup_source_init(struct wakeup_source *ws,
          const char *name)
{
 wakeup_source_prepare(ws, name);
 wakeup_source_add(ws);
}

static inline __attribute__((no_instrument_function)) void wakeup_source_trash(struct wakeup_source *ws)
{
 wakeup_source_remove(ws);
 wakeup_source_drop(ws);
}

static inline __attribute__((no_instrument_function)) const char *dev_name(const struct device *dev)
{

 if (dev->init_name)
  return dev->init_name;

 return kobject_name(&dev->kobj);
}

extern __attribute__((format(printf, 2, 3)))
int dev_set_name(struct device *dev, const char *name, ...);


static inline __attribute__((no_instrument_function)) int dev_to_node(struct device *dev)
{
 return dev->numa_node;
}
static inline __attribute__((no_instrument_function)) void set_dev_node(struct device *dev, int node)
{
 dev->numa_node = node;
}
static inline __attribute__((no_instrument_function)) void *dev_get_drvdata(const struct device *dev)
{
 return dev->driver_data;
}

static inline __attribute__((no_instrument_function)) void dev_set_drvdata(struct device *dev, void *data)
{
 dev->driver_data = data;
}

static inline __attribute__((no_instrument_function)) struct pm_subsys_data *dev_to_psd(struct device *dev)
{
 return dev ? dev->power.subsys_data : ((void *)0);
}

static inline __attribute__((no_instrument_function)) unsigned int dev_get_uevent_suppress(const struct device *dev)
{
 return dev->kobj.uevent_suppress;
}

static inline __attribute__((no_instrument_function)) void dev_set_uevent_suppress(struct device *dev, int val)
{
 dev->kobj.uevent_suppress = val;
}

static inline __attribute__((no_instrument_function)) int device_is_registered(struct device *dev)
{
 return dev->kobj.state_in_sysfs;
}

static inline __attribute__((no_instrument_function)) void device_enable_async_suspend(struct device *dev)
{
 if (!dev->power.is_prepared)
  dev->power.async_suspend = true;
}

static inline __attribute__((no_instrument_function)) void device_disable_async_suspend(struct device *dev)
{
 if (!dev->power.is_prepared)
  dev->power.async_suspend = false;
}

static inline __attribute__((no_instrument_function)) bool device_async_suspend_enabled(struct device *dev)
{
 return !!dev->power.async_suspend;
}

static inline __attribute__((no_instrument_function)) void pm_suspend_ignore_children(struct device *dev, bool enable)
{
 dev->power.ignore_children = enable;
}

static inline __attribute__((no_instrument_function)) void dev_pm_syscore_device(struct device *dev, bool val)
{

 dev->power.syscore = val;

}

static inline __attribute__((no_instrument_function)) void device_lock(struct device *dev)
{
 mutex_lock(&dev->mutex);
}

static inline __attribute__((no_instrument_function)) int device_trylock(struct device *dev)
{
 return mutex_trylock(&dev->mutex);
}

static inline __attribute__((no_instrument_function)) void device_unlock(struct device *dev)
{
 mutex_unlock(&dev->mutex);
}

void driver_init(void);




extern int device_register(struct device *dev);
extern void device_unregister(struct device *dev);
extern void device_initialize(struct device *dev);
extern int device_add(struct device *dev);
extern void device_del(struct device *dev);
extern int device_for_each_child(struct device *dev, void *data,
       int (*fn)(struct device *dev, void *data));
extern struct device *device_find_child(struct device *dev, void *data,
    int (*match)(struct device *dev, void *data));
extern int device_rename(struct device *dev, const char *new_name);
extern int device_move(struct device *dev, struct device *new_parent,
         enum dpm_order dpm_order);
extern const char *device_get_devnode(struct device *dev,
          umode_t *mode, kuid_t *uid, kgid_t *gid,
          const char **tmp);

static inline __attribute__((no_instrument_function)) bool device_supports_offline(struct device *dev)
{
 return dev->bus && dev->bus->offline && dev->bus->online;
}

extern void lock_device_hotplug(void);
extern void unlock_device_hotplug(void);
extern int lock_device_hotplug_sysfs(void);
extern int device_offline(struct device *dev);
extern int device_online(struct device *dev);



extern struct device *__root_device_register(const char *name,
          struct module *owner);





extern void root_device_unregister(struct device *root);

static inline __attribute__((no_instrument_function)) void *dev_get_platdata(const struct device *dev)
{
 return dev->platform_data;
}





extern int device_bind_driver(struct device *dev);
extern void device_release_driver(struct device *dev);
extern int device_attach(struct device *dev);
extern int driver_attach(struct device_driver *drv);
extern int device_reprobe(struct device *dev);




extern struct device *device_create_vargs(struct class *cls,
       struct device *parent,
       dev_t devt,
       void *drvdata,
       const char *fmt,
       va_list vargs);
extern __attribute__((format(printf, 5, 6)))
struct device *device_create(struct class *cls, struct device *parent,
        dev_t devt, void *drvdata,
        const char *fmt, ...);
extern __attribute__((format(printf, 6, 7)))
struct device *device_create_with_groups(struct class *cls,
        struct device *parent, dev_t devt, void *drvdata,
        const struct attribute_group **groups,
        const char *fmt, ...);
extern void device_destroy(struct class *cls, dev_t devt);







extern int (*platform_notify)(struct device *dev);

extern int (*platform_notify_remove)(struct device *dev);






extern struct device *get_device(struct device *dev);
extern void put_device(struct device *dev);


extern int devtmpfs_create_node(struct device *dev);
extern int devtmpfs_delete_node(struct device *dev);
extern int devtmpfs_mount(const char *mntdir);







extern void device_shutdown(void);


extern const char *dev_driver_string(const struct device *dev);




extern __attribute__((format(printf, 3, 0)))
int dev_vprintk_emit(int level, const struct device *dev,
       const char *fmt, va_list args);
extern __attribute__((format(printf, 3, 4)))
int dev_printk_emit(int level, const struct device *dev, const char *fmt, ...);

extern __attribute__((format(printf, 3, 4)))
int dev_printk(const char *level, const struct device *dev,
        const char *fmt, ...);
extern __attribute__((format(printf, 2, 3)))
int dev_emerg(const struct device *dev, const char *fmt, ...);
extern __attribute__((format(printf, 2, 3)))
int dev_alert(const struct device *dev, const char *fmt, ...);
extern __attribute__((format(printf, 2, 3)))
int dev_crit(const struct device *dev, const char *fmt, ...);
extern __attribute__((format(printf, 2, 3)))
int dev_err(const struct device *dev, const char *fmt, ...);
extern __attribute__((format(printf, 2, 3)))
int dev_warn(const struct device *dev, const char *fmt, ...);
extern __attribute__((format(printf, 2, 3)))
int dev_notice(const struct device *dev, const char *fmt, ...);
extern __attribute__((format(printf, 2, 3)))
int _dev_info(const struct device *dev, const char *fmt, ...);









static inline __attribute__((no_instrument_function)) int old_valid_dev(dev_t dev)
{
 return ((unsigned int) ((dev) >> 20)) < 256 && ((unsigned int) ((dev) & ((1U << 20) - 1))) < 256;
}

static inline __attribute__((no_instrument_function)) u16 old_encode_dev(dev_t dev)
{
 return (((unsigned int) ((dev) >> 20)) << 8) | ((unsigned int) ((dev) & ((1U << 20) - 1)));
}

static inline __attribute__((no_instrument_function)) dev_t old_decode_dev(u16 val)
{
 return ((((val >> 8) & 255) << 20) | (val & 255));
}

static inline __attribute__((no_instrument_function)) int new_valid_dev(dev_t dev)
{
 return 1;
}

static inline __attribute__((no_instrument_function)) u32 new_encode_dev(dev_t dev)
{
 unsigned major = ((unsigned int) ((dev) >> 20));
 unsigned minor = ((unsigned int) ((dev) & ((1U << 20) - 1)));
 return (minor & 0xff) | (major << 8) | ((minor & ~0xff) << 12);
}

static inline __attribute__((no_instrument_function)) dev_t new_decode_dev(u32 dev)
{
 unsigned major = (dev & 0xfff00) >> 8;
 unsigned minor = (dev & 0xff) | ((dev >> 12) & 0xfff00);
 return (((major) << 20) | (minor));
}

static inline __attribute__((no_instrument_function)) int huge_valid_dev(dev_t dev)
{
 return 1;
}

static inline __attribute__((no_instrument_function)) u64 huge_encode_dev(dev_t dev)
{
 return new_encode_dev(dev);
}

static inline __attribute__((no_instrument_function)) dev_t huge_decode_dev(u64 dev)
{
 return new_decode_dev(dev);
}

static inline __attribute__((no_instrument_function)) int sysv_valid_dev(dev_t dev)
{
 return ((unsigned int) ((dev) >> 20)) < (1<<14) && ((unsigned int) ((dev) & ((1U << 20) - 1))) < (1<<18);
}

static inline __attribute__((no_instrument_function)) u32 sysv_encode_dev(dev_t dev)
{
 return ((unsigned int) ((dev) & ((1U << 20) - 1))) | (((unsigned int) ((dev) >> 20)) << 18);
}

static inline __attribute__((no_instrument_function)) unsigned sysv_major(u32 dev)
{
 return (dev >> 18) & 0x3fff;
}

static inline __attribute__((no_instrument_function)) unsigned sysv_minor(u32 dev)
{
 return dev & 0x3ffff;
}





static inline __attribute__((no_instrument_function)) void INIT_LIST_HEAD_RCU(struct list_head *list)
{
 (*(volatile typeof(list->next) *)&(list->next)) = list;
 (*(volatile typeof(list->prev) *)&(list->prev)) = list;
}
static inline __attribute__((no_instrument_function)) void __list_add_rcu(struct list_head *new,
  struct list_head *prev, struct list_head *next)
{
 new->next = next;
 new->prev = prev;
 do { do { bool __cond = !((sizeof(*&(*((struct list_head **)(&(prev)->next)))) == sizeof(int) || sizeof(*&(*((struct list_head **)(&(prev)->next)))) == sizeof(long))); extern void __compiletime_assert_54(void) __attribute__((error("Need native word sized stores/loads for atomicity."))); if (__cond) __compiletime_assert_54(); do { } while (0); } while (0); __asm__ __volatile__("": : :"memory"); (*(volatile typeof(*&(*((struct list_head **)(&(prev)->next)))) *)&(*&(*((struct list_head **)(&(prev)->next))))) = ((typeof(*(new)) *)(new)); } while (0);
 next->prev = new;
}
static inline __attribute__((no_instrument_function)) void list_add_rcu(struct list_head *new, struct list_head *head)
{
 __list_add_rcu(new, head, head->next);
}
static inline __attribute__((no_instrument_function)) void list_add_tail_rcu(struct list_head *new,
     struct list_head *head)
{
 __list_add_rcu(new, head->prev, head);
}
static inline __attribute__((no_instrument_function)) void list_del_rcu(struct list_head *entry)
{
 __list_del_entry(entry);
 entry->prev = ((void *) 0x00200200 + (0xdead000000000000UL));
}
static inline __attribute__((no_instrument_function)) void hlist_del_init_rcu(struct hlist_node *n)
{
 if (!hlist_unhashed(n)) {
  __hlist_del(n);
  n->pprev = ((void *)0);
 }
}
static inline __attribute__((no_instrument_function)) void list_replace_rcu(struct list_head *old,
    struct list_head *new)
{
 new->next = old->next;
 new->prev = old->prev;
 do { do { bool __cond = !((sizeof(*&(*((struct list_head **)(&(new->prev)->next)))) == sizeof(int) || sizeof(*&(*((struct list_head **)(&(new->prev)->next)))) == sizeof(long))); extern void __compiletime_assert_176(void) __attribute__((error("Need native word sized stores/loads for atomicity."))); if (__cond) __compiletime_assert_176(); do { } while (0); } while (0); __asm__ __volatile__("": : :"memory"); (*(volatile typeof(*&(*((struct list_head **)(&(new->prev)->next)))) *)&(*&(*((struct list_head **)(&(new->prev)->next))))) = ((typeof(*(new)) *)(new)); } while (0);
 new->next->prev = new;
 old->prev = ((void *) 0x00200200 + (0xdead000000000000UL));
}
static inline __attribute__((no_instrument_function)) void list_splice_init_rcu(struct list_head *list,
     struct list_head *head,
     void (*sync)(void))
{
 struct list_head *first = list->next;
 struct list_head *last = list->prev;
 struct list_head *at = head->next;

 if (list_empty(list))
  return;







 INIT_LIST_HEAD_RCU(list);
 sync();
 last->next = at;
 do { do { bool __cond = !((sizeof(*&(*((struct list_head **)(&(head)->next)))) == sizeof(int) || sizeof(*&(*((struct list_head **)(&(head)->next)))) == sizeof(long))); extern void __compiletime_assert_235(void) __attribute__((error("Need native word sized stores/loads for atomicity."))); if (__cond) __compiletime_assert_235(); do { } while (0); } while (0); __asm__ __volatile__("": : :"memory"); (*(volatile typeof(*&(*((struct list_head **)(&(head)->next)))) *)&(*&(*((struct list_head **)(&(head)->next))))) = ((typeof(*(first)) *)(first)); } while (0);
 first->prev = head;
 at->prev = last;
}
static inline __attribute__((no_instrument_function)) void hlist_del_rcu(struct hlist_node *n)
{
 __hlist_del(n);
 n->pprev = ((void *) 0x00200200 + (0xdead000000000000UL));
}
static inline __attribute__((no_instrument_function)) void hlist_replace_rcu(struct hlist_node *old,
     struct hlist_node *new)
{
 struct hlist_node *next = old->next;

 new->next = next;
 new->pprev = old->pprev;
 do { do { bool __cond = !((sizeof(*&*(struct hlist_node **)new->pprev) == sizeof(int) || sizeof(*&*(struct hlist_node **)new->pprev) == sizeof(long))); extern void __compiletime_assert_363(void) __attribute__((error("Need native word sized stores/loads for atomicity."))); if (__cond) __compiletime_assert_363(); do { } while (0); } while (0); __asm__ __volatile__("": : :"memory"); (*(volatile typeof(*&*(struct hlist_node **)new->pprev) *)&(*&*(struct hlist_node **)new->pprev)) = ((typeof(*(new)) *)(new)); } while (0);
 if (next)
  new->next->pprev = &new->next;
 old->pprev = ((void *) 0x00200200 + (0xdead000000000000UL));
}
static inline __attribute__((no_instrument_function)) void hlist_add_head_rcu(struct hlist_node *n,
     struct hlist_head *h)
{
 struct hlist_node *first = h->first;

 n->next = first;
 n->pprev = &h->first;
 do { do { bool __cond = !((sizeof(*&(*((struct hlist_node **)(&(h)->first)))) == sizeof(int) || sizeof(*&(*((struct hlist_node **)(&(h)->first)))) == sizeof(long))); extern void __compiletime_assert_402(void) __attribute__((error("Need native word sized stores/loads for atomicity."))); if (__cond) __compiletime_assert_402(); do { } while (0); } while (0); __asm__ __volatile__("": : :"memory"); (*(volatile typeof(*&(*((struct hlist_node **)(&(h)->first)))) *)&(*&(*((struct hlist_node **)(&(h)->first))))) = ((typeof(*(n)) *)(n)); } while (0);
 if (first)
  first->pprev = &n->next;
}
static inline __attribute__((no_instrument_function)) void hlist_add_before_rcu(struct hlist_node *n,
     struct hlist_node *next)
{
 n->pprev = next->pprev;
 n->next = next;
 do { do { bool __cond = !((sizeof(*&(*((struct hlist_node **)((n)->pprev)))) == sizeof(int) || sizeof(*&(*((struct hlist_node **)((n)->pprev)))) == sizeof(long))); extern void __compiletime_assert_430(void) __attribute__((error("Need native word sized stores/loads for atomicity."))); if (__cond) __compiletime_assert_430(); do { } while (0); } while (0); __asm__ __volatile__("": : :"memory"); (*(volatile typeof(*&(*((struct hlist_node **)((n)->pprev)))) *)&(*&(*((struct hlist_node **)((n)->pprev))))) = ((typeof(*(n)) *)(n)); } while (0);
 next->pprev = &n->next;
}
static inline __attribute__((no_instrument_function)) void hlist_add_behind_rcu(struct hlist_node *n,
     struct hlist_node *prev)
{
 n->next = prev->next;
 n->pprev = &prev->next;
 do { do { bool __cond = !((sizeof(*&(*((struct hlist_node **)(&(prev)->next)))) == sizeof(int) || sizeof(*&(*((struct hlist_node **)(&(prev)->next)))) == sizeof(long))); extern void __compiletime_assert_457(void) __attribute__((error("Need native word sized stores/loads for atomicity."))); if (__cond) __compiletime_assert_457(); do { } while (0); } while (0); __asm__ __volatile__("": : :"memory"); (*(volatile typeof(*&(*((struct hlist_node **)(&(prev)->next)))) *)&(*&(*((struct hlist_node **)(&(prev)->next))))) = ((typeof(*(n)) *)(n)); } while (0);
 if (n->next)
  n->next->pprev = &n->next;
}










static inline __attribute__((no_instrument_function)) void bit_spin_lock(int bitnum, unsigned long *addr)
{







 __asm__ __volatile__("": : :"memory");

 while (__builtin_expect(!!(test_and_set_bit_lock(bitnum, addr)), 0)) {
  __asm__ __volatile__("": : :"memory");
  do {
   cpu_relax();
  } while ((__builtin_constant_p((bitnum)) ? constant_test_bit((bitnum), (addr)) : variable_test_bit((bitnum), (addr))));
  __asm__ __volatile__("": : :"memory");
 }

 (void)0;
}




static inline __attribute__((no_instrument_function)) int bit_spin_trylock(int bitnum, unsigned long *addr)
{
 __asm__ __volatile__("": : :"memory");

 if (__builtin_expect(!!(test_and_set_bit_lock(bitnum, addr)), 0)) {
  __asm__ __volatile__("": : :"memory");
  return 0;
 }

 (void)0;
 return 1;
}




static inline __attribute__((no_instrument_function)) void bit_spin_unlock(int bitnum, unsigned long *addr)
{




 clear_bit_unlock(bitnum, addr);

 __asm__ __volatile__("": : :"memory");
 (void)0;
}






static inline __attribute__((no_instrument_function)) void __bit_spin_unlock(int bitnum, unsigned long *addr)
{




 __clear_bit_unlock(bitnum, addr);

 __asm__ __volatile__("": : :"memory");
 (void)0;
}




static inline __attribute__((no_instrument_function)) int bit_spin_is_locked(int bitnum, unsigned long *addr)
{

 return (__builtin_constant_p((bitnum)) ? constant_test_bit((bitnum), (addr)) : variable_test_bit((bitnum), (addr)));





}
struct hlist_bl_head {
 struct hlist_bl_node *first;
};

struct hlist_bl_node {
 struct hlist_bl_node *next, **pprev;
};



static inline __attribute__((no_instrument_function)) void INIT_HLIST_BL_NODE(struct hlist_bl_node *h)
{
 h->next = ((void *)0);
 h->pprev = ((void *)0);
}



static inline __attribute__((no_instrument_function)) int hlist_bl_unhashed(const struct hlist_bl_node *h)
{
 return !h->pprev;
}

static inline __attribute__((no_instrument_function)) struct hlist_bl_node *hlist_bl_first(struct hlist_bl_head *h)
{
 return (struct hlist_bl_node *)
  ((unsigned long)h->first & ~1UL);
}

static inline __attribute__((no_instrument_function)) void hlist_bl_set_first(struct hlist_bl_head *h,
     struct hlist_bl_node *n)
{
 ;

                        ;
 h->first = (struct hlist_bl_node *)((unsigned long)n | 1UL);
}

static inline __attribute__((no_instrument_function)) int hlist_bl_empty(const struct hlist_bl_head *h)
{
 return !((unsigned long)h->first & ~1UL);
}

static inline __attribute__((no_instrument_function)) void hlist_bl_add_head(struct hlist_bl_node *n,
     struct hlist_bl_head *h)
{
 struct hlist_bl_node *first = hlist_bl_first(h);

 n->next = first;
 if (first)
  first->pprev = &n->next;
 n->pprev = &h->first;
 hlist_bl_set_first(h, n);
}

static inline __attribute__((no_instrument_function)) void __hlist_bl_del(struct hlist_bl_node *n)
{
 struct hlist_bl_node *next = n->next;
 struct hlist_bl_node **pprev = n->pprev;

 ;


 *pprev = (struct hlist_bl_node *)
   ((unsigned long)next |
    ((unsigned long)*pprev & 1UL));
 if (next)
  next->pprev = pprev;
}

static inline __attribute__((no_instrument_function)) void hlist_bl_del(struct hlist_bl_node *n)
{
 __hlist_bl_del(n);
 n->next = ((void *) 0x00100100 + (0xdead000000000000UL));
 n->pprev = ((void *) 0x00200200 + (0xdead000000000000UL));
}

static inline __attribute__((no_instrument_function)) void hlist_bl_del_init(struct hlist_bl_node *n)
{
 if (!hlist_bl_unhashed(n)) {
  __hlist_bl_del(n);
  INIT_HLIST_BL_NODE(n);
 }
}

static inline __attribute__((no_instrument_function)) void hlist_bl_lock(struct hlist_bl_head *b)
{
 bit_spin_lock(0, (unsigned long *)b);
}

static inline __attribute__((no_instrument_function)) void hlist_bl_unlock(struct hlist_bl_head *b)
{
 __bit_spin_unlock(0, (unsigned long *)b);
}

static inline __attribute__((no_instrument_function)) bool hlist_bl_is_locked(struct hlist_bl_head *b)
{
 return bit_spin_is_locked(0, (unsigned long *)b);
}


static inline __attribute__((no_instrument_function)) void hlist_bl_set_first_rcu(struct hlist_bl_head *h,
     struct hlist_bl_node *n)
{
 ;

                        ;
 do { do { bool __cond = !((sizeof(*&h->first) == sizeof(int) || sizeof(*&h->first) == sizeof(long))); extern void
 __compiletime_assert_17
 (void) __attribute__((error("Need native word sized stores/loads for atomicity."))); if (__cond)
 __compiletime_assert_17
 (); do { } while (0); } while (0); __asm__ __volatile__("": : :"memory"); (*(volatile typeof(*&h->first) *)&(*&h->first)) = ((typeof(*((struct hlist_bl_node *)((unsigned long)n | 1UL))) *)((struct hlist_bl_node *)((unsigned long)n | 1UL))); } while (0)
                                                                ;
}

static inline __attribute__((no_instrument_function)) struct hlist_bl_node *hlist_bl_first_rcu(struct hlist_bl_head *h)
{
 return (struct hlist_bl_node *)
  ((unsigned long)({ typeof(*(h->first)) *_________p1 = (typeof(*(h->first)) *)(*(volatile typeof((h->first)) *)&((h->first))); do { } while (0); ; do { } while (0); ((typeof(*(h->first)) *)(_________p1)); }) & ~1UL);
}
static inline __attribute__((no_instrument_function)) void hlist_bl_del_init_rcu(struct hlist_bl_node *n)
{
 if (!hlist_bl_unhashed(n)) {
  __hlist_bl_del(n);
  n->pprev = ((void *)0);
 }
}
static inline __attribute__((no_instrument_function)) void hlist_bl_del_rcu(struct hlist_bl_node *n)
{
 __hlist_bl_del(n);
 n->pprev = ((void *) 0x00200200 + (0xdead000000000000UL));
}
static inline __attribute__((no_instrument_function)) void hlist_bl_add_head_rcu(struct hlist_bl_node *n,
     struct hlist_bl_head *h)
{
 struct hlist_bl_node *first;


 first = hlist_bl_first(h);

 n->next = first;
 if (first)
  first->pprev = &n->next;
 n->pprev = &h->first;


 hlist_bl_set_first_rcu(h, n);
}




struct lockref {
 union {

  __u64 __attribute__((aligned(8))) lock_count;

  struct {
   spinlock_t lock;
   unsigned int count;
  };
 };
};

extern void lockref_get(struct lockref *);
extern int lockref_get_not_zero(struct lockref *);
extern int lockref_get_or_lock(struct lockref *);
extern int lockref_put_or_lock(struct lockref *);

extern void lockref_mark_dead(struct lockref *);
extern int lockref_get_not_dead(struct lockref *);


static inline __attribute__((no_instrument_function)) int __lockref_is_dead(const struct lockref *l)
{
 return ((int)l->count < 0);
}

struct nameidata;
struct path;
struct vfsmount;
struct qstr {
 union {
  struct {
   u32 hash; u32 len;;
  };
  u64 hash_len;
 };
 const unsigned char *name;
};






struct dentry_stat_t {
 long nr_dentry;
 long nr_unused;
 long age_limit;
 long want_pages;
 long dummy[2];
};
extern struct dentry_stat_t dentry_stat;






static inline __attribute__((no_instrument_function)) unsigned long
partial_name_hash(unsigned long c, unsigned long prevhash)
{
 return (prevhash + (c << 4) + (c >> 4)) * 11;
}





static inline __attribute__((no_instrument_function)) unsigned long end_name_hash(unsigned long hash)
{
 return (unsigned int) hash;
}


extern unsigned int full_name_hash(const unsigned char *, unsigned int);
struct dentry {

 unsigned int d_flags;
 seqcount_t d_seq;
 struct hlist_bl_node d_hash;
 struct dentry *d_parent;
 struct qstr d_name;
 struct inode *d_inode;

 unsigned char d_iname[32];


 struct lockref d_lockref;
 const struct dentry_operations *d_op;
 struct super_block *d_sb;
 unsigned long d_time;
 void *d_fsdata;

 struct list_head d_lru;



 union {
  struct list_head d_child;
   struct callback_head d_rcu;
 } d_u;
 struct list_head d_subdirs;
 struct hlist_node d_alias;
};







enum dentry_d_lock_class
{
 DENTRY_D_LOCK_NORMAL,
 DENTRY_D_LOCK_NESTED
};

struct dentry_operations {
 int (*d_revalidate)(struct dentry *, unsigned int);
 int (*d_weak_revalidate)(struct dentry *, unsigned int);
 int (*d_hash)(const struct dentry *, struct qstr *);
 int (*d_compare)(const struct dentry *, const struct dentry *,
   unsigned int, const char *, const struct qstr *);
 int (*d_delete)(const struct dentry *);
 void (*d_release)(struct dentry *);
 void (*d_prune)(struct dentry *);
 void (*d_iput)(struct dentry *, struct inode *);
 char *(*d_dname)(struct dentry *, char *, int);
 struct vfsmount *(*d_automount)(struct path *);
 int (*d_manage)(struct dentry *, bool);
} __attribute__((__aligned__((1 << (6)))));
extern seqlock_t rename_lock;

static inline __attribute__((no_instrument_function)) int dname_external(const struct dentry *dentry)
{
 return dentry->d_name.name != dentry->d_iname;
}




extern void d_instantiate(struct dentry *, struct inode *);
extern struct dentry * d_instantiate_unique(struct dentry *, struct inode *);
extern struct dentry * d_materialise_unique(struct dentry *, struct inode *);
extern int d_instantiate_no_diralias(struct dentry *, struct inode *);
extern void __d_drop(struct dentry *dentry);
extern void d_drop(struct dentry *dentry);
extern void d_delete(struct dentry *);
extern void d_set_d_op(struct dentry *dentry, const struct dentry_operations *op);


extern struct dentry * d_alloc(struct dentry *, const struct qstr *);
extern struct dentry * d_alloc_pseudo(struct super_block *, const struct qstr *);
extern struct dentry * d_splice_alias(struct inode *, struct dentry *);
extern struct dentry * d_add_ci(struct dentry *, struct inode *, struct qstr *);
extern struct dentry *d_find_any_alias(struct inode *inode);
extern struct dentry * d_obtain_alias(struct inode *);
extern struct dentry * d_obtain_root(struct inode *);
extern void shrink_dcache_sb(struct super_block *);
extern void shrink_dcache_parent(struct dentry *);
extern void shrink_dcache_for_umount(struct super_block *);
extern int d_invalidate(struct dentry *);


extern struct dentry * d_make_root(struct inode *);


extern void d_genocide(struct dentry *);

extern void d_tmpfile(struct dentry *, struct inode *);

extern struct dentry *d_find_alias(struct inode *);
extern void d_prune_aliases(struct inode *);


extern int have_submounts(struct dentry *);
extern int check_submounts_and_drop(struct dentry *);




extern void d_rehash(struct dentry *);
static inline __attribute__((no_instrument_function)) void d_add(struct dentry *entry, struct inode *inode)
{
 d_instantiate(entry, inode);
 d_rehash(entry);
}
static inline __attribute__((no_instrument_function)) struct dentry *d_add_unique(struct dentry *entry, struct inode *inode)
{
 struct dentry *res;

 res = d_instantiate_unique(entry, inode);
 d_rehash(res != ((void *)0) ? res : entry);
 return res;
}

extern void dentry_update_name_case(struct dentry *, struct qstr *);


extern void d_move(struct dentry *, struct dentry *);
extern void d_exchange(struct dentry *, struct dentry *);
extern struct dentry *d_ancestor(struct dentry *, struct dentry *);


extern struct dentry *d_lookup(const struct dentry *, const struct qstr *);
extern struct dentry *d_hash_and_lookup(struct dentry *, struct qstr *);
extern struct dentry *__d_lookup(const struct dentry *, const struct qstr *);
extern struct dentry *__d_lookup_rcu(const struct dentry *parent,
    const struct qstr *name, unsigned *seq);

static inline __attribute__((no_instrument_function)) unsigned d_count(const struct dentry *dentry)
{
 return dentry->d_lockref.count;
}


extern int d_validate(struct dentry *, struct dentry *);




extern char *dynamic_dname(struct dentry *, char *, int, const char *, ...);
extern char *simple_dname(struct dentry *, char *, int);

extern char *__d_path(const struct path *, const struct path *, char *, int);
extern char *d_absolute_path(const struct path *, char *, int);
extern char *d_path(const struct path *, char *, int);
extern char *dentry_path_raw(struct dentry *, char *, int);
extern char *dentry_path(struct dentry *, char *, int);
static inline __attribute__((no_instrument_function)) struct dentry *dget_dlock(struct dentry *dentry)
{
 if (dentry)
  dentry->d_lockref.count++;
 return dentry;
}

static inline __attribute__((no_instrument_function)) struct dentry *dget(struct dentry *dentry)
{
 if (dentry)
  lockref_get(&dentry->d_lockref);
 return dentry;
}

extern struct dentry *dget_parent(struct dentry *dentry);
static inline __attribute__((no_instrument_function)) int d_unhashed(const struct dentry *dentry)
{
 return hlist_bl_unhashed(&dentry->d_hash);
}

static inline __attribute__((no_instrument_function)) int d_unlinked(const struct dentry *dentry)
{
 return d_unhashed(dentry) && !((dentry) == (dentry)->d_parent);
}

static inline __attribute__((no_instrument_function)) int cant_mount(const struct dentry *dentry)
{
 return (dentry->d_flags & 0x00000100);
}

static inline __attribute__((no_instrument_function)) void dont_mount(struct dentry *dentry)
{
 spin_lock(&dentry->d_lockref.lock);
 dentry->d_flags |= 0x00000100;
 spin_unlock(&dentry->d_lockref.lock);
}

extern void dput(struct dentry *);

static inline __attribute__((no_instrument_function)) bool d_managed(const struct dentry *dentry)
{
 return dentry->d_flags & (0x00010000|0x00020000|0x00040000);
}

static inline __attribute__((no_instrument_function)) bool d_mountpoint(const struct dentry *dentry)
{
 return dentry->d_flags & 0x00010000;
}




static inline __attribute__((no_instrument_function)) void __d_set_type(struct dentry *dentry, unsigned type)
{
 dentry->d_flags = (dentry->d_flags & ~0x00700000) | type;
}

static inline __attribute__((no_instrument_function)) void __d_clear_type(struct dentry *dentry)
{
 __d_set_type(dentry, 0x00000000);
}

static inline __attribute__((no_instrument_function)) void d_set_type(struct dentry *dentry, unsigned type)
{
 spin_lock(&dentry->d_lockref.lock);
 __d_set_type(dentry, type);
 spin_unlock(&dentry->d_lockref.lock);
}

static inline __attribute__((no_instrument_function)) unsigned __d_entry_type(const struct dentry *dentry)
{
 return dentry->d_flags & 0x00700000;
}

static inline __attribute__((no_instrument_function)) bool d_can_lookup(const struct dentry *dentry)
{
 return __d_entry_type(dentry) == 0x00100000;
}

static inline __attribute__((no_instrument_function)) bool d_is_autodir(const struct dentry *dentry)
{
 return __d_entry_type(dentry) == 0x00200000;
}

static inline __attribute__((no_instrument_function)) bool d_is_dir(const struct dentry *dentry)
{
 return d_can_lookup(dentry) || d_is_autodir(dentry);
}

static inline __attribute__((no_instrument_function)) bool d_is_symlink(const struct dentry *dentry)
{
 return __d_entry_type(dentry) == 0x00300000;
}

static inline __attribute__((no_instrument_function)) bool d_is_file(const struct dentry *dentry)
{
 return __d_entry_type(dentry) == 0x00400000;
}

static inline __attribute__((no_instrument_function)) bool d_is_negative(const struct dentry *dentry)
{
 return __d_entry_type(dentry) == 0x00000000;
}

static inline __attribute__((no_instrument_function)) bool d_is_positive(const struct dentry *dentry)
{
 return !d_is_negative(dentry);
}

extern int sysctl_vfs_cache_pressure;

static inline __attribute__((no_instrument_function)) unsigned long vfs_pressure_ratio(unsigned long val)
{
 return ( { typeof(val) quot = (val) / (100); typeof(val) rem = (val) % (100); (quot * (sysctl_vfs_cache_pressure)) + ((rem * (sysctl_vfs_cache_pressure)) / (100)); } );
}



struct dentry;
struct vfsmount;

struct path {
 struct vfsmount *mnt;
 struct dentry *dentry;
};

extern void path_get(const struct path *);
extern void path_put(const struct path *);

static inline __attribute__((no_instrument_function)) int path_equal(const struct path *path1, const struct path *path2)
{
 return path1->mnt == path2->mnt && path1->dentry == path2->dentry;
}



enum lru_status {
 LRU_REMOVED,
 LRU_REMOVED_RETRY,

 LRU_ROTATE,
 LRU_SKIP,
 LRU_RETRY,

};

struct list_lru_node {
 spinlock_t lock;
 struct list_head list;

 long nr_items;
} __attribute__((__aligned__((1 << (6)))));

struct list_lru {
 struct list_lru_node *node;
 nodemask_t active_nodes;
};

void list_lru_destroy(struct list_lru *lru);
int list_lru_init_key(struct list_lru *lru, struct lock_class_key *key);
static inline __attribute__((no_instrument_function)) int list_lru_init(struct list_lru *lru)
{
 return list_lru_init_key(lru, ((void *)0));
}
bool list_lru_add(struct list_lru *lru, struct list_head *item);
bool list_lru_del(struct list_lru *lru, struct list_head *item);
unsigned long list_lru_count_node(struct list_lru *lru, int nid);
static inline __attribute__((no_instrument_function)) unsigned long list_lru_count(struct list_lru *lru)
{
 long count = 0;
 int nid;

 for ((nid) = __first_node(&(lru->active_nodes)); (nid) < (1 << 6); (nid) = __next_node(((nid)), &((lru->active_nodes))))
  count += list_lru_count_node(lru, nid);

 return count;
}

typedef enum lru_status
(*list_lru_walk_cb)(struct list_head *item, spinlock_t *lock, void *cb_arg);
unsigned long list_lru_walk_node(struct list_lru *lru, int nid,
     list_lru_walk_cb isolate, void *cb_arg,
     unsigned long *nr_to_walk);

static inline __attribute__((no_instrument_function)) unsigned long
list_lru_walk(struct list_lru *lru, list_lru_walk_cb isolate,
       void *cb_arg, unsigned long nr_to_walk)
{
 long isolated = 0;
 int nid;

 for ((nid) = __first_node(&(lru->active_nodes)); (nid) < (1 << 6); (nid) = __next_node(((nid)), &((lru->active_nodes)))) {
  isolated += list_lru_walk_node(lru, nid, isolate,
            cb_arg, &nr_to_walk);
  if (nr_to_walk <= 0)
   break;
 }
 return isolated;
}

static inline __attribute__((no_instrument_function)) int radix_tree_is_indirect_ptr(void *ptr)
{
 return (int)((unsigned long)ptr & 1);
}
struct radix_tree_node {
 unsigned int path;
 unsigned int count;
 union {
  struct {

   struct radix_tree_node *parent;

   void *private_data;
  };

  struct callback_head callback_head;
 };

 struct list_head private_list;
 void *slots[(1UL << (0 ? 4 : 6))];
 unsigned long tags[3][(((1UL << (0 ? 4 : 6)) + 64 - 1) / 64)];
};


struct radix_tree_root {
 unsigned int height;
 gfp_t gfp_mask;
 struct radix_tree_node *rnode;
};
static inline __attribute__((no_instrument_function)) void *radix_tree_deref_slot(void **pslot)
{
 return ({ typeof(*(*pslot)) *_________p1 = (typeof(*(*pslot)) *)(*(volatile typeof((*pslot)) *)&((*pslot))); do { } while (0); ; do { } while (0); ((typeof(*(*pslot)) *)(_________p1)); });
}
static inline __attribute__((no_instrument_function)) void *radix_tree_deref_slot_protected(void **pslot,
       spinlock_t *treelock)
{
 return ({ do { } while (0); ; ((typeof(*(*pslot)) *)((*pslot))); });
}
static inline __attribute__((no_instrument_function)) int radix_tree_deref_retry(void *arg)
{
 return __builtin_expect(!!((unsigned long)arg & 1), 0);
}






static inline __attribute__((no_instrument_function)) int radix_tree_exceptional_entry(void *arg)
{

 return (unsigned long)arg & 2;
}






static inline __attribute__((no_instrument_function)) int radix_tree_exception(void *arg)
{
 return __builtin_expect(!!((unsigned long)arg & (1 | 2)), 0)
                                                           ;
}
static inline __attribute__((no_instrument_function)) void radix_tree_replace_slot(void **pslot, void *item)
{
 do { if (__builtin_expect(!!(radix_tree_is_indirect_ptr(item)), 0)) do { asm volatile("1:\tud2\n" ".pushsection __bug_table,\"a\"\n" "2:\t.long 1b - 2b, %c0 - 2b\n" "\t.word %c1, 0\n" "\t.org 2b+%c2\n" ".popsection" : : "i" ("include/linux/radix-tree.h"), "i" (259), "i" (sizeof(struct bug_entry))); __builtin_unreachable(); } while (0); } while (0);
 do { do { bool __cond = !((sizeof(*&*pslot) == sizeof(int) || sizeof(*&*pslot) == sizeof(long))); extern void __compiletime_assert_260(void) __attribute__((error("Need native word sized stores/loads for atomicity."))); if (__cond) __compiletime_assert_260(); do { } while (0); } while (0); __asm__ __volatile__("": : :"memory"); (*(volatile typeof(*&*pslot) *)&(*&*pslot)) = ((typeof(*(item)) *)(item)); } while (0);
}

int __radix_tree_create(struct radix_tree_root *root, unsigned long index,
   struct radix_tree_node **nodep, void ***slotp);
int radix_tree_insert(struct radix_tree_root *, unsigned long, void *);
void *__radix_tree_lookup(struct radix_tree_root *root, unsigned long index,
     struct radix_tree_node **nodep, void ***slotp);
void *radix_tree_lookup(struct radix_tree_root *, unsigned long);
void **radix_tree_lookup_slot(struct radix_tree_root *, unsigned long);
bool __radix_tree_delete_node(struct radix_tree_root *root,
         struct radix_tree_node *node);
void *radix_tree_delete_item(struct radix_tree_root *, unsigned long, void *);
void *radix_tree_delete(struct radix_tree_root *, unsigned long);
unsigned int
radix_tree_gang_lookup(struct radix_tree_root *root, void **results,
   unsigned long first_index, unsigned int max_items);
unsigned int radix_tree_gang_lookup_slot(struct radix_tree_root *root,
   void ***results, unsigned long *indices,
   unsigned long first_index, unsigned int max_items);
int radix_tree_preload(gfp_t gfp_mask);
int radix_tree_maybe_preload(gfp_t gfp_mask);
void radix_tree_init(void);
void *radix_tree_tag_set(struct radix_tree_root *root,
   unsigned long index, unsigned int tag);
void *radix_tree_tag_clear(struct radix_tree_root *root,
   unsigned long index, unsigned int tag);
int radix_tree_tag_get(struct radix_tree_root *root,
   unsigned long index, unsigned int tag);
unsigned int
radix_tree_gang_lookup_tag(struct radix_tree_root *root, void **results,
  unsigned long first_index, unsigned int max_items,
  unsigned int tag);
unsigned int
radix_tree_gang_lookup_tag_slot(struct radix_tree_root *root, void ***results,
  unsigned long first_index, unsigned int max_items,
  unsigned int tag);
unsigned long radix_tree_range_tag_if_tagged(struct radix_tree_root *root,
  unsigned long *first_indexp, unsigned long last_index,
  unsigned long nr_to_tag,
  unsigned int fromtag, unsigned int totag);
int radix_tree_tagged(struct radix_tree_root *root, unsigned int tag);
unsigned long radix_tree_locate_item(struct radix_tree_root *root, void *item);

static inline __attribute__((no_instrument_function)) void radix_tree_preload_end(void)
{
 __asm__ __volatile__("": : :"memory");
}
struct radix_tree_iter {
 unsigned long index;
 unsigned long next_index;
 unsigned long tags;
};
static inline __attribute__((no_instrument_function)) __attribute__((always_inline)) void **
radix_tree_iter_init(struct radix_tree_iter *iter, unsigned long start)
{
 iter->index = 0;
 iter->next_index = start;
 return ((void *)0);
}
void **radix_tree_next_chunk(struct radix_tree_root *root,
        struct radix_tree_iter *iter, unsigned flags);







static inline __attribute__((no_instrument_function)) __attribute__((always_inline)) unsigned
radix_tree_chunk_size(struct radix_tree_iter *iter)
{
 return iter->next_index - iter->index;
}
static inline __attribute__((no_instrument_function)) __attribute__((always_inline)) void **
radix_tree_next_slot(void **slot, struct radix_tree_iter *iter, unsigned flags)
{
 if (flags & 0x0100) {
  iter->tags >>= 1;
  if (__builtin_expect(!!(iter->tags & 1ul), 1)) {
   iter->index++;
   return slot + 1;
  }
  if (!(flags & 0x0200) && __builtin_expect(!!(iter->tags), 1)) {
   unsigned offset = __ffs(iter->tags);

   iter->tags >>= offset;
   iter->index += offset + 1;
   return slot + offset + 1;
  }
 } else {
  unsigned size = radix_tree_chunk_size(iter) - 1;

  while (size--) {
   slot++;
   iter->index++;
   if (__builtin_expect(!!(*slot), 1))
    return slot;
   if (flags & 0x0200) {

    iter->next_index = 0;
    break;
   }
  }
 }
 return ((void *)0);
}







enum pid_type
{
 PIDTYPE_PID,
 PIDTYPE_PGID,
 PIDTYPE_SID,
 PIDTYPE_MAX
};
struct upid {

 int nr;
 struct pid_namespace *ns;
 struct hlist_node pid_chain;
};

struct pid
{
 atomic_t count;
 unsigned int level;

 struct hlist_head tasks[PIDTYPE_MAX];
 struct callback_head rcu;
 struct upid numbers[1];
};

extern struct pid init_struct_pid;

struct pid_link
{
 struct hlist_node node;
 struct pid *pid;
};

static inline __attribute__((no_instrument_function)) struct pid *get_pid(struct pid *pid)
{
 if (pid)
  atomic_inc(&pid->count);
 return pid;
}

extern void put_pid(struct pid *pid);
extern struct task_struct *pid_task(struct pid *pid, enum pid_type);
extern struct task_struct *get_pid_task(struct pid *pid, enum pid_type);

extern struct pid *get_task_pid(struct task_struct *task, enum pid_type type);




extern void attach_pid(struct task_struct *task, enum pid_type);
extern void detach_pid(struct task_struct *task, enum pid_type);
extern void change_pid(struct task_struct *task, enum pid_type,
   struct pid *pid);
extern void transfer_pid(struct task_struct *old, struct task_struct *new,
    enum pid_type);

struct pid_namespace;
extern struct pid_namespace init_pid_ns;
extern struct pid *find_pid_ns(int nr, struct pid_namespace *ns);
extern struct pid *find_vpid(int nr);




extern struct pid *find_get_pid(int nr);
extern struct pid *find_ge_pid(int nr, struct pid_namespace *);
int next_pidmap(struct pid_namespace *pid_ns, unsigned int last);

extern struct pid *alloc_pid(struct pid_namespace *ns);
extern void free_pid(struct pid *pid);
extern void disable_pid_allocation(struct pid_namespace *ns);
static inline __attribute__((no_instrument_function)) struct pid_namespace *ns_of_pid(struct pid *pid)
{
 struct pid_namespace *ns = ((void *)0);
 if (pid)
  ns = pid->numbers[pid->level].ns;
 return ns;
}







static inline __attribute__((no_instrument_function)) bool is_child_reaper(struct pid *pid)
{
 return pid->numbers[pid->level].nr == 1;
}
static inline __attribute__((no_instrument_function)) pid_t pid_nr(struct pid *pid)
{
 pid_t nr = 0;
 if (pid)
  nr = pid->numbers[0].nr;
 return nr;
}

pid_t pid_nr_ns(struct pid *pid, struct pid_namespace *ns);
pid_t pid_vnr(struct pid *pid);


struct task_struct;
typedef struct __user_cap_header_struct {
 __u32 version;
 int pid;
} *cap_user_header_t;

typedef struct __user_cap_data_struct {
        __u32 effective;
        __u32 permitted;
        __u32 inheritable;
} *cap_user_data_t;
struct vfs_cap_data {
 __le32 magic_etc;
 struct {
  __le32 permitted;
  __le32 inheritable;
 } data[2];
};





extern int file_caps_enabled;

typedef struct kernel_cap_struct {
 __u32 cap[2];
} kernel_cap_t;


struct cpu_vfs_cap_data {
 __u32 magic_etc;
 kernel_cap_t permitted;
 kernel_cap_t inheritable;
};





struct file;
struct inode;
struct dentry;
struct user_namespace;

struct user_namespace *current_user_ns(void);

extern const kernel_cap_t __cap_empty_set;
extern const kernel_cap_t __cap_init_eff_set;
static inline __attribute__((no_instrument_function)) kernel_cap_t cap_combine(const kernel_cap_t a,
           const kernel_cap_t b)
{
 kernel_cap_t dest;
 do { unsigned __capi; for (__capi = 0; __capi < 2; ++__capi) { dest.cap[__capi] = a.cap[__capi] | b.cap[__capi]; } } while (0);
 return dest;
}

static inline __attribute__((no_instrument_function)) kernel_cap_t cap_intersect(const kernel_cap_t a,
      const kernel_cap_t b)
{
 kernel_cap_t dest;
 do { unsigned __capi; for (__capi = 0; __capi < 2; ++__capi) { dest.cap[__capi] = a.cap[__capi] & b.cap[__capi]; } } while (0);
 return dest;
}

static inline __attribute__((no_instrument_function)) kernel_cap_t cap_drop(const kernel_cap_t a,
        const kernel_cap_t drop)
{
 kernel_cap_t dest;
 do { unsigned __capi; for (__capi = 0; __capi < 2; ++__capi) { dest.cap[__capi] = a.cap[__capi] &~ drop.cap[__capi]; } } while (0);
 return dest;
}

static inline __attribute__((no_instrument_function)) kernel_cap_t cap_invert(const kernel_cap_t c)
{
 kernel_cap_t dest;
 do { unsigned __capi; for (__capi = 0; __capi < 2; ++__capi) { dest.cap[__capi] = ~ c.cap[__capi]; } } while (0);
 return dest;
}

static inline __attribute__((no_instrument_function)) int cap_isclear(const kernel_cap_t a)
{
 unsigned __capi;
 for (__capi = 0; __capi < 2; ++__capi) {
  if (a.cap[__capi] != 0)
   return 0;
 }
 return 1;
}
static inline __attribute__((no_instrument_function)) int cap_issubset(const kernel_cap_t a, const kernel_cap_t set)
{
 kernel_cap_t dest;
 dest = cap_drop(a, set);
 return cap_isclear(dest);
}



static inline __attribute__((no_instrument_function)) int cap_is_fs_cap(int cap)
{
 const kernel_cap_t __cap_fs_set = ((kernel_cap_t){{ ((1 << ((0) & 31)) | (1 << ((27) & 31)) | (1 << ((1) & 31)) | (1 << ((2) & 31)) | (1 << ((3) & 31)) | (1 << ((4) & 31))) | (1 << ((9) & 31)), ((1 << ((32) & 31))) } });
 return !!((1 << ((cap) & 31)) & __cap_fs_set.cap[((cap) >> 5)]);
}

static inline __attribute__((no_instrument_function)) kernel_cap_t cap_drop_fs_set(const kernel_cap_t a)
{
 const kernel_cap_t __cap_fs_set = ((kernel_cap_t){{ ((1 << ((0) & 31)) | (1 << ((27) & 31)) | (1 << ((1) & 31)) | (1 << ((2) & 31)) | (1 << ((3) & 31)) | (1 << ((4) & 31))) | (1 << ((9) & 31)), ((1 << ((32) & 31))) } });
 return cap_drop(a, __cap_fs_set);
}

static inline __attribute__((no_instrument_function)) kernel_cap_t cap_raise_fs_set(const kernel_cap_t a,
         const kernel_cap_t permitted)
{
 const kernel_cap_t __cap_fs_set = ((kernel_cap_t){{ ((1 << ((0) & 31)) | (1 << ((27) & 31)) | (1 << ((1) & 31)) | (1 << ((2) & 31)) | (1 << ((3) & 31)) | (1 << ((4) & 31))) | (1 << ((9) & 31)), ((1 << ((32) & 31))) } });
 return cap_combine(a,
      cap_intersect(permitted, __cap_fs_set));
}

static inline __attribute__((no_instrument_function)) kernel_cap_t cap_drop_nfsd_set(const kernel_cap_t a)
{
 const kernel_cap_t __cap_fs_set = ((kernel_cap_t){{ ((1 << ((0) & 31)) | (1 << ((27) & 31)) | (1 << ((1) & 31)) | (1 << ((2) & 31)) | (1 << ((3) & 31)) | (1 << ((4) & 31))) | (1 << ((24) & 31)), ((1 << ((32) & 31))) } });
 return cap_drop(a, __cap_fs_set);
}

static inline __attribute__((no_instrument_function)) kernel_cap_t cap_raise_nfsd_set(const kernel_cap_t a,
           const kernel_cap_t permitted)
{
 const kernel_cap_t __cap_nfsd_set = ((kernel_cap_t){{ ((1 << ((0) & 31)) | (1 << ((27) & 31)) | (1 << ((1) & 31)) | (1 << ((2) & 31)) | (1 << ((3) & 31)) | (1 << ((4) & 31))) | (1 << ((24) & 31)), ((1 << ((32) & 31))) } });
 return cap_combine(a,
      cap_intersect(permitted, __cap_nfsd_set));
}

extern bool has_capability(struct task_struct *t, int cap);
extern bool has_ns_capability(struct task_struct *t,
         struct user_namespace *ns, int cap);
extern bool has_capability_noaudit(struct task_struct *t, int cap);
extern bool has_ns_capability_noaudit(struct task_struct *t,
          struct user_namespace *ns, int cap);
extern bool capable(int cap);
extern bool ns_capable(struct user_namespace *ns, int cap);
extern bool capable_wrt_inode_uidgid(const struct inode *inode, int cap);
extern bool file_ns_capable(const struct file *file, struct user_namespace *ns, int cap);


extern int get_vfs_caps_from_disk(const struct dentry *dentry, struct cpu_vfs_cap_data *cpu_caps);
struct semaphore {
 raw_spinlock_t lock;
 unsigned int count;
 struct list_head wait_list;
};
static inline __attribute__((no_instrument_function)) void sema_init(struct semaphore *sem, int val)
{
 static struct lock_class_key __key;
 *sem = (struct semaphore) { .lock = (raw_spinlock_t) { .raw_lock = { { 0 } }, }, .count = val, .wait_list = { &((*sem).wait_list), &((*sem).wait_list) }, };
 do { (void)("semaphore->lock"); (void)(&__key); } while (0);
}

extern void down(struct semaphore *sem);
extern int down_interruptible(struct semaphore *sem);
extern int down_killable(struct semaphore *sem);
extern int down_trylock(struct semaphore *sem);
extern int down_timeout(struct semaphore *sem, long jiffies);
extern void up(struct semaphore *sem);
struct fiemap_extent {
 __u64 fe_logical;

 __u64 fe_physical;

 __u64 fe_length;
 __u64 fe_reserved64[2];
 __u32 fe_flags;
 __u32 fe_reserved[3];
};

struct fiemap {
 __u64 fm_start;

 __u64 fm_length;

 __u32 fm_flags;
 __u32 fm_mapped_extents;
 __u32 fm_extent_count;
 __u32 fm_reserved;
 struct fiemap_extent fm_extents[0];
};


struct shrink_control {
 gfp_t gfp_mask;






 unsigned long nr_to_scan;


 nodemask_t nodes_to_scan;

 int nid;
};
struct shrinker {
 unsigned long (*count_objects)(struct shrinker *,
           struct shrink_control *sc);
 unsigned long (*scan_objects)(struct shrinker *,
          struct shrink_control *sc);

 int seeks;
 long batch;
 unsigned long flags;


 struct list_head list;

 atomic_long_t *nr_deferred;
};





extern int register_shrinker(struct shrinker *);
extern void unregister_shrinker(struct shrinker *);
enum migrate_mode {
 MIGRATE_ASYNC,
 MIGRATE_SYNC_LIGHT,
 MIGRATE_SYNC,
};


struct percpu_rw_semaphore {
 unsigned int *fast_read_ctr;
 atomic_t write_ctr;
 struct rw_semaphore rw_sem;
 atomic_t slow_read_ctr;
 wait_queue_head_t write_waitq;
};

extern void percpu_down_read(struct percpu_rw_semaphore *);
extern void percpu_up_read(struct percpu_rw_semaphore *);

extern void percpu_down_write(struct percpu_rw_semaphore *);
extern void percpu_up_write(struct percpu_rw_semaphore *);

extern int __percpu_init_rwsem(struct percpu_rw_semaphore *,
    const char *, struct lock_class_key *);
extern void percpu_free_rwsem(struct percpu_rw_semaphore *);
struct bio_set;
struct bio;
struct bio_integrity_payload;
struct page;
struct block_device;
struct io_context;
struct cgroup_subsys_state;
typedef void (bio_end_io_t) (struct bio *, int);
typedef void (bio_destructor_t) (struct bio *);




struct bio_vec {
 struct page *bv_page;
 unsigned int bv_len;
 unsigned int bv_offset;
};



struct bvec_iter {
 sector_t bi_sector;

 unsigned int bi_size;

 unsigned int bi_idx;

 unsigned int bi_bvec_done;

};





struct bio {
 struct bio *bi_next;
 struct block_device *bi_bdev;
 unsigned long bi_flags;
 unsigned long bi_rw;



 struct bvec_iter bi_iter;




 unsigned int bi_phys_segments;





 unsigned int bi_seg_front_size;
 unsigned int bi_seg_back_size;

 atomic_t bi_remaining;

 bio_end_io_t *bi_end_io;

 void *bi_private;





 struct io_context *bi_ioc;
 struct cgroup_subsys_state *bi_css;


 struct bio_integrity_payload *bi_integrity;


 unsigned short bi_vcnt;





 unsigned short bi_max_vecs;

 atomic_t bi_cnt;

 struct bio_vec *bi_io_vec;

 struct bio_set *bi_pool;






 struct bio_vec bi_inline_vecs[0];
};
enum rq_flag_bits {

 __REQ_WRITE,
 __REQ_FAILFAST_DEV,
 __REQ_FAILFAST_TRANSPORT,
 __REQ_FAILFAST_DRIVER,

 __REQ_SYNC,
 __REQ_META,
 __REQ_PRIO,
 __REQ_DISCARD,
 __REQ_SECURE,
 __REQ_WRITE_SAME,

 __REQ_NOIDLE,
 __REQ_FUA,
 __REQ_FLUSH,


 __REQ_RAHEAD,
 __REQ_THROTTLED,



 __REQ_SORTED,
 __REQ_SOFTBARRIER,
 __REQ_NOMERGE,
 __REQ_STARTED,
 __REQ_DONTPREP,
 __REQ_QUEUED,
 __REQ_ELVPRIV,
 __REQ_FAILED,
 __REQ_QUIET,
 __REQ_PREEMPT,
 __REQ_ALLOCED,
 __REQ_COPY_USER,
 __REQ_FLUSH_SEQ,
 __REQ_IO_STAT,
 __REQ_MIXED_MERGE,
 __REQ_KERNEL,
 __REQ_PM,
 __REQ_END,
 __REQ_HASHED,
 __REQ_MQ_INFLIGHT,
 __REQ_NR_BITS,
};


struct fstrim_range {
 __u64 start;
 __u64 len;
 __u64 minlen;
};


struct files_stat_struct {
 unsigned long nr_files;
 unsigned long nr_free_files;
 unsigned long max_files;
};

struct inodes_stat_t {
 long nr_inodes;
 long nr_unused;
 long dummy[5];
};

struct export_operations;
struct hd_geometry;
struct iovec;
struct nameidata;
struct kiocb;
struct kobject;
struct pipe_inode_info;
struct poll_table_struct;
struct kstatfs;
struct vm_area_struct;
struct vfsmount;
struct cred;
struct swap_info_struct;
struct seq_file;
struct workqueue_struct;
struct iov_iter;

extern void __attribute__ ((__section__(".init.text"))) __attribute__((__cold__)) __attribute__((no_instrument_function)) inode_init(void);
extern void __attribute__ ((__section__(".init.text"))) __attribute__((__cold__)) __attribute__((no_instrument_function)) inode_init_early(void);
extern void __attribute__ ((__section__(".init.text"))) __attribute__((__cold__)) __attribute__((no_instrument_function)) files_init(unsigned long);

extern struct files_stat_struct files_stat;
extern unsigned long get_max_files(void);
extern int sysctl_nr_open;
extern struct inodes_stat_t inodes_stat;
extern int leases_enable, lease_break_time;
extern int sysctl_protected_symlinks;
extern int sysctl_protected_hardlinks;

struct buffer_head;
typedef int (get_block_t)(struct inode *inode, sector_t iblock,
   struct buffer_head *bh_result, int create);
typedef void (dio_iodone_t)(struct kiocb *iocb, loff_t offset,
   ssize_t bytes, void *private);
struct iattr {
 unsigned int ia_valid;
 umode_t ia_mode;
 kuid_t ia_uid;
 kgid_t ia_gid;
 loff_t ia_size;
 struct timespec ia_atime;
 struct timespec ia_mtime;
 struct timespec ia_ctime;






 struct file *ia_file;
};




struct percpu_counter {
 raw_spinlock_t lock;
 s64 count;

 struct list_head list;

 s32 *counters;
};

extern int percpu_counter_batch;

int __percpu_counter_init(struct percpu_counter *fbc, s64 amount,
     struct lock_class_key *key);
void percpu_counter_destroy(struct percpu_counter *fbc);
void percpu_counter_set(struct percpu_counter *fbc, s64 amount);
void __percpu_counter_add(struct percpu_counter *fbc, s64 amount, s32 batch);
s64 __percpu_counter_sum(struct percpu_counter *fbc);
int percpu_counter_compare(struct percpu_counter *fbc, s64 rhs);

static inline __attribute__((no_instrument_function)) void percpu_counter_add(struct percpu_counter *fbc, s64 amount)
{
 __percpu_counter_add(fbc, amount, percpu_counter_batch);
}

static inline __attribute__((no_instrument_function)) s64 percpu_counter_sum_positive(struct percpu_counter *fbc)
{
 s64 ret = __percpu_counter_sum(fbc);
 return ret < 0 ? 0 : ret;
}

static inline __attribute__((no_instrument_function)) s64 percpu_counter_sum(struct percpu_counter *fbc)
{
 return __percpu_counter_sum(fbc);
}

static inline __attribute__((no_instrument_function)) s64 percpu_counter_read(struct percpu_counter *fbc)
{
 return fbc->count;
}






static inline __attribute__((no_instrument_function)) s64 percpu_counter_read_positive(struct percpu_counter *fbc)
{
 s64 ret = fbc->count;

 __asm__ __volatile__("": : :"memory");
 if (ret >= 0)
  return ret;
 return 0;
}

static inline __attribute__((no_instrument_function)) int percpu_counter_initialized(struct percpu_counter *fbc)
{
 return (fbc->counters != ((void *)0));
}
static inline __attribute__((no_instrument_function)) void percpu_counter_inc(struct percpu_counter *fbc)
{
 percpu_counter_add(fbc, 1);
}

static inline __attribute__((no_instrument_function)) void percpu_counter_dec(struct percpu_counter *fbc)
{
 percpu_counter_add(fbc, -1);
}

static inline __attribute__((no_instrument_function)) void percpu_counter_sub(struct percpu_counter *fbc, s64 amount)
{
 percpu_counter_add(fbc, -amount);
}

typedef struct fs_disk_quota {
 __s8 d_version;
 __s8 d_flags;
 __u16 d_fieldmask;
 __u32 d_id;
 __u64 d_blk_hardlimit;
 __u64 d_blk_softlimit;
 __u64 d_ino_hardlimit;
 __u64 d_ino_softlimit;
 __u64 d_bcount;
 __u64 d_icount;
 __s32 d_itimer;

 __s32 d_btimer;
 __u16 d_iwarns;
 __u16 d_bwarns;
 __s32 d_padding2;
 __u64 d_rtb_hardlimit;
 __u64 d_rtb_softlimit;
 __u64 d_rtbcount;
 __s32 d_rtbtimer;
 __u16 d_rtbwarns;
 __s16 d_padding3;
 char d_padding4[8];
} fs_disk_quota_t;
typedef struct fs_qfilestat {
 __u64 qfs_ino;
 __u64 qfs_nblks;
 __u32 qfs_nextents;
} fs_qfilestat_t;

typedef struct fs_quota_stat {
 __s8 qs_version;
 __u16 qs_flags;
 __s8 qs_pad;
 fs_qfilestat_t qs_uquota;
 fs_qfilestat_t qs_gquota;
 __u32 qs_incoredqs;
 __s32 qs_btimelimit;
 __s32 qs_itimelimit;
 __s32 qs_rtbtimelimit;
 __u16 qs_bwarnlimit;
 __u16 qs_iwarnlimit;
} fs_quota_stat_t;
struct fs_qfilestatv {
 __u64 qfs_ino;
 __u64 qfs_nblks;
 __u32 qfs_nextents;
 __u32 qfs_pad;
};

struct fs_quota_statv {
 __s8 qs_version;
 __u8 qs_pad1;
 __u16 qs_flags;
 __u32 qs_incoredqs;
 struct fs_qfilestatv qs_uquota;
 struct fs_qfilestatv qs_gquota;
 struct fs_qfilestatv qs_pquota;
 __s32 qs_btimelimit;
 __s32 qs_itimelimit;
 __s32 qs_rtbtimelimit;
 __u16 qs_bwarnlimit;
 __u16 qs_iwarnlimit;
 __u64 qs_pad2[8];
};







struct dquot;


struct qtree_fmt_operations {
 void (*mem2disk_dqblk)(void *disk, struct dquot *dquot);
 void (*disk2mem_dqblk)(struct dquot *dquot, void *disk);
 int (*is_id)(void *disk, struct dquot *dquot);
};


struct qtree_mem_dqinfo {
 struct super_block *dqi_sb;
 int dqi_type;
 unsigned int dqi_blocks;
 unsigned int dqi_free_blk;
 unsigned int dqi_free_entry;
 unsigned int dqi_blocksize_bits;
 unsigned int dqi_entry_size;
 unsigned int dqi_usable_bs;
 unsigned int dqi_qtree_depth;
 struct qtree_fmt_operations *dqi_ops;
};

int qtree_write_dquot(struct qtree_mem_dqinfo *info, struct dquot *dquot);
int qtree_read_dquot(struct qtree_mem_dqinfo *info, struct dquot *dquot);
int qtree_delete_dquot(struct qtree_mem_dqinfo *info, struct dquot *dquot);
int qtree_release_dquot(struct qtree_mem_dqinfo *info, struct dquot *dquot);
int qtree_entry_unused(struct qtree_mem_dqinfo *info, char *disk);
static inline __attribute__((no_instrument_function)) int qtree_depth(struct qtree_mem_dqinfo *info)
{
 unsigned int epb = info->dqi_usable_bs >> 2;
 unsigned long long entries = epb;
 int i;

 for (i = 1; entries < (1ULL << 32); i++)
  entries *= epb;
 return i;
}



struct user_namespace;
extern struct user_namespace init_user_ns;

typedef __kernel_uid32_t projid_t;

typedef struct {
 projid_t val;
} kprojid_t;

static inline __attribute__((no_instrument_function)) projid_t __kprojid_val(kprojid_t projid)
{
 return projid.val;
}






static inline __attribute__((no_instrument_function)) bool projid_eq(kprojid_t left, kprojid_t right)
{
 return __kprojid_val(left) == __kprojid_val(right);
}

static inline __attribute__((no_instrument_function)) bool projid_lt(kprojid_t left, kprojid_t right)
{
 return __kprojid_val(left) < __kprojid_val(right);
}

static inline __attribute__((no_instrument_function)) bool projid_valid(kprojid_t projid)
{
 return !projid_eq(projid, (kprojid_t){ -1 });
}
static inline __attribute__((no_instrument_function)) kprojid_t make_kprojid(struct user_namespace *from, projid_t projid)
{
 return (kprojid_t){ projid };
}

static inline __attribute__((no_instrument_function)) projid_t from_kprojid(struct user_namespace *to, kprojid_t kprojid)
{
 return __kprojid_val(kprojid);
}

static inline __attribute__((no_instrument_function)) projid_t from_kprojid_munged(struct user_namespace *to, kprojid_t kprojid)
{
 projid_t projid = from_kprojid(to, kprojid);
 if (projid == (projid_t)-1)
  projid = 65534;
 return projid;
}

static inline __attribute__((no_instrument_function)) bool kprojid_has_mapping(struct user_namespace *ns, kprojid_t projid)
{
 return true;
}
enum {
 QIF_BLIMITS_B = 0,
 QIF_SPACE_B,
 QIF_ILIMITS_B,
 QIF_INODES_B,
 QIF_BTIME_B,
 QIF_ITIME_B,
};
struct if_dqblk {
 __u64 dqb_bhardlimit;
 __u64 dqb_bsoftlimit;
 __u64 dqb_curspace;
 __u64 dqb_ihardlimit;
 __u64 dqb_isoftlimit;
 __u64 dqb_curinodes;
 __u64 dqb_btime;
 __u64 dqb_itime;
 __u32 dqb_valid;
};
struct if_dqinfo {
 __u64 dqi_bgrace;
 __u64 dqi_igrace;
 __u32 dqi_flags;
 __u32 dqi_valid;
};
enum {
 QUOTA_NL_C_UNSPEC,
 QUOTA_NL_C_WARNING,
 __QUOTA_NL_C_MAX,
};


enum {
 QUOTA_NL_A_UNSPEC,
 QUOTA_NL_A_QTYPE,
 QUOTA_NL_A_EXCESS_ID,
 QUOTA_NL_A_WARNING,
 QUOTA_NL_A_DEV_MAJOR,
 QUOTA_NL_A_DEV_MINOR,
 QUOTA_NL_A_CAUSED_ID,
 __QUOTA_NL_A_MAX,
};



enum quota_type {
 USRQUOTA = 0,
 GRPQUOTA = 1,
 PRJQUOTA = 2,
};

typedef __kernel_uid32_t qid_t;
typedef long long qsize_t;

struct kqid {
 union {
  kuid_t uid;
  kgid_t gid;
  kprojid_t projid;
 };
 enum quota_type type;
};

extern bool qid_eq(struct kqid left, struct kqid right);
extern bool qid_lt(struct kqid left, struct kqid right);
extern qid_t from_kqid(struct user_namespace *to, struct kqid qid);
extern qid_t from_kqid_munged(struct user_namespace *to, struct kqid qid);
extern bool qid_valid(struct kqid qid);
static inline __attribute__((no_instrument_function)) struct kqid make_kqid(struct user_namespace *from,
        enum quota_type type, qid_t qid)
{
 struct kqid kqid;

 kqid.type = type;
 switch (type) {
 case USRQUOTA:
  kqid.uid = make_kuid(from, qid);
  break;
 case GRPQUOTA:
  kqid.gid = make_kgid(from, qid);
  break;
 case PRJQUOTA:
  kqid.projid = make_kprojid(from, qid);
  break;
 default:
  do { asm volatile("1:\tud2\n" ".pushsection __bug_table,\"a\"\n" "2:\t.long 1b - 2b, %c0 - 2b\n" "\t.word %c1, 0\n" "\t.org 2b+%c2\n" ".popsection" : : "i" ("include/linux/quota.h"), "i" (108), "i" (sizeof(struct bug_entry))); __builtin_unreachable(); } while (0);
 }
 return kqid;
}







static inline __attribute__((no_instrument_function)) struct kqid make_kqid_invalid(enum quota_type type)
{
 struct kqid kqid;

 kqid.type = type;
 switch (type) {
 case USRQUOTA:
  kqid.uid = (kuid_t){ -1 };
  break;
 case GRPQUOTA:
  kqid.gid = (kgid_t){ -1 };
  break;
 case PRJQUOTA:
  kqid.projid = (kprojid_t){ -1 };
  break;
 default:
  do { asm volatile("1:\tud2\n" ".pushsection __bug_table,\"a\"\n" "2:\t.long 1b - 2b, %c0 - 2b\n" "\t.word %c1, 0\n" "\t.org 2b+%c2\n" ".popsection" : : "i" ("include/linux/quota.h"), "i" (135), "i" (sizeof(struct bug_entry))); __builtin_unreachable(); } while (0);
 }
 return kqid;
}





static inline __attribute__((no_instrument_function)) struct kqid make_kqid_uid(kuid_t uid)
{
 struct kqid kqid;
 kqid.type = USRQUOTA;
 kqid.uid = uid;
 return kqid;
}





static inline __attribute__((no_instrument_function)) struct kqid make_kqid_gid(kgid_t gid)
{
 struct kqid kqid;
 kqid.type = GRPQUOTA;
 kqid.gid = gid;
 return kqid;
}





static inline __attribute__((no_instrument_function)) struct kqid make_kqid_projid(kprojid_t projid)
{
 struct kqid kqid;
 kqid.type = PRJQUOTA;
 kqid.projid = projid;
 return kqid;
}


extern spinlock_t dq_data_lock;
struct mem_dqblk {
 qsize_t dqb_bhardlimit;
 qsize_t dqb_bsoftlimit;
 qsize_t dqb_curspace;
 qsize_t dqb_rsvspace;
 qsize_t dqb_ihardlimit;
 qsize_t dqb_isoftlimit;
 qsize_t dqb_curinodes;
 time_t dqb_btime;
 time_t dqb_itime;
};




struct quota_format_type;

struct mem_dqinfo {
 struct quota_format_type *dqi_format;
 int dqi_fmt_id;

 struct list_head dqi_dirty_list;
 unsigned long dqi_flags;
 unsigned int dqi_bgrace;
 unsigned int dqi_igrace;
 qsize_t dqi_maxblimit;
 qsize_t dqi_maxilimit;
 void *dqi_priv;
};

struct super_block;
extern void mark_info_dirty(struct super_block *sb, int type);
static inline __attribute__((no_instrument_function)) int info_dirty(struct mem_dqinfo *info)
{
 return (__builtin_constant_p((31)) ? constant_test_bit((31), (&info->dqi_flags)) : variable_test_bit((31), (&info->dqi_flags)));
}

enum {
 DQST_LOOKUPS,
 DQST_DROPS,
 DQST_READS,
 DQST_WRITES,
 DQST_CACHE_HITS,
 DQST_ALLOC_DQUOTS,
 DQST_FREE_DQUOTS,
 DQST_SYNCS,
 _DQST_DQSTAT_LAST
};

struct dqstats {
 int stat[_DQST_DQSTAT_LAST];
 struct percpu_counter counter[_DQST_DQSTAT_LAST];
};

extern struct dqstats *dqstats_pcpu;
extern struct dqstats dqstats;

static inline __attribute__((no_instrument_function)) void dqstats_inc(unsigned int type)
{
 percpu_counter_inc(&dqstats.counter[type]);
}

static inline __attribute__((no_instrument_function)) void dqstats_dec(unsigned int type)
{
 percpu_counter_dec(&dqstats.counter[type]);
}
struct dquot {
 struct hlist_node dq_hash;
 struct list_head dq_inuse;
 struct list_head dq_free;
 struct list_head dq_dirty;
 struct mutex dq_lock;
 atomic_t dq_count;
 wait_queue_head_t dq_wait_unused;
 struct super_block *dq_sb;
 struct kqid dq_id;
 loff_t dq_off;
 unsigned long dq_flags;
 struct mem_dqblk dq_dqb;
};


struct quota_format_ops {
 int (*check_quota_file)(struct super_block *sb, int type);
 int (*read_file_info)(struct super_block *sb, int type);
 int (*write_file_info)(struct super_block *sb, int type);
 int (*free_file_info)(struct super_block *sb, int type);
 int (*read_dqblk)(struct dquot *dquot);
 int (*commit_dqblk)(struct dquot *dquot);
 int (*release_dqblk)(struct dquot *dquot);
};


struct dquot_operations {
 int (*write_dquot) (struct dquot *);
 struct dquot *(*alloc_dquot)(struct super_block *, int);
 void (*destroy_dquot)(struct dquot *);
 int (*acquire_dquot) (struct dquot *);
 int (*release_dquot) (struct dquot *);
 int (*mark_dirty) (struct dquot *);
 int (*write_info) (struct super_block *, int);


 qsize_t *(*get_reserved_space) (struct inode *);
};

struct path;


struct quotactl_ops {
 int (*quota_on)(struct super_block *, int, int, struct path *);
 int (*quota_on_meta)(struct super_block *, int, int);
 int (*quota_off)(struct super_block *, int);
 int (*quota_sync)(struct super_block *, int);
 int (*get_info)(struct super_block *, int, struct if_dqinfo *);
 int (*set_info)(struct super_block *, int, struct if_dqinfo *);
 int (*get_dqblk)(struct super_block *, struct kqid, struct fs_disk_quota *);
 int (*set_dqblk)(struct super_block *, struct kqid, struct fs_disk_quota *);
 int (*get_xstate)(struct super_block *, struct fs_quota_stat *);
 int (*set_xstate)(struct super_block *, unsigned int, int);
 int (*get_xstatev)(struct super_block *, struct fs_quota_statv *);
 int (*rm_xquota)(struct super_block *, unsigned int);
};

struct quota_format_type {
 int qf_fmt_id;
 const struct quota_format_ops *qf_ops;
 struct module *qf_owner;
 struct quota_format_type *qf_next;
};


enum {
 _DQUOT_USAGE_ENABLED = 0,
 _DQUOT_LIMITS_ENABLED,
 _DQUOT_SUSPENDED,


 _DQUOT_STATE_FLAGS
};
static inline __attribute__((no_instrument_function)) unsigned int dquot_state_flag(unsigned int flags, int type)
{
 return flags << _DQUOT_STATE_FLAGS * type;
}

static inline __attribute__((no_instrument_function)) unsigned int dquot_generic_flag(unsigned int flags, int type)
{
 return (flags >> _DQUOT_STATE_FLAGS * type) & ((1 << _DQUOT_USAGE_ENABLED) | (1 << _DQUOT_LIMITS_ENABLED) | (1 << _DQUOT_SUSPENDED));
}


extern void quota_send_warning(struct kqid qid, dev_t dev,
          const char warntype);
struct quota_info {
 unsigned int flags;
 struct mutex dqio_mutex;
 struct mutex dqonoff_mutex;
 struct inode *files[2];
 struct mem_dqinfo info[2];
 const struct quota_format_ops *ops[2];
};

int register_quota_format(struct quota_format_type *fmt);
void unregister_quota_format(struct quota_format_type *fmt);

struct quota_module_name {
 int qm_fmt_id;
 char *qm_mod_name;
};
enum positive_aop_returns {
 AOP_WRITEPAGE_ACTIVATE = 0x80000,
 AOP_TRUNCATED_PAGE = 0x80001,
};
struct page;
struct address_space;
struct writeback_control;
typedef struct {
 size_t written;
 size_t count;
 union {
  char *buf;
  void *data;
 } arg;
 int error;
} read_descriptor_t;

typedef int (*read_actor_t)(read_descriptor_t *, struct page *,
  unsigned long, unsigned long);

struct address_space_operations {
 int (*writepage)(struct page *page, struct writeback_control *wbc);
 int (*readpage)(struct file *, struct page *);


 int (*writepages)(struct address_space *, struct writeback_control *);


 int (*set_page_dirty)(struct page *page);

 int (*readpages)(struct file *filp, struct address_space *mapping,
   struct list_head *pages, unsigned nr_pages);

 int (*write_begin)(struct file *, struct address_space *mapping,
    loff_t pos, unsigned len, unsigned flags,
    struct page **pagep, void **fsdata);
 int (*write_end)(struct file *, struct address_space *mapping,
    loff_t pos, unsigned len, unsigned copied,
    struct page *page, void *fsdata);


 sector_t (*bmap)(struct address_space *, sector_t);
 void (*invalidatepage) (struct page *, unsigned int, unsigned int);
 int (*releasepage) (struct page *, gfp_t);
 void (*freepage)(struct page *);
 ssize_t (*direct_IO)(int, struct kiocb *, struct iov_iter *iter, loff_t offset);
 int (*get_xip_mem)(struct address_space *, unsigned long, int,
      void **, unsigned long *);




 int (*migratepage) (struct address_space *,
   struct page *, struct page *, enum migrate_mode);
 int (*launder_page) (struct page *);
 int (*is_partially_uptodate) (struct page *, unsigned long,
     unsigned long);
 void (*is_dirty_writeback) (struct page *, bool *, bool *);
 int (*error_remove_page)(struct address_space *, struct page *);


 int (*swap_activate)(struct swap_info_struct *sis, struct file *file,
    sector_t *span);
 void (*swap_deactivate)(struct file *file);
};

extern const struct address_space_operations empty_aops;





int pagecache_write_begin(struct file *, struct address_space *mapping,
    loff_t pos, unsigned len, unsigned flags,
    struct page **pagep, void **fsdata);

int pagecache_write_end(struct file *, struct address_space *mapping,
    loff_t pos, unsigned len, unsigned copied,
    struct page *page, void *fsdata);

struct backing_dev_info;
struct address_space {
 struct inode *host;
 struct radix_tree_root page_tree;
 spinlock_t tree_lock;
 atomic_t i_mmap_writable;
 struct rb_root i_mmap;
 struct list_head i_mmap_nonlinear;
 struct mutex i_mmap_mutex;

 unsigned long nrpages;
 unsigned long nrshadows;
 unsigned long writeback_index;
 const struct address_space_operations *a_ops;
 unsigned long flags;
 struct backing_dev_info *backing_dev_info;
 spinlock_t private_lock;
 struct list_head private_list;
 void *private_data;
} __attribute__((aligned(sizeof(long))));





struct request_queue;

struct block_device {
 dev_t bd_dev;
 int bd_openers;
 struct inode * bd_inode;
 struct super_block * bd_super;
 struct mutex bd_mutex;
 struct list_head bd_inodes;
 void * bd_claiming;
 void * bd_holder;
 int bd_holders;
 bool bd_write_holder;

 struct list_head bd_holder_disks;

 struct block_device * bd_contains;
 unsigned bd_block_size;
 struct hd_struct * bd_part;

 unsigned bd_part_count;
 int bd_invalidated;
 struct gendisk * bd_disk;
 struct request_queue * bd_queue;
 struct list_head bd_list;






 unsigned long bd_private;


 int bd_fsfreeze_count;

 struct mutex bd_fsfreeze_mutex;
};
int mapping_tagged(struct address_space *mapping, int tag);




static inline __attribute__((no_instrument_function)) int mapping_mapped(struct address_space *mapping)
{
 return !((&mapping->i_mmap)->rb_node == ((void *)0)) ||
  !list_empty(&mapping->i_mmap_nonlinear);
}
static inline __attribute__((no_instrument_function)) int mapping_writably_mapped(struct address_space *mapping)
{
 return atomic_read(&mapping->i_mmap_writable) > 0;
}

static inline __attribute__((no_instrument_function)) int mapping_map_writable(struct address_space *mapping)
{
 return atomic_inc_unless_negative(&mapping->i_mmap_writable) ?
  0 : -1;
}

static inline __attribute__((no_instrument_function)) void mapping_unmap_writable(struct address_space *mapping)
{
 atomic_dec(&mapping->i_mmap_writable);
}

static inline __attribute__((no_instrument_function)) int mapping_deny_writable(struct address_space *mapping)
{
 return atomic_dec_unless_positive(&mapping->i_mmap_writable) ?
  0 : -16;
}

static inline __attribute__((no_instrument_function)) void mapping_allow_writable(struct address_space *mapping)
{
 atomic_inc(&mapping->i_mmap_writable);
}
struct posix_acl;
struct inode {
 umode_t i_mode;
 unsigned short i_opflags;
 kuid_t i_uid;
 kgid_t i_gid;
 unsigned int i_flags;


 struct posix_acl *i_acl;
 struct posix_acl *i_default_acl;


 const struct inode_operations *i_op;
 struct super_block *i_sb;
 struct address_space *i_mapping;


 void *i_security;



 unsigned long i_ino;







 union {
  const unsigned int i_nlink;
  unsigned int __i_nlink;
 };
 dev_t i_rdev;
 loff_t i_size;
 struct timespec i_atime;
 struct timespec i_mtime;
 struct timespec i_ctime;
 spinlock_t i_lock;
 unsigned short i_bytes;
 unsigned int i_blkbits;
 blkcnt_t i_blocks;






 unsigned long i_state;
 struct mutex i_mutex;

 unsigned long dirtied_when;

 struct hlist_node i_hash;
 struct list_head i_wb_list;
 struct list_head i_lru;
 struct list_head i_sb_list;
 union {
  struct hlist_head i_dentry;
  struct callback_head i_rcu;
 };
 u64 i_version;
 atomic_t i_count;
 atomic_t i_dio_count;
 atomic_t i_writecount;



 const struct file_operations *i_fop;
 struct file_lock *i_flock;
 struct address_space i_data;

 struct dquot *i_dquot[2];

 struct list_head i_devices;
 union {
  struct pipe_inode_info *i_pipe;
  struct block_device *i_bdev;
  struct cdev *i_cdev;
 };

 __u32 i_generation;


 __u32 i_fsnotify_mask;
 struct hlist_head i_fsnotify_marks;


 void *i_private;
};

static inline __attribute__((no_instrument_function)) int inode_unhashed(struct inode *inode)
{
 return hlist_unhashed(&inode->i_hash);
}
enum inode_i_mutex_lock_class
{
 I_MUTEX_NORMAL,
 I_MUTEX_PARENT,
 I_MUTEX_CHILD,
 I_MUTEX_XATTR,
 I_MUTEX_NONDIR2
};

void lock_two_nondirectories(struct inode *, struct inode*);
void unlock_two_nondirectories(struct inode *, struct inode*);
static inline __attribute__((no_instrument_function)) loff_t i_size_read(const struct inode *inode)
{
 return inode->i_size;

}






static inline __attribute__((no_instrument_function)) void i_size_write(struct inode *inode, loff_t i_size)
{
 inode->i_size = i_size;

}






static inline __attribute__((no_instrument_function)) uid_t i_uid_read(const struct inode *inode)
{
 return from_kuid(&init_user_ns, inode->i_uid);
}

static inline __attribute__((no_instrument_function)) gid_t i_gid_read(const struct inode *inode)
{
 return from_kgid(&init_user_ns, inode->i_gid);
}

static inline __attribute__((no_instrument_function)) void i_uid_write(struct inode *inode, uid_t uid)
{
 inode->i_uid = make_kuid(&init_user_ns, uid);
}

static inline __attribute__((no_instrument_function)) void i_gid_write(struct inode *inode, gid_t gid)
{
 inode->i_gid = make_kgid(&init_user_ns, gid);
}

static inline __attribute__((no_instrument_function)) unsigned iminor(const struct inode *inode)
{
 return ((unsigned int) ((inode->i_rdev) & ((1U << 20) - 1)));
}

static inline __attribute__((no_instrument_function)) unsigned imajor(const struct inode *inode)
{
 return ((unsigned int) ((inode->i_rdev) >> 20));
}

extern struct block_device *I_BDEV(struct inode *inode);

struct fown_struct {
 rwlock_t lock;
 struct pid *pid;
 enum pid_type pid_type;
 kuid_t uid, euid;
 int signum;
};




struct file_ra_state {
 unsigned long start;
 unsigned int size;
 unsigned int async_size;


 unsigned int ra_pages;
 unsigned int mmap_miss;
 loff_t prev_pos;
};




static inline __attribute__((no_instrument_function)) int ra_has_index(struct file_ra_state *ra, unsigned long index)
{
 return (index >= ra->start &&
  index < ra->start + ra->size);
}

struct file {
 union {
  struct llist_node fu_llist;
  struct callback_head fu_rcuhead;
 } f_u;
 struct path f_path;

 struct inode *f_inode;
 const struct file_operations *f_op;





 spinlock_t f_lock;
 atomic_long_t f_count;
 unsigned int f_flags;
 fmode_t f_mode;
 struct mutex f_pos_lock;
 loff_t f_pos;
 struct fown_struct f_owner;
 const struct cred *f_cred;
 struct file_ra_state f_ra;

 u64 f_version;

 void *f_security;


 void *private_data;



 struct list_head f_ep_links;
 struct list_head f_tfile_llink;

 struct address_space *f_mapping;
} __attribute__((aligned(4)));

struct file_handle {
 __u32 handle_bytes;
 int handle_type;

 unsigned char f_handle[0];
};

static inline __attribute__((no_instrument_function)) struct file *get_file(struct file *f)
{
 atomic_long_inc(&f->f_count);
 return f;
}
typedef void *fl_owner_t;

struct file_lock_operations {
 void (*fl_copy_lock)(struct file_lock *, struct file_lock *);
 void (*fl_release_private)(struct file_lock *);
};

struct lock_manager_operations {
 int (*lm_compare_owner)(struct file_lock *, struct file_lock *);
 unsigned long (*lm_owner_key)(struct file_lock *);
 void (*lm_notify)(struct file_lock *);
 int (*lm_grant)(struct file_lock *, struct file_lock *, int);
 void (*lm_break)(struct file_lock *);
 int (*lm_change)(struct file_lock **, int);
};

struct lock_manager {
 struct list_head list;
};

struct net;
void locks_start_grace(struct net *, struct lock_manager *);
void locks_end_grace(struct lock_manager *);
int locks_in_grace(struct net *);





struct nlm_lockowner;




struct nfs_lock_info {
 u32 state;
 struct nlm_lockowner *owner;
 struct list_head list;
};

struct nfs4_lock_state;
struct nfs4_lock_info {
 struct nfs4_lock_state *owner;
};
struct file_lock {
 struct file_lock *fl_next;
 struct hlist_node fl_link;
 struct list_head fl_block;
 fl_owner_t fl_owner;
 unsigned int fl_flags;
 unsigned char fl_type;
 unsigned int fl_pid;
 int fl_link_cpu;
 struct pid *fl_nspid;
 wait_queue_head_t fl_wait;
 struct file *fl_file;
 loff_t fl_start;
 loff_t fl_end;

 struct fasync_struct * fl_fasync;

 unsigned long fl_break_time;
 unsigned long fl_downgrade_time;

 const struct file_lock_operations *fl_ops;
 const struct lock_manager_operations *fl_lmops;
 union {
  struct nfs_lock_info nfs_fl;
  struct nfs4_lock_info nfs4_fl;
  struct {
   struct list_head link;
   int state;
  } afs;
 } fl_u;
};






struct f_owner_ex {
 int type;
 __kernel_pid_t pid;
};
struct flock {
 short l_type;
 short l_whence;
 __kernel_off_t l_start;
 __kernel_off_t l_len;
 __kernel_pid_t l_pid;

};







struct flock64 {
 short l_type;
 short l_whence;
 __kernel_loff_t l_start;
 __kernel_loff_t l_len;
 __kernel_pid_t l_pid;

};

extern void send_sigio(struct fown_struct *fown, int fd, int band);


extern int fcntl_getlk(struct file *, unsigned int, struct flock *);
extern int fcntl_setlk(unsigned int, struct file *, unsigned int,
   struct flock *);







extern int fcntl_setlease(unsigned int fd, struct file *filp, long arg);
extern int fcntl_getlease(struct file *filp);


void locks_free_lock(struct file_lock *fl);
extern void locks_init_lock(struct file_lock *);
extern struct file_lock * locks_alloc_lock(void);
extern void locks_copy_lock(struct file_lock *, struct file_lock *);
extern void __locks_copy_lock(struct file_lock *, const struct file_lock *);
extern void locks_remove_posix(struct file *, fl_owner_t);
extern void locks_remove_file(struct file *);
extern void locks_release_private(struct file_lock *);
extern void posix_test_lock(struct file *, struct file_lock *);
extern int posix_lock_file(struct file *, struct file_lock *, struct file_lock *);
extern int posix_lock_file_wait(struct file *, struct file_lock *);
extern int posix_unblock_lock(struct file_lock *);
extern int vfs_test_lock(struct file *, struct file_lock *);
extern int vfs_lock_file(struct file *, unsigned int, struct file_lock *, struct file_lock *);
extern int vfs_cancel_lock(struct file *filp, struct file_lock *fl);
extern int flock_lock_file_wait(struct file *filp, struct file_lock *fl);
extern int __break_lease(struct inode *inode, unsigned int flags, unsigned int type);
extern void lease_get_mtime(struct inode *, struct timespec *time);
extern int generic_setlease(struct file *, long, struct file_lock **);
extern int vfs_setlease(struct file *, long, struct file_lock **);
extern int lease_modify(struct file_lock **, int);
extern int lock_may_read(struct inode *, loff_t start, unsigned long count);
extern int lock_may_write(struct inode *, loff_t start, unsigned long count);
struct fasync_struct {
 spinlock_t fa_lock;
 int magic;
 int fa_fd;
 struct fasync_struct *fa_next;
 struct file *fa_file;
 struct callback_head fa_rcu;
};




extern int fasync_helper(int, struct file *, int, struct fasync_struct **);
extern struct fasync_struct *fasync_insert_entry(int, struct file *, struct fasync_struct **, struct fasync_struct *);
extern int fasync_remove_entry(struct file *, struct fasync_struct **);
extern struct fasync_struct *fasync_alloc(void);
extern void fasync_free(struct fasync_struct *);


extern void kill_fasync(struct fasync_struct **, int, int);

extern int __f_setown(struct file *filp, struct pid *, enum pid_type, int force);
extern int f_setown(struct file *filp, unsigned long arg, int force);
extern void f_delown(struct file *filp);
extern pid_t f_getown(struct file *filp);
extern int send_sigurg(struct fown_struct *fown);

struct mm_struct;
extern struct list_head super_blocks;
extern spinlock_t sb_lock;


enum {
 SB_UNFROZEN = 0,
 SB_FREEZE_WRITE = 1,
 SB_FREEZE_PAGEFAULT = 2,
 SB_FREEZE_FS = 3,

 SB_FREEZE_COMPLETE = 4,
};



struct sb_writers {

 struct percpu_counter counter[(SB_FREEZE_COMPLETE - 1)];
 wait_queue_head_t wait;

 int frozen;
 wait_queue_head_t wait_unfrozen;




};

struct super_block {
 struct list_head s_list;
 dev_t s_dev;
 unsigned char s_blocksize_bits;
 unsigned long s_blocksize;
 loff_t s_maxbytes;
 struct file_system_type *s_type;
 const struct super_operations *s_op;
 const struct dquot_operations *dq_op;
 const struct quotactl_ops *s_qcop;
 const struct export_operations *s_export_op;
 unsigned long s_flags;
 unsigned long s_magic;
 struct dentry *s_root;
 struct rw_semaphore s_umount;
 int s_count;
 atomic_t s_active;

 void *s_security;

 const struct xattr_handler **s_xattr;

 struct list_head s_inodes;
 struct hlist_bl_head s_anon;
 struct list_head s_mounts;
 struct block_device *s_bdev;
 struct backing_dev_info *s_bdi;
 struct mtd_info *s_mtd;
 struct hlist_node s_instances;
 struct quota_info s_dquot;

 struct sb_writers s_writers;

 char s_id[32];
 u8 s_uuid[16];

 void *s_fs_info;
 unsigned int s_max_links;
 fmode_t s_mode;



 u32 s_time_gran;





 struct mutex s_vfs_rename_mutex;





 char *s_subtype;





 char *s_options;
 const struct dentry_operations *s_d_op;




 int cleancache_poolid;

 struct shrinker s_shrink;


 atomic_long_t s_remove_count;


 int s_readonly_remount;


 struct workqueue_struct *s_dio_done_wq;
 struct hlist_head s_pins;





 struct list_lru s_dentry_lru __attribute__((__aligned__((1 << (6)))));
 struct list_lru s_inode_lru __attribute__((__aligned__((1 << (6)))));
 struct callback_head rcu;
};

extern struct timespec current_fs_time(struct super_block *sb);





void __sb_end_write(struct super_block *sb, int level);
int __sb_start_write(struct super_block *sb, int level, bool wait);
static inline __attribute__((no_instrument_function)) void sb_end_write(struct super_block *sb)
{
 __sb_end_write(sb, SB_FREEZE_WRITE);
}
static inline __attribute__((no_instrument_function)) void sb_end_pagefault(struct super_block *sb)
{
 __sb_end_write(sb, SB_FREEZE_PAGEFAULT);
}
static inline __attribute__((no_instrument_function)) void sb_end_intwrite(struct super_block *sb)
{
 __sb_end_write(sb, SB_FREEZE_FS);
}
static inline __attribute__((no_instrument_function)) void sb_start_write(struct super_block *sb)
{
 __sb_start_write(sb, SB_FREEZE_WRITE, true);
}

static inline __attribute__((no_instrument_function)) int sb_start_write_trylock(struct super_block *sb)
{
 return __sb_start_write(sb, SB_FREEZE_WRITE, false);
}
static inline __attribute__((no_instrument_function)) void sb_start_pagefault(struct super_block *sb)
{
 __sb_start_write(sb, SB_FREEZE_PAGEFAULT, true);
}
static inline __attribute__((no_instrument_function)) void sb_start_intwrite(struct super_block *sb)
{
 __sb_start_write(sb, SB_FREEZE_FS, true);
}


extern bool inode_owner_or_capable(const struct inode *inode);




extern int vfs_create(struct inode *, struct dentry *, umode_t, bool);
extern int vfs_mkdir(struct inode *, struct dentry *, umode_t);
extern int vfs_mknod(struct inode *, struct dentry *, umode_t, dev_t);
extern int vfs_symlink(struct inode *, struct dentry *, const char *);
extern int vfs_link(struct dentry *, struct inode *, struct dentry *, struct inode **);
extern int vfs_rmdir(struct inode *, struct dentry *);
extern int vfs_unlink(struct inode *, struct dentry *, struct inode **);
extern int vfs_rename(struct inode *, struct dentry *, struct inode *, struct dentry *, struct inode **, unsigned int);




extern void dentry_unhash(struct dentry *dentry);




extern void inode_init_owner(struct inode *inode, const struct inode *dir,
   umode_t mode);



struct fiemap_extent_info {
 unsigned int fi_flags;
 unsigned int fi_extents_mapped;
 unsigned int fi_extents_max;
 struct fiemap_extent *fi_extents_start;

};
int fiemap_fill_next_extent(struct fiemap_extent_info *info, u64 logical,
       u64 phys, u64 len, u32 flags);
int fiemap_check_flags(struct fiemap_extent_info *fieinfo, u32 fs_flags);
typedef int (*filldir_t)(void *, const char *, int, loff_t, u64, unsigned);
struct dir_context {
 const filldir_t actor;
 loff_t pos;
};

struct block_device_operations;







struct iov_iter;

struct file_operations {
 struct module *owner;
 loff_t (*llseek) (struct file *, loff_t, int);
 ssize_t (*read) (struct file *, char *, size_t, loff_t *);
 ssize_t (*write) (struct file *, const char *, size_t, loff_t *);
 ssize_t (*aio_read) (struct kiocb *, const struct iovec *, unsigned long, loff_t);
 ssize_t (*aio_write) (struct kiocb *, const struct iovec *, unsigned long, loff_t);
 ssize_t (*read_iter) (struct kiocb *, struct iov_iter *);
 ssize_t (*write_iter) (struct kiocb *, struct iov_iter *);
 int (*iterate) (struct file *, struct dir_context *);
 unsigned int (*poll) (struct file *, struct poll_table_struct *);
 long (*unlocked_ioctl) (struct file *, unsigned int, unsigned long);
 long (*compat_ioctl) (struct file *, unsigned int, unsigned long);
 int (*mmap) (struct file *, struct vm_area_struct *);
 int (*open) (struct inode *, struct file *);
 int (*flush) (struct file *, fl_owner_t id);
 int (*release) (struct inode *, struct file *);
 int (*fsync) (struct file *, loff_t, loff_t, int datasync);
 int (*aio_fsync) (struct kiocb *, int datasync);
 int (*fasync) (int, struct file *, int);
 int (*lock) (struct file *, int, struct file_lock *);
 ssize_t (*sendpage) (struct file *, struct page *, int, size_t, loff_t *, int);
 unsigned long (*get_unmapped_area)(struct file *, unsigned long, unsigned long, unsigned long, unsigned long);
 int (*check_flags)(int);
 int (*flock) (struct file *, int, struct file_lock *);
 ssize_t (*splice_write)(struct pipe_inode_info *, struct file *, loff_t *, size_t, unsigned int);
 ssize_t (*splice_read)(struct file *, loff_t *, struct pipe_inode_info *, size_t, unsigned int);
 int (*setlease)(struct file *, long, struct file_lock **);
 long (*fallocate)(struct file *file, int mode, loff_t offset,
     loff_t len);
 int (*show_fdinfo)(struct seq_file *m, struct file *f);
};

struct inode_operations {
 struct dentry * (*lookup) (struct inode *,struct dentry *, unsigned int);
 void * (*follow_link) (struct dentry *, struct nameidata *);
 int (*permission) (struct inode *, int);
 struct posix_acl * (*get_acl)(struct inode *, int);

 int (*readlink) (struct dentry *, char *,int);
 void (*put_link) (struct dentry *, struct nameidata *, void *);

 int (*create) (struct inode *,struct dentry *, umode_t, bool);
 int (*link) (struct dentry *,struct inode *,struct dentry *);
 int (*unlink) (struct inode *,struct dentry *);
 int (*symlink) (struct inode *,struct dentry *,const char *);
 int (*mkdir) (struct inode *,struct dentry *,umode_t);
 int (*rmdir) (struct inode *,struct dentry *);
 int (*mknod) (struct inode *,struct dentry *,umode_t,dev_t);
 int (*rename) (struct inode *, struct dentry *,
   struct inode *, struct dentry *);
 int (*rename2) (struct inode *, struct dentry *,
   struct inode *, struct dentry *, unsigned int);
 int (*setattr) (struct dentry *, struct iattr *);
 int (*getattr) (struct vfsmount *mnt, struct dentry *, struct kstat *);
 int (*setxattr) (struct dentry *, const char *,const void *,size_t,int);
 ssize_t (*getxattr) (struct dentry *, const char *, void *, size_t);
 ssize_t (*listxattr) (struct dentry *, char *, size_t);
 int (*removexattr) (struct dentry *, const char *);
 int (*fiemap)(struct inode *, struct fiemap_extent_info *, u64 start,
        u64 len);
 int (*update_time)(struct inode *, struct timespec *, int);
 int (*atomic_open)(struct inode *, struct dentry *,
      struct file *, unsigned open_flag,
      umode_t create_mode, int *opened);
 int (*tmpfile) (struct inode *, struct dentry *, umode_t);
 int (*set_acl)(struct inode *, struct posix_acl *, int);
} __attribute__((__aligned__((1 << (6)))));

ssize_t rw_copy_check_uvector(int type, const struct iovec * uvector,
         unsigned long nr_segs, unsigned long fast_segs,
         struct iovec *fast_pointer,
         struct iovec **ret_pointer);

extern ssize_t vfs_read(struct file *, char *, size_t, loff_t *);
extern ssize_t vfs_write(struct file *, const char *, size_t, loff_t *);
extern ssize_t vfs_readv(struct file *, const struct iovec *,
  unsigned long, loff_t *);
extern ssize_t vfs_writev(struct file *, const struct iovec *,
  unsigned long, loff_t *);

struct super_operations {
    struct inode *(*alloc_inode)(struct super_block *sb);
 void (*destroy_inode)(struct inode *);

    void (*dirty_inode) (struct inode *, int flags);
 int (*write_inode) (struct inode *, struct writeback_control *wbc);
 int (*drop_inode) (struct inode *);
 void (*evict_inode) (struct inode *);
 void (*put_super) (struct super_block *);
 int (*sync_fs)(struct super_block *sb, int wait);
 int (*freeze_fs) (struct super_block *);
 int (*unfreeze_fs) (struct super_block *);
 int (*statfs) (struct dentry *, struct kstatfs *);
 int (*remount_fs) (struct super_block *, int *, char *);
 void (*umount_begin) (struct super_block *);

 int (*show_options)(struct seq_file *, struct dentry *);
 int (*show_devname)(struct seq_file *, struct dentry *);
 int (*show_path)(struct seq_file *, struct dentry *);
 int (*show_stats)(struct seq_file *, struct dentry *);

 ssize_t (*quota_read)(struct super_block *, int, char *, size_t, loff_t);
 ssize_t (*quota_write)(struct super_block *, int, const char *, size_t, loff_t);

 int (*bdev_try_to_free_page)(struct super_block*, struct page*, gfp_t);
 long (*nr_cached_objects)(struct super_block *, int);
 long (*free_cached_objects)(struct super_block *, long, int);
};
extern void __mark_inode_dirty(struct inode *, int);
static inline __attribute__((no_instrument_function)) void mark_inode_dirty(struct inode *inode)
{
 __mark_inode_dirty(inode, ((1 << 0) | (1 << 1) | (1 << 2)));
}

static inline __attribute__((no_instrument_function)) void mark_inode_dirty_sync(struct inode *inode)
{
 __mark_inode_dirty(inode, (1 << 0));
}

extern void inc_nlink(struct inode *inode);
extern void drop_nlink(struct inode *inode);
extern void clear_nlink(struct inode *inode);
extern void set_nlink(struct inode *inode, unsigned int nlink);

static inline __attribute__((no_instrument_function)) void inode_inc_link_count(struct inode *inode)
{
 inc_nlink(inode);
 mark_inode_dirty(inode);
}

static inline __attribute__((no_instrument_function)) void inode_dec_link_count(struct inode *inode)
{
 drop_nlink(inode);
 mark_inode_dirty(inode);
}
static inline __attribute__((no_instrument_function)) void inode_inc_iversion(struct inode *inode)
{
       spin_lock(&inode->i_lock);
       inode->i_version++;
       spin_unlock(&inode->i_lock);
}

enum file_time_flags {
 S_ATIME = 1,
 S_MTIME = 2,
 S_CTIME = 4,
 S_VERSION = 8,
};

extern void touch_atime(const struct path *);
static inline __attribute__((no_instrument_function)) void file_accessed(struct file *file)
{
 if (!(file->f_flags & 01000000))
  touch_atime(&file->f_path);
}

int sync_inode(struct inode *inode, struct writeback_control *wbc);
int sync_inode_metadata(struct inode *inode, int wait);

struct file_system_type {
 const char *name;
 int fs_flags;






 struct dentry *(*mount) (struct file_system_type *, int,
         const char *, void *);
 void (*kill_sb) (struct super_block *);
 struct module *owner;
 struct file_system_type * next;
 struct hlist_head fs_supers;

 struct lock_class_key s_lock_key;
 struct lock_class_key s_umount_key;
 struct lock_class_key s_vfs_rename_key;
 struct lock_class_key s_writers_key[(SB_FREEZE_COMPLETE - 1)];

 struct lock_class_key i_lock_key;
 struct lock_class_key i_mutex_key;
 struct lock_class_key i_mutex_dir_key;
};



extern struct dentry *mount_ns(struct file_system_type *fs_type, int flags,
 void *data, int (*fill_super)(struct super_block *, void *, int));
extern struct dentry *mount_bdev(struct file_system_type *fs_type,
 int flags, const char *dev_name, void *data,
 int (*fill_super)(struct super_block *, void *, int));
extern struct dentry *mount_single(struct file_system_type *fs_type,
 int flags, void *data,
 int (*fill_super)(struct super_block *, void *, int));
extern struct dentry *mount_nodev(struct file_system_type *fs_type,
 int flags, void *data,
 int (*fill_super)(struct super_block *, void *, int));
extern struct dentry *mount_subtree(struct vfsmount *mnt, const char *path);
void generic_shutdown_super(struct super_block *sb);
void kill_block_super(struct super_block *sb);
void kill_anon_super(struct super_block *sb);
void kill_litter_super(struct super_block *sb);
void deactivate_super(struct super_block *sb);
void deactivate_locked_super(struct super_block *sb);
int set_anon_super(struct super_block *s, void *data);
int get_anon_bdev(dev_t *);
void free_anon_bdev(dev_t);
struct super_block *sget(struct file_system_type *type,
   int (*test)(struct super_block *,void *),
   int (*set)(struct super_block *,void *),
   int flags, void *data);
extern struct dentry *mount_pseudo(struct file_system_type *, char *,
 const struct super_operations *ops,
 const struct dentry_operations *dops,
 unsigned long);
extern int register_filesystem(struct file_system_type *);
extern int unregister_filesystem(struct file_system_type *);
extern struct vfsmount *kern_mount_data(struct file_system_type *, void *data);

extern void kern_unmount(struct vfsmount *mnt);
extern int may_umount_tree(struct vfsmount *);
extern int may_umount(struct vfsmount *);
extern long do_mount(const char *, const char *, const char *, unsigned long, void *);
extern struct vfsmount *collect_mounts(struct path *);
extern void drop_collected_mounts(struct vfsmount *);
extern int iterate_mounts(int (*)(struct vfsmount *, void *), void *,
     struct vfsmount *);
extern int vfs_statfs(struct path *, struct kstatfs *);
extern int user_statfs(const char *, struct kstatfs *);
extern int fd_statfs(int, struct kstatfs *);
extern int vfs_ustat(dev_t, struct kstatfs *);
extern int freeze_super(struct super_block *super);
extern int thaw_super(struct super_block *super);
extern bool our_mnt(struct vfsmount *mnt);
extern bool fs_fully_visible(struct file_system_type *);

extern int current_umask(void);

extern void ihold(struct inode * inode);
extern void iput(struct inode *);

static inline __attribute__((no_instrument_function)) struct inode *file_inode(struct file *f)
{
 return f->f_inode;
}


extern struct kobject *fs_kobj;







extern int locks_mandatory_locked(struct file *);
extern int locks_mandatory_area(int, struct inode *, struct file *, loff_t, size_t);






static inline __attribute__((no_instrument_function)) int __mandatory_lock(struct inode *ino)
{
 return (ino->i_mode & (0002000 | 00010)) == 0002000;
}






static inline __attribute__((no_instrument_function)) int mandatory_lock(struct inode *ino)
{
 return ((ino)->i_sb->s_flags & (64)) && __mandatory_lock(ino);
}

static inline __attribute__((no_instrument_function)) int locks_verify_locked(struct file *file)
{
 if (mandatory_lock(file_inode(file)))
  return locks_mandatory_locked(file);
 return 0;
}

static inline __attribute__((no_instrument_function)) int locks_verify_truncate(struct inode *inode,
        struct file *filp,
        loff_t size)
{
 if (inode->i_flock && mandatory_lock(inode))
  return locks_mandatory_area(
   2, inode, filp,
   size < inode->i_size ? size : inode->i_size,
   (size < inode->i_size ? inode->i_size - size
    : size - inode->i_size)
  );
 return 0;
}

static inline __attribute__((no_instrument_function)) int break_lease(struct inode *inode, unsigned int mode)
{





 asm volatile("mfence":::"memory");
 if (inode->i_flock)
  return __break_lease(inode, mode, 32);
 return 0;
}

static inline __attribute__((no_instrument_function)) int break_deleg(struct inode *inode, unsigned int mode)
{





 asm volatile("mfence":::"memory");
 if (inode->i_flock)
  return __break_lease(inode, mode, 4);
 return 0;
}

static inline __attribute__((no_instrument_function)) int try_break_deleg(struct inode *inode, struct inode **delegated_inode)
{
 int ret;

 ret = break_deleg(inode, 00000001|00004000);
 if (ret == -11 && delegated_inode) {
  *delegated_inode = inode;
  ihold(inode);
 }
 return ret;
}

static inline __attribute__((no_instrument_function)) int break_deleg_wait(struct inode **delegated_inode)
{
 int ret;

 ret = break_deleg(*delegated_inode, 00000001);
 iput(*delegated_inode);
 *delegated_inode = ((void *)0);
 return ret;
}
struct audit_names;
struct filename {
 const char *name;
 const char *uptr;
 struct audit_names *aname;
 bool separate;
};

extern long vfs_truncate(struct path *, loff_t);
extern int do_truncate(struct dentry *, loff_t start, unsigned int time_attrs,
         struct file *filp);
extern int do_fallocate(struct file *file, int mode, loff_t offset,
   loff_t len);
extern long do_sys_open(int dfd, const char *filename, int flags,
   umode_t mode);
extern struct file *file_open_name(struct filename *, int, umode_t);
extern struct file *filp_open(const char *, int, umode_t);
extern struct file *file_open_root(struct dentry *, struct vfsmount *,
       const char *, int);
extern struct file * dentry_open(const struct path *, int, const struct cred *);
extern int filp_close(struct file *, fl_owner_t id);

extern struct filename *getname(const char *);
extern struct filename *getname_kernel(const char *);

enum {
 FILE_CREATED = 1,
 FILE_OPENED = 2
};
extern int finish_open(struct file *file, struct dentry *dentry,
   int (*open)(struct inode *, struct file *),
   int *opened);
extern int finish_no_open(struct file *file, struct dentry *dentry);



extern int ioctl_preallocate(struct file *filp, void *argp);


extern void __attribute__ ((__section__(".init.text"))) __attribute__((__cold__)) __attribute__((no_instrument_function)) vfs_caches_init_early(void);
extern void __attribute__ ((__section__(".init.text"))) __attribute__((__cold__)) __attribute__((no_instrument_function)) vfs_caches_init(unsigned long);

extern struct kmem_cache *names_cachep;

extern void final_putname(struct filename *name);






extern void putname(struct filename *name);



extern int register_blkdev(unsigned int, const char *);
extern void unregister_blkdev(unsigned int, const char *);
extern struct block_device *bdget(dev_t);
extern struct block_device *bdgrab(struct block_device *bdev);
extern void bd_set_size(struct block_device *, loff_t size);
extern void bd_forget(struct inode *inode);
extern void bdput(struct block_device *);
extern void invalidate_bdev(struct block_device *);
extern void iterate_bdevs(void (*)(struct block_device *, void *), void *);
extern int sync_blockdev(struct block_device *bdev);
extern void kill_bdev(struct block_device *);
extern struct super_block *freeze_bdev(struct block_device *);
extern void emergency_thaw_all(void);
extern int thaw_bdev(struct block_device *bdev, struct super_block *sb);
extern int fsync_bdev(struct block_device *);
extern int sb_is_blkdev_sb(struct super_block *sb);
extern int sync_filesystem(struct super_block *);
extern const struct file_operations def_blk_fops;
extern const struct file_operations def_chr_fops;
extern const struct file_operations bad_sock_fops;

extern int ioctl_by_bdev(struct block_device *, unsigned, unsigned long);
extern int blkdev_ioctl(struct block_device *, fmode_t, unsigned, unsigned long);
extern long compat_blkdev_ioctl(struct file *, unsigned, unsigned long);
extern int blkdev_get(struct block_device *bdev, fmode_t mode, void *holder);
extern struct block_device *blkdev_get_by_path(const char *path, fmode_t mode,
            void *holder);
extern struct block_device *blkdev_get_by_dev(dev_t dev, fmode_t mode,
           void *holder);
extern void blkdev_put(struct block_device *bdev, fmode_t mode);

extern int bd_link_disk_holder(struct block_device *bdev, struct gendisk *disk);
extern void bd_unlink_disk_holder(struct block_device *bdev,
      struct gendisk *disk);
extern int alloc_chrdev_region(dev_t *, unsigned, unsigned, const char *);
extern int register_chrdev_region(dev_t, unsigned, const char *);
extern int __register_chrdev(unsigned int major, unsigned int baseminor,
        unsigned int count, const char *name,
        const struct file_operations *fops);
extern void __unregister_chrdev(unsigned int major, unsigned int baseminor,
    unsigned int count, const char *name);
extern void unregister_chrdev_region(dev_t, unsigned);
extern void chrdev_show(struct seq_file *,off_t);

static inline __attribute__((no_instrument_function)) int register_chrdev(unsigned int major, const char *name,
      const struct file_operations *fops)
{
 return __register_chrdev(major, 0, 256, name, fops);
}

static inline __attribute__((no_instrument_function)) void unregister_chrdev(unsigned int major, const char *name)
{
 __unregister_chrdev(major, 0, 256, name);
}







extern const char *__bdevname(dev_t, char *buffer);
extern const char *bdevname(struct block_device *bdev, char *buffer);
extern struct block_device *lookup_bdev(const char *);
extern void blkdev_show(struct seq_file *,off_t);





extern void init_special_inode(struct inode *, umode_t, dev_t);


extern void make_bad_inode(struct inode *);
extern int is_bad_inode(struct inode *);
extern void check_disk_size_change(struct gendisk *disk,
       struct block_device *bdev);
extern int revalidate_disk(struct gendisk *);
extern int check_disk_change(struct block_device *);
extern int __invalidate_device(struct block_device *, bool);
extern int invalidate_partition(struct gendisk *, int);

unsigned long invalidate_mapping_pages(struct address_space *mapping,
     unsigned long start, unsigned long end);

static inline __attribute__((no_instrument_function)) void invalidate_remote_inode(struct inode *inode)
{
 if ((((inode->i_mode) & 00170000) == 0100000) || (((inode->i_mode) & 00170000) == 0040000) ||
     (((inode->i_mode) & 00170000) == 0120000))
  invalidate_mapping_pages(inode->i_mapping, 0, -1);
}
extern int invalidate_inode_pages2(struct address_space *mapping);
extern int invalidate_inode_pages2_range(struct address_space *mapping,
      unsigned long start, unsigned long end);
extern int write_inode_now(struct inode *, int);
extern int filemap_fdatawrite(struct address_space *);
extern int filemap_flush(struct address_space *);
extern int filemap_fdatawait(struct address_space *);
extern int filemap_fdatawait_range(struct address_space *, loff_t lstart,
       loff_t lend);
extern int filemap_write_and_wait(struct address_space *mapping);
extern int filemap_write_and_wait_range(struct address_space *mapping,
            loff_t lstart, loff_t lend);
extern int __filemap_fdatawrite_range(struct address_space *mapping,
    loff_t start, loff_t end, int sync_mode);
extern int filemap_fdatawrite_range(struct address_space *mapping,
    loff_t start, loff_t end);

extern int vfs_fsync_range(struct file *file, loff_t start, loff_t end,
      int datasync);
extern int vfs_fsync(struct file *file, int datasync);
static inline __attribute__((no_instrument_function)) int generic_write_sync(struct file *file, loff_t pos, loff_t count)
{
 if (!(file->f_flags & 00010000) && !(((file->f_mapping->host)->i_sb->s_flags & (16)) || ((file->f_mapping->host)->i_flags & 1)))
  return 0;
 return vfs_fsync_range(file, pos, pos + count - 1,
          (file->f_flags & 04000000) ? 0 : 1);
}
extern void emergency_sync(void);
extern void emergency_remount(void);

extern sector_t bmap(struct inode *, sector_t);

extern int notify_change(struct dentry *, struct iattr *, struct inode **);
extern int inode_permission(struct inode *, int);
extern int generic_permission(struct inode *, int);

static inline __attribute__((no_instrument_function)) bool execute_ok(struct inode *inode)
{
 return (inode->i_mode & (00100|00010|00001)) || (((inode->i_mode) & 00170000) == 0040000);
}

static inline __attribute__((no_instrument_function)) void file_start_write(struct file *file)
{
 if (!(((file_inode(file)->i_mode) & 00170000) == 0100000))
  return;
 __sb_start_write(file_inode(file)->i_sb, SB_FREEZE_WRITE, true);
}

static inline __attribute__((no_instrument_function)) bool file_start_write_trylock(struct file *file)
{
 if (!(((file_inode(file)->i_mode) & 00170000) == 0100000))
  return true;
 return __sb_start_write(file_inode(file)->i_sb, SB_FREEZE_WRITE, false);
}

static inline __attribute__((no_instrument_function)) void file_end_write(struct file *file)
{
 if (!(((file_inode(file)->i_mode) & 00170000) == 0100000))
  return;
 __sb_end_write(file_inode(file)->i_sb, SB_FREEZE_WRITE);
}
static inline __attribute__((no_instrument_function)) int get_write_access(struct inode *inode)
{
 return atomic_inc_unless_negative(&inode->i_writecount) ? 0 : -26;
}
static inline __attribute__((no_instrument_function)) int deny_write_access(struct file *file)
{
 struct inode *inode = file_inode(file);
 return atomic_dec_unless_positive(&inode->i_writecount) ? 0 : -26;
}
static inline __attribute__((no_instrument_function)) void put_write_access(struct inode * inode)
{
 atomic_dec(&inode->i_writecount);
}
static inline __attribute__((no_instrument_function)) void allow_write_access(struct file *file)
{
 if (file)
  atomic_inc(&file_inode(file)->i_writecount);
}
static inline __attribute__((no_instrument_function)) bool inode_is_open_for_write(const struct inode *inode)
{
 return atomic_read(&inode->i_writecount) > 0;
}
static inline __attribute__((no_instrument_function)) void i_readcount_dec(struct inode *inode)
{
 return;
}
static inline __attribute__((no_instrument_function)) void i_readcount_inc(struct inode *inode)
{
 return;
}

extern int do_pipe_flags(int *, int);

extern int kernel_read(struct file *, loff_t, char *, unsigned long);
extern ssize_t kernel_write(struct file *, const char *, size_t, loff_t);
extern ssize_t __kernel_write(struct file *, const char *, size_t, loff_t *);
extern struct file * open_exec(const char *);


extern int is_subdir(struct dentry *, struct dentry *);
extern int path_is_under(struct path *, struct path *);




extern loff_t default_llseek(struct file *file, loff_t offset, int whence);

extern loff_t vfs_llseek(struct file *file, loff_t offset, int whence);

extern int inode_init_always(struct super_block *, struct inode *);
extern void inode_init_once(struct inode *);
extern void address_space_init_once(struct address_space *mapping);
extern struct inode * igrab(struct inode *);
extern ino_t iunique(struct super_block *, ino_t);
extern int inode_needs_sync(struct inode *inode);
extern int generic_delete_inode(struct inode *inode);
static inline __attribute__((no_instrument_function)) int generic_drop_inode(struct inode *inode)
{
 return !inode->i_nlink || inode_unhashed(inode);
}

extern struct inode *ilookup5_nowait(struct super_block *sb,
  unsigned long hashval, int (*test)(struct inode *, void *),
  void *data);
extern struct inode *ilookup5(struct super_block *sb, unsigned long hashval,
  int (*test)(struct inode *, void *), void *data);
extern struct inode *ilookup(struct super_block *sb, unsigned long ino);

extern struct inode * iget5_locked(struct super_block *, unsigned long, int (*test)(struct inode *, void *), int (*set)(struct inode *, void *), void *);
extern struct inode * iget_locked(struct super_block *, unsigned long);
extern int insert_inode_locked4(struct inode *, unsigned long, int (*test)(struct inode *, void *), void *);
extern int insert_inode_locked(struct inode *);



static inline __attribute__((no_instrument_function)) void lockdep_annotate_inode_mutex_key(struct inode *inode) { };

extern void unlock_new_inode(struct inode *);
extern unsigned int get_next_ino(void);

extern void __iget(struct inode * inode);
extern void iget_failed(struct inode *);
extern void clear_inode(struct inode *);
extern void __destroy_inode(struct inode *);
extern struct inode *new_inode_pseudo(struct super_block *sb);
extern struct inode *new_inode(struct super_block *sb);
extern void free_inode_nonrcu(struct inode *inode);
extern int should_remove_suid(struct dentry *);
extern int file_remove_suid(struct file *);

extern void __insert_inode_hash(struct inode *, unsigned long hashval);
static inline __attribute__((no_instrument_function)) void insert_inode_hash(struct inode *inode)
{
 __insert_inode_hash(inode, inode->i_ino);
}

extern void __remove_inode_hash(struct inode *);
static inline __attribute__((no_instrument_function)) void remove_inode_hash(struct inode *inode)
{
 if (!inode_unhashed(inode))
  __remove_inode_hash(inode);
}

extern void inode_sb_list_add(struct inode *inode);


extern void submit_bio(int, struct bio *);
extern int bdev_read_only(struct block_device *);

extern int set_blocksize(struct block_device *, int);
extern int sb_set_blocksize(struct super_block *, int);
extern int sb_min_blocksize(struct super_block *, int);

extern int generic_file_mmap(struct file *, struct vm_area_struct *);
extern int generic_file_readonly_mmap(struct file *, struct vm_area_struct *);
extern int generic_file_remap_pages(struct vm_area_struct *, unsigned long addr,
  unsigned long size, unsigned long pgoff);
int generic_write_checks(struct file *file, loff_t *pos, size_t *count, int isblk);
extern ssize_t generic_file_read_iter(struct kiocb *, struct iov_iter *);
extern ssize_t __generic_file_write_iter(struct kiocb *, struct iov_iter *);
extern ssize_t generic_file_write_iter(struct kiocb *, struct iov_iter *);
extern ssize_t generic_file_direct_write(struct kiocb *, struct iov_iter *, loff_t);
extern ssize_t generic_perform_write(struct file *, struct iov_iter *, loff_t);
extern ssize_t do_sync_read(struct file *filp, char *buf, size_t len, loff_t *ppos);
extern ssize_t do_sync_write(struct file *filp, const char *buf, size_t len, loff_t *ppos);
extern ssize_t new_sync_read(struct file *filp, char *buf, size_t len, loff_t *ppos);
extern ssize_t new_sync_write(struct file *filp, const char *buf, size_t len, loff_t *ppos);


extern ssize_t blkdev_write_iter(struct kiocb *iocb, struct iov_iter *from);
extern int blkdev_fsync(struct file *filp, loff_t start, loff_t end,
   int datasync);
extern void block_sync_page(struct page *page);


extern ssize_t generic_file_splice_read(struct file *, loff_t *,
  struct pipe_inode_info *, size_t, unsigned int);
extern ssize_t default_file_splice_read(struct file *, loff_t *,
  struct pipe_inode_info *, size_t, unsigned int);
extern ssize_t iter_file_splice_write(struct pipe_inode_info *,
  struct file *, loff_t *, size_t, unsigned int);
extern ssize_t generic_splice_sendpage(struct pipe_inode_info *pipe,
  struct file *out, loff_t *, size_t len, unsigned int flags);

extern void
file_ra_state_init(struct file_ra_state *ra, struct address_space *mapping);
extern loff_t noop_llseek(struct file *file, loff_t offset, int whence);
extern loff_t no_llseek(struct file *file, loff_t offset, int whence);
extern loff_t vfs_setpos(struct file *file, loff_t offset, loff_t maxsize);
extern loff_t generic_file_llseek(struct file *file, loff_t offset, int whence);
extern loff_t generic_file_llseek_size(struct file *file, loff_t offset,
  int whence, loff_t maxsize, loff_t eof);
extern loff_t fixed_size_llseek(struct file *file, loff_t offset,
  int whence, loff_t size);
extern int generic_file_open(struct inode * inode, struct file * filp);
extern int nonseekable_open(struct inode * inode, struct file * filp);
static inline __attribute__((no_instrument_function)) int xip_truncate_page(struct address_space *mapping, loff_t from)
{
 return 0;
}



typedef void (dio_submit_t)(int rw, struct bio *bio, struct inode *inode,
       loff_t file_offset);

enum {

 DIO_LOCKING = 0x01,


 DIO_SKIP_HOLES = 0x02,


 DIO_ASYNC_EXTEND = 0x04,
};

void dio_end_io(struct bio *bio, int error);

ssize_t __blockdev_direct_IO(int rw, struct kiocb *iocb, struct inode *inode,
 struct block_device *bdev, struct iov_iter *iter, loff_t offset,
 get_block_t get_block, dio_iodone_t end_io,
 dio_submit_t submit_io, int flags);

static inline __attribute__((no_instrument_function)) ssize_t blockdev_direct_IO(int rw, struct kiocb *iocb,
  struct inode *inode, struct iov_iter *iter, loff_t offset,
  get_block_t get_block)
{
 return __blockdev_direct_IO(rw, iocb, inode, inode->i_sb->s_bdev, iter,
        offset, get_block, ((void *)0), ((void *)0),
        DIO_LOCKING | DIO_SKIP_HOLES);
}


void inode_dio_wait(struct inode *inode);
void inode_dio_done(struct inode *inode);

extern void inode_set_flags(struct inode *inode, unsigned int flags,
       unsigned int mask);

extern const struct file_operations generic_ro_fops;



extern int readlink_copy(char *, int, const char *);
extern int page_readlink(struct dentry *, char *, int);
extern void *page_follow_link_light(struct dentry *, struct nameidata *);
extern void page_put_link(struct dentry *, struct nameidata *, void *);
extern int __page_symlink(struct inode *inode, const char *symname, int len,
  int nofs);
extern int page_symlink(struct inode *inode, const char *symname, int len);
extern const struct inode_operations page_symlink_inode_operations;
extern void kfree_put_link(struct dentry *, struct nameidata *, void *);
extern int generic_readlink(struct dentry *, char *, int);
extern void generic_fillattr(struct inode *, struct kstat *);
int vfs_getattr_nosec(struct path *path, struct kstat *stat);
extern int vfs_getattr(struct path *, struct kstat *);
void __inode_add_bytes(struct inode *inode, loff_t bytes);
void inode_add_bytes(struct inode *inode, loff_t bytes);
void __inode_sub_bytes(struct inode *inode, loff_t bytes);
void inode_sub_bytes(struct inode *inode, loff_t bytes);
loff_t inode_get_bytes(struct inode *inode);
void inode_set_bytes(struct inode *inode, loff_t bytes);

extern int vfs_readdir(struct file *, filldir_t, void *);
extern int iterate_dir(struct file *, struct dir_context *);

extern int vfs_stat(const char *, struct kstat *);
extern int vfs_lstat(const char *, struct kstat *);
extern int vfs_fstat(unsigned int, struct kstat *);
extern int vfs_fstatat(int , const char *, struct kstat *, int);

extern int do_vfs_ioctl(struct file *filp, unsigned int fd, unsigned int cmd,
      unsigned long arg);
extern int __generic_block_fiemap(struct inode *inode,
      struct fiemap_extent_info *fieinfo,
      loff_t start, loff_t len,
      get_block_t *get_block);
extern int generic_block_fiemap(struct inode *inode,
    struct fiemap_extent_info *fieinfo, u64 start,
    u64 len, get_block_t *get_block);

extern void get_filesystem(struct file_system_type *fs);
extern void put_filesystem(struct file_system_type *fs);
extern struct file_system_type *get_fs_type(const char *name);
extern struct super_block *get_super(struct block_device *);
extern struct super_block *get_super_thawed(struct block_device *);
extern struct super_block *get_active_super(struct block_device *bdev);
extern void drop_super(struct super_block *sb);
extern void iterate_supers(void (*)(struct super_block *, void *), void *);
extern void iterate_supers_type(struct file_system_type *,
           void (*)(struct super_block *, void *), void *);

extern int dcache_dir_open(struct inode *, struct file *);
extern int dcache_dir_close(struct inode *, struct file *);
extern loff_t dcache_dir_lseek(struct file *, loff_t, int);
extern int dcache_readdir(struct file *, struct dir_context *);
extern int simple_setattr(struct dentry *, struct iattr *);
extern int simple_getattr(struct vfsmount *, struct dentry *, struct kstat *);
extern int simple_statfs(struct dentry *, struct kstatfs *);
extern int simple_open(struct inode *inode, struct file *file);
extern int simple_link(struct dentry *, struct inode *, struct dentry *);
extern int simple_unlink(struct inode *, struct dentry *);
extern int simple_rmdir(struct inode *, struct dentry *);
extern int simple_rename(struct inode *, struct dentry *, struct inode *, struct dentry *);
extern int noop_fsync(struct file *, loff_t, loff_t, int);
extern int simple_empty(struct dentry *);
extern int simple_readpage(struct file *file, struct page *page);
extern int simple_write_begin(struct file *file, struct address_space *mapping,
   loff_t pos, unsigned len, unsigned flags,
   struct page **pagep, void **fsdata);
extern int simple_write_end(struct file *file, struct address_space *mapping,
   loff_t pos, unsigned len, unsigned copied,
   struct page *page, void *fsdata);
extern int always_delete_dentry(const struct dentry *);
extern struct inode *alloc_anon_inode(struct super_block *);
extern const struct dentry_operations simple_dentry_operations;

extern struct dentry *simple_lookup(struct inode *, struct dentry *, unsigned int flags);
extern ssize_t generic_read_dir(struct file *, char *, size_t, loff_t *);
extern const struct file_operations simple_dir_operations;
extern const struct inode_operations simple_dir_inode_operations;
struct tree_descr { char *name; const struct file_operations *ops; int mode; };
struct dentry *d_alloc_name(struct dentry *, const char *);
extern int simple_fill_super(struct super_block *, unsigned long, struct tree_descr *);
extern int simple_pin_fs(struct file_system_type *, struct vfsmount **mount, int *count);
extern void simple_release_fs(struct vfsmount **mount, int *count);

extern ssize_t simple_read_from_buffer(void *to, size_t count,
   loff_t *ppos, const void *from, size_t available);
extern ssize_t simple_write_to_buffer(void *to, size_t available, loff_t *ppos,
  const void *from, size_t count);

extern int __generic_file_fsync(struct file *, loff_t, loff_t, int);
extern int generic_file_fsync(struct file *, loff_t, loff_t, int);

extern int generic_check_addressable(unsigned, u64);


extern int buffer_migrate_page(struct address_space *,
    struct page *, struct page *,
    enum migrate_mode);




extern int inode_change_ok(const struct inode *, struct iattr *);
extern int inode_newsize_ok(const struct inode *, loff_t offset);
extern void setattr_copy(struct inode *inode, const struct iattr *attr);

extern int file_update_time(struct file *file);

extern int generic_show_options(struct seq_file *m, struct dentry *root);
extern void save_mount_options(struct super_block *sb, char *options);
extern void replace_mount_options(struct super_block *sb, char *options);

static inline __attribute__((no_instrument_function)) ino_t parent_ino(struct dentry *dentry)
{
 ino_t res;





 spin_lock(&dentry->d_lockref.lock);
 res = dentry->d_parent->d_inode->i_ino;
 spin_unlock(&dentry->d_lockref.lock);
 return res;
}







struct simple_transaction_argresp {
 ssize_t size;
 char data[0];
};



char *simple_transaction_get(struct file *file, const char *buf,
    size_t size);
ssize_t simple_transaction_read(struct file *file, char *buf,
    size_t size, loff_t *pos);
int simple_transaction_release(struct inode *inode, struct file *file);

void simple_transaction_set(struct file *file, size_t n);
static inline __attribute__((no_instrument_function)) __attribute__((format(printf, 1, 2)))
void __simple_attr_check_format(const char *fmt, ...)
{

}

int simple_attr_open(struct inode *inode, struct file *file,
       int (*get)(void *, u64 *), int (*set)(void *, u64),
       const char *fmt);
int simple_attr_release(struct inode *inode, struct file *file);
ssize_t simple_attr_read(struct file *file, char *buf,
    size_t len, loff_t *ppos);
ssize_t simple_attr_write(struct file *file, const char *buf,
     size_t len, loff_t *ppos);

struct ctl_table;
int proc_nr_files(struct ctl_table *table, int write,
    void *buffer, size_t *lenp, loff_t *ppos);
int proc_nr_dentry(struct ctl_table *table, int write,
    void *buffer, size_t *lenp, loff_t *ppos);
int proc_nr_inodes(struct ctl_table *table, int write,
     void *buffer, size_t *lenp, loff_t *ppos);
int __attribute__ ((__section__(".init.text"))) __attribute__((__cold__)) __attribute__((no_instrument_function)) get_filesystem_list(char *buf);
static inline __attribute__((no_instrument_function)) int is_sxid(umode_t mode)
{
 return (mode & 0004000) || ((mode & 0002000) && (mode & 00010));
}

static inline __attribute__((no_instrument_function)) void inode_has_no_xattr(struct inode *inode)
{
 if (!is_sxid(inode->i_mode) && (inode->i_sb->s_flags & (1<<28)))
  inode->i_flags |= 4096;
}

static inline __attribute__((no_instrument_function)) bool dir_emit(struct dir_context *ctx,
       const char *name, int namelen,
       u64 ino, unsigned type)
{
 return ctx->actor(ctx, name, namelen, ctx->pos, ino, type) == 0;
}
static inline __attribute__((no_instrument_function)) bool dir_emit_dot(struct file *file, struct dir_context *ctx)
{
 return ctx->actor(ctx, ".", 1, ctx->pos,
     file->f_path.dentry->d_inode->i_ino, 4) == 0;
}
static inline __attribute__((no_instrument_function)) bool dir_emit_dotdot(struct file *file, struct dir_context *ctx)
{
 return ctx->actor(ctx, "..", 2, ctx->pos,
     parent_ino(file->f_path.dentry), 4) == 0;
}
static inline __attribute__((no_instrument_function)) bool dir_emit_dots(struct file *file, struct dir_context *ctx)
{
 if (ctx->pos == 0) {
  if (!dir_emit_dot(file, ctx))
   return false;
  ctx->pos = 1;
 }
 if (ctx->pos == 1) {
  if (!dir_emit_dotdot(file, ctx))
   return false;
  ctx->pos = 2;
 }
 return true;
}
static inline __attribute__((no_instrument_function)) bool dir_relax(struct inode *inode)
{
 mutex_unlock(&inode->i_mutex);
 mutex_lock(&inode->i_mutex);
 return !((inode)->i_flags & 16);
}

typedef struct {
 __u8 b[16];
} uuid_le;

typedef struct {
 __u8 b[16];
} uuid_be;


static inline __attribute__((no_instrument_function)) int uuid_le_cmp(const uuid_le u1, const uuid_le u2)
{
 return memcmp(&u1, &u2, sizeof(uuid_le));
}

static inline __attribute__((no_instrument_function)) int uuid_be_cmp(const uuid_be u1, const uuid_be u2)
{
 return memcmp(&u1, &u2, sizeof(uuid_be));
}

extern void uuid_le_gen(uuid_le *u);
extern void uuid_be_gen(uuid_be *u);
typedef unsigned long kernel_ulong_t;




struct pci_device_id {
 __u32 vendor, device;
 __u32 subvendor, subdevice;
 __u32 class, class_mask;
 kernel_ulong_t driver_data;
};







struct ieee1394_device_id {
 __u32 match_flags;
 __u32 vendor_id;
 __u32 model_id;
 __u32 specifier_id;
 __u32 version;
 kernel_ulong_t driver_data;
};
struct usb_device_id {

 __u16 match_flags;


 __u16 idVendor;
 __u16 idProduct;
 __u16 bcdDevice_lo;
 __u16 bcdDevice_hi;


 __u8 bDeviceClass;
 __u8 bDeviceSubClass;
 __u8 bDeviceProtocol;


 __u8 bInterfaceClass;
 __u8 bInterfaceSubClass;
 __u8 bInterfaceProtocol;


 __u8 bInterfaceNumber;


 kernel_ulong_t driver_info
  __attribute__((aligned(sizeof(kernel_ulong_t))));
};
struct hid_device_id {
 __u16 bus;
 __u16 group;
 __u32 vendor;
 __u32 product;
 kernel_ulong_t driver_data;
};


struct ccw_device_id {
 __u16 match_flags;

 __u16 cu_type;
 __u16 dev_type;
 __u8 cu_model;
 __u8 dev_model;

 kernel_ulong_t driver_info;
};







struct ap_device_id {
 __u16 match_flags;
 __u8 dev_type;
 kernel_ulong_t driver_info;
};




struct css_device_id {
 __u8 match_flags;
 __u8 type;
 kernel_ulong_t driver_data;
};



struct acpi_device_id {
 __u8 id[9];
 kernel_ulong_t driver_data;
};




struct pnp_device_id {
 __u8 id[8];
 kernel_ulong_t driver_data;
};

struct pnp_card_device_id {
 __u8 id[8];
 kernel_ulong_t driver_data;
 struct {
  __u8 id[8];
 } devs[8];
};




struct serio_device_id {
 __u8 type;
 __u8 extra;
 __u8 id;
 __u8 proto;
};




struct of_device_id
{
 char name[32];
 char type[32];
 char compatible[128];
 const void *data;
};


struct vio_device_id {
 char type[32];
 char compat[32];
};



struct pcmcia_device_id {
 __u16 match_flags;

 __u16 manf_id;
 __u16 card_id;

 __u8 func_id;


 __u8 function;


 __u8 device_no;

 __u32 prod_id_hash[4];


 const char * prod_id[4];


 kernel_ulong_t driver_info;
 char * cisfile;
};
struct input_device_id {

 kernel_ulong_t flags;

 __u16 bustype;
 __u16 vendor;
 __u16 product;
 __u16 version;

 kernel_ulong_t evbit[0x1f / 64 + 1];
 kernel_ulong_t keybit[0x2ff / 64 + 1];
 kernel_ulong_t relbit[0x0f / 64 + 1];
 kernel_ulong_t absbit[0x3f / 64 + 1];
 kernel_ulong_t mscbit[0x07 / 64 + 1];
 kernel_ulong_t ledbit[0x0f / 64 + 1];
 kernel_ulong_t sndbit[0x07 / 64 + 1];
 kernel_ulong_t ffbit[0x7f / 64 + 1];
 kernel_ulong_t swbit[0x0f / 64 + 1];

 kernel_ulong_t driver_info;
};






struct eisa_device_id {
 char sig[8];
 kernel_ulong_t driver_data;
};



struct parisc_device_id {
 __u8 hw_type;
 __u8 hversion_rev;
 __u16 hversion;
 __u32 sversion;
};
struct sdio_device_id {
 __u8 class;
 __u16 vendor;
 __u16 device;
 kernel_ulong_t driver_data;
};


struct ssb_device_id {
 __u16 vendor;
 __u16 coreid;
 __u8 revision;
 __u8 __pad;
} __attribute__((packed, aligned(2)));
struct bcma_device_id {
 __u16 manuf;
 __u16 id;
 __u8 rev;
 __u8 class;
} __attribute__((packed,aligned(2)));
struct virtio_device_id {
 __u32 device;
 __u32 vendor;
};





struct hv_vmbus_device_id {
 __u8 guid[16];
 kernel_ulong_t driver_data;
};






struct rpmsg_device_id {
 char name[32];
};






struct i2c_device_id {
 char name[20];
 kernel_ulong_t driver_data;
};






struct spi_device_id {
 char name[32];
 kernel_ulong_t driver_data;
};




struct spmi_device_id {
 char name[32];
 kernel_ulong_t driver_data;
};


enum dmi_field {
 DMI_NONE,
 DMI_BIOS_VENDOR,
 DMI_BIOS_VERSION,
 DMI_BIOS_DATE,
 DMI_SYS_VENDOR,
 DMI_PRODUCT_NAME,
 DMI_PRODUCT_VERSION,
 DMI_PRODUCT_SERIAL,
 DMI_PRODUCT_UUID,
 DMI_BOARD_VENDOR,
 DMI_BOARD_NAME,
 DMI_BOARD_VERSION,
 DMI_BOARD_SERIAL,
 DMI_BOARD_ASSET_TAG,
 DMI_CHASSIS_VENDOR,
 DMI_CHASSIS_TYPE,
 DMI_CHASSIS_VERSION,
 DMI_CHASSIS_SERIAL,
 DMI_CHASSIS_ASSET_TAG,
 DMI_STRING_MAX,
};

struct dmi_strmatch {
 unsigned char slot:7;
 unsigned char exact_match:1;
 char substr[79];
};

struct dmi_system_id {
 int (*callback)(const struct dmi_system_id *);
 const char *ident;
 struct dmi_strmatch matches[4];
 void *driver_data;
};
struct platform_device_id {
 char name[20];
 kernel_ulong_t driver_data;
};
struct mdio_device_id {
 __u32 phy_id;
 __u32 phy_id_mask;
};

struct zorro_device_id {
 __u32 id;
 kernel_ulong_t driver_data;
};






struct isapnp_device_id {
 unsigned short card_vendor, card_device;
 unsigned short vendor, function;
 kernel_ulong_t driver_data;
};
struct amba_id {
 unsigned int id;
 unsigned int mask;
 void *data;
};
struct x86_cpu_id {
 __u16 vendor;
 __u16 family;
 __u16 model;
 __u16 feature;
 kernel_ulong_t driver_data;
};
struct cpu_feature {
 __u16 feature;
};



struct ipack_device_id {
 __u8 format;
 __u32 vendor;
 __u32 device;
};




struct mei_cl_device_id {
 char name[32];
 kernel_ulong_t driver_info;
};
struct rio_device_id {
 __u16 did, vid;
 __u16 asm_did, asm_vid;
};

struct mcb_device_id {
 __u16 device;
 kernel_ulong_t driver_data;
};







struct input_value {
 __u16 type;
 __u16 code;
 __s32 value;
};
struct input_dev {
 const char *name;
 const char *phys;
 const char *uniq;
 struct input_id id;

 unsigned long propbit[((((0x1f + 1)) + (8 * sizeof(long)) - 1) / (8 * sizeof(long)))];

 unsigned long evbit[((((0x1f +1)) + (8 * sizeof(long)) - 1) / (8 * sizeof(long)))];
 unsigned long keybit[((((0x2ff +1)) + (8 * sizeof(long)) - 1) / (8 * sizeof(long)))];
 unsigned long relbit[((((0x0f +1)) + (8 * sizeof(long)) - 1) / (8 * sizeof(long)))];
 unsigned long absbit[((((0x3f +1)) + (8 * sizeof(long)) - 1) / (8 * sizeof(long)))];
 unsigned long mscbit[((((0x07 +1)) + (8 * sizeof(long)) - 1) / (8 * sizeof(long)))];
 unsigned long ledbit[((((0x0f +1)) + (8 * sizeof(long)) - 1) / (8 * sizeof(long)))];
 unsigned long sndbit[((((0x07 +1)) + (8 * sizeof(long)) - 1) / (8 * sizeof(long)))];
 unsigned long ffbit[((((0x7f +1)) + (8 * sizeof(long)) - 1) / (8 * sizeof(long)))];
 unsigned long swbit[((((0x0f +1)) + (8 * sizeof(long)) - 1) / (8 * sizeof(long)))];

 unsigned int hint_events_per_packet;

 unsigned int keycodemax;
 unsigned int keycodesize;
 void *keycode;

 int (*setkeycode)(struct input_dev *dev,
     const struct input_keymap_entry *ke,
     unsigned int *old_keycode);
 int (*getkeycode)(struct input_dev *dev,
     struct input_keymap_entry *ke);

 struct ff_device *ff;

 unsigned int repeat_key;
 struct timer_list timer;

 int rep[(0x01 +1)];

 struct input_mt *mt;

 struct input_absinfo *absinfo;

 unsigned long key[((((0x2ff +1)) + (8 * sizeof(long)) - 1) / (8 * sizeof(long)))];
 unsigned long led[((((0x0f +1)) + (8 * sizeof(long)) - 1) / (8 * sizeof(long)))];
 unsigned long snd[((((0x07 +1)) + (8 * sizeof(long)) - 1) / (8 * sizeof(long)))];
 unsigned long sw[((((0x0f +1)) + (8 * sizeof(long)) - 1) / (8 * sizeof(long)))];

 int (*open)(struct input_dev *dev);
 void (*close)(struct input_dev *dev);
 int (*flush)(struct input_dev *dev, struct file *file);
 int (*event)(struct input_dev *dev, unsigned int type, unsigned int code, int value);

 struct input_handle *grab;

 spinlock_t event_lock;
 struct mutex mutex;

 unsigned int users;
 bool going_away;

 struct device dev;

 struct list_head h_list;
 struct list_head node;

 unsigned int num_vals;
 unsigned int max_vals;
 struct input_value *vals;

 bool devres_managed;
};
struct input_handle;
struct input_handler {

 void *private;

 void (*event)(struct input_handle *handle, unsigned int type, unsigned int code, int value);
 void (*events)(struct input_handle *handle,
         const struct input_value *vals, unsigned int count);
 bool (*filter)(struct input_handle *handle, unsigned int type, unsigned int code, int value);
 bool (*match)(struct input_handler *handler, struct input_dev *dev);
 int (*connect)(struct input_handler *handler, struct input_dev *dev, const struct input_device_id *id);
 void (*disconnect)(struct input_handle *handle);
 void (*start)(struct input_handle *handle);

 bool legacy_minors;
 int minor;
 const char *name;

 const struct input_device_id *id_table;

 struct list_head h_list;
 struct list_head node;
};
struct input_handle {

 void *private;

 int open;
 const char *name;

 struct input_dev *dev;
 struct input_handler *handler;

 struct list_head d_node;
 struct list_head h_node;
};

struct input_dev *input_allocate_device(void);
struct input_dev *devm_input_allocate_device(struct device *);
void input_free_device(struct input_dev *dev);

static inline __attribute__((no_instrument_function)) struct input_dev *input_get_device(struct input_dev *dev)
{
 return dev ? ({ const typeof( ((struct input_dev *)0)->dev ) *__mptr = (get_device(&dev->dev)); (struct input_dev *)( (char *)__mptr - __builtin_offsetof(struct input_dev,dev) );}) : ((void *)0);
}

static inline __attribute__((no_instrument_function)) void input_put_device(struct input_dev *dev)
{
 if (dev)
  put_device(&dev->dev);
}

static inline __attribute__((no_instrument_function)) void *input_get_drvdata(struct input_dev *dev)
{
 return dev_get_drvdata(&dev->dev);
}

static inline __attribute__((no_instrument_function)) void input_set_drvdata(struct input_dev *dev, void *data)
{
 dev_set_drvdata(&dev->dev, data);
}

int input_register_device(struct input_dev *);
void input_unregister_device(struct input_dev *);

void input_reset_device(struct input_dev *);

int input_register_handler(struct input_handler *);
void input_unregister_handler(struct input_handler *);

int input_get_new_minor(int legacy_base, unsigned int legacy_num,
         bool allow_dynamic);
void input_free_minor(unsigned int minor);

int input_handler_for_each_handle(struct input_handler *, void *data,
      int (*fn)(struct input_handle *, void *));

int input_register_handle(struct input_handle *);
void input_unregister_handle(struct input_handle *);

int input_grab_device(struct input_handle *);
void input_release_device(struct input_handle *);

int input_open_device(struct input_handle *);
void input_close_device(struct input_handle *);

int input_flush_device(struct input_handle *handle, struct file *file);

void input_event(struct input_dev *dev, unsigned int type, unsigned int code, int value);
void input_inject_event(struct input_handle *handle, unsigned int type, unsigned int code, int value);

static inline __attribute__((no_instrument_function)) void input_report_key(struct input_dev *dev, unsigned int code, int value)
{
 input_event(dev, 0x01, code, !!value);
}

static inline __attribute__((no_instrument_function)) void input_report_rel(struct input_dev *dev, unsigned int code, int value)
{
 input_event(dev, 0x02, code, value);
}

static inline __attribute__((no_instrument_function)) void input_report_abs(struct input_dev *dev, unsigned int code, int value)
{
 input_event(dev, 0x03, code, value);
}

static inline __attribute__((no_instrument_function)) void input_report_ff_status(struct input_dev *dev, unsigned int code, int value)
{
 input_event(dev, 0x17, code, value);
}

static inline __attribute__((no_instrument_function)) void input_report_switch(struct input_dev *dev, unsigned int code, int value)
{
 input_event(dev, 0x05, code, !!value);
}

static inline __attribute__((no_instrument_function)) void input_sync(struct input_dev *dev)
{
 input_event(dev, 0x00, 0, 0);
}

static inline __attribute__((no_instrument_function)) void input_mt_sync(struct input_dev *dev)
{
 input_event(dev, 0x00, 2, 0);
}

void input_set_capability(struct input_dev *dev, unsigned int type, unsigned int code);
static inline __attribute__((no_instrument_function)) void input_set_events_per_packet(struct input_dev *dev, int n_events)
{
 dev->hint_events_per_packet = n_events;
}

void input_alloc_absinfo(struct input_dev *dev);
void input_set_abs_params(struct input_dev *dev, unsigned int axis,
     int min, int max, int fuzz, int flat);
static inline __attribute__((no_instrument_function)) int input_abs_get_val(struct input_dev *dev, unsigned int axis) { return dev->absinfo ? dev->absinfo[axis].value : 0; } static inline __attribute__((no_instrument_function)) void input_abs_set_val(struct input_dev *dev, unsigned int axis, int val) { input_alloc_absinfo(dev); if (dev->absinfo) dev->absinfo[axis].value = val; }
static inline __attribute__((no_instrument_function)) int input_abs_get_min(struct input_dev *dev, unsigned int axis) { return dev->absinfo ? dev->absinfo[axis].minimum : 0; } static inline __attribute__((no_instrument_function)) void input_abs_set_min(struct input_dev *dev, unsigned int axis, int val) { input_alloc_absinfo(dev); if (dev->absinfo) dev->absinfo[axis].minimum = val; }
static inline __attribute__((no_instrument_function)) int input_abs_get_max(struct input_dev *dev, unsigned int axis) { return dev->absinfo ? dev->absinfo[axis].maximum : 0; } static inline __attribute__((no_instrument_function)) void input_abs_set_max(struct input_dev *dev, unsigned int axis, int val) { input_alloc_absinfo(dev); if (dev->absinfo) dev->absinfo[axis].maximum = val; }
static inline __attribute__((no_instrument_function)) int input_abs_get_fuzz(struct input_dev *dev, unsigned int axis) { return dev->absinfo ? dev->absinfo[axis].fuzz : 0; } static inline __attribute__((no_instrument_function)) void input_abs_set_fuzz(struct input_dev *dev, unsigned int axis, int val) { input_alloc_absinfo(dev); if (dev->absinfo) dev->absinfo[axis].fuzz = val; }
static inline __attribute__((no_instrument_function)) int input_abs_get_flat(struct input_dev *dev, unsigned int axis) { return dev->absinfo ? dev->absinfo[axis].flat : 0; } static inline __attribute__((no_instrument_function)) void input_abs_set_flat(struct input_dev *dev, unsigned int axis, int val) { input_alloc_absinfo(dev); if (dev->absinfo) dev->absinfo[axis].flat = val; }
static inline __attribute__((no_instrument_function)) int input_abs_get_res(struct input_dev *dev, unsigned int axis) { return dev->absinfo ? dev->absinfo[axis].resolution : 0; } static inline __attribute__((no_instrument_function)) void input_abs_set_res(struct input_dev *dev, unsigned int axis, int val) { input_alloc_absinfo(dev); if (dev->absinfo) dev->absinfo[axis].resolution = val; }

int input_scancode_to_scalar(const struct input_keymap_entry *ke,
        unsigned int *scancode);

int input_get_keycode(struct input_dev *dev, struct input_keymap_entry *ke);
int input_set_keycode(struct input_dev *dev,
        const struct input_keymap_entry *ke);

extern struct class input_class;
struct ff_device {
 int (*upload)(struct input_dev *dev, struct ff_effect *effect,
        struct ff_effect *old);
 int (*erase)(struct input_dev *dev, int effect_id);

 int (*playback)(struct input_dev *dev, int effect_id, int value);
 void (*set_gain)(struct input_dev *dev, u16 gain);
 void (*set_autocenter)(struct input_dev *dev, u16 magnitude);

 void (*destroy)(struct ff_device *);

 void *private;

 unsigned long ffbit[((((0x7f +1)) + (8 * sizeof(long)) - 1) / (8 * sizeof(long)))];

 struct mutex mutex;

 int max_effects;
 struct ff_effect *effects;
 struct file *effect_owners[];
};

int input_ff_create(struct input_dev *dev, unsigned int max_effects);
void input_ff_destroy(struct input_dev *dev);

int input_ff_event(struct input_dev *dev, unsigned int type, unsigned int code, int value);

int input_ff_upload(struct input_dev *dev, struct ff_effect *effect, struct file *file);
int input_ff_erase(struct input_dev *dev, int effect_id, struct file *file);

int input_ff_create_memless(struct input_dev *dev, void *data,
  int (*play_effect)(struct input_dev *, void *, struct ff_effect *));

struct serio {
 void *port_data;

 char name[32];
 char phys[32];
 char firmware_id[128];

 bool manual_bind;

 struct serio_device_id id;

 spinlock_t lock;

 int (*write)(struct serio *, unsigned char);
 int (*open)(struct serio *);
 void (*close)(struct serio *);
 int (*start)(struct serio *);
 void (*stop)(struct serio *);

 struct serio *parent;
 struct list_head child_node;
 struct list_head children;
 unsigned int depth;

 struct serio_driver *drv;
 struct mutex drv_mutex;

 struct device dev;

 struct list_head node;
};


struct serio_driver {
 const char *description;

 const struct serio_device_id *id_table;
 bool manual_bind;

 void (*write_wakeup)(struct serio *);
 irqreturn_t (*interrupt)(struct serio *, unsigned char, unsigned int);
 int (*connect)(struct serio *, struct serio_driver *drv);
 int (*reconnect)(struct serio *);
 void (*disconnect)(struct serio *);
 void (*cleanup)(struct serio *);

 struct device_driver driver;
};


int serio_open(struct serio *serio, struct serio_driver *drv);
void serio_close(struct serio *serio);
void serio_rescan(struct serio *serio);
void serio_reconnect(struct serio *serio);
irqreturn_t serio_interrupt(struct serio *serio, unsigned char data, unsigned int flags);

void __serio_register_port(struct serio *serio, struct module *owner);





void serio_unregister_port(struct serio *serio);
void serio_unregister_child_port(struct serio *serio);

int __serio_register_driver(struct serio_driver *drv,
    struct module *owner, const char *mod_name);





void serio_unregister_driver(struct serio_driver *drv);
static inline __attribute__((no_instrument_function)) int serio_write(struct serio *serio, unsigned char data)
{
 if (serio->write)
  return serio->write(serio, data);
 else
  return -1;
}

static inline __attribute__((no_instrument_function)) void serio_drv_write_wakeup(struct serio *serio)
{
 if (serio->drv && serio->drv->write_wakeup)
  serio->drv->write_wakeup(serio);
}





static inline __attribute__((no_instrument_function)) void *serio_get_drvdata(struct serio *serio)
{
 return dev_get_drvdata(&serio->dev);
}

static inline __attribute__((no_instrument_function)) void serio_set_drvdata(struct serio *serio, void *data)
{
 dev_set_drvdata(&serio->dev, data);
}





static inline __attribute__((no_instrument_function)) void serio_pause_rx(struct serio *serio)
{
 spin_lock_irq(&serio->lock);
}

static inline __attribute__((no_instrument_function)) void serio_continue_rx(struct serio *serio)
{
 spin_unlock_irq(&serio->lock);
}

struct ps2dev {
 struct serio *serio;


 struct mutex cmd_mutex;


 wait_queue_head_t wait;

 unsigned long flags;
 unsigned char cmdbuf[8];
 unsigned char cmdcnt;
 unsigned char nak;
};

void ps2_init(struct ps2dev *ps2dev, struct serio *serio);
int ps2_sendbyte(struct ps2dev *ps2dev, unsigned char byte, int timeout);
void ps2_drain(struct ps2dev *ps2dev, int maxbytes, int timeout);
void ps2_begin_command(struct ps2dev *ps2dev);
void ps2_end_command(struct ps2dev *ps2dev);
int __ps2_command(struct ps2dev *ps2dev, unsigned char *param, int command);
int ps2_command(struct ps2dev *ps2dev, unsigned char *param, int command);
int ps2_handle_ack(struct ps2dev *ps2dev, unsigned char data);
int ps2_handle_response(struct ps2dev *ps2dev, unsigned char data);
void ps2_cmd_aborted(struct ps2dev *ps2dev);
int ps2_is_keyboard_id(char id);


enum psmouse_state {
 PSMOUSE_IGNORE,
 PSMOUSE_INITIALIZING,
 PSMOUSE_RESYNCING,
 PSMOUSE_CMD_MODE,
 PSMOUSE_ACTIVATED,
};


typedef enum {
 PSMOUSE_BAD_DATA,
 PSMOUSE_GOOD_DATA,
 PSMOUSE_FULL_PACKET
} psmouse_ret_t;

struct psmouse {
 void *private;
 struct input_dev *dev;
 struct ps2dev ps2dev;
 struct delayed_work resync_work;
 char *vendor;
 char *name;
 unsigned char packet[8];
 unsigned char badbyte;
 unsigned char pktcnt;
 unsigned char pktsize;
 unsigned char type;
 bool ignore_parity;
 bool acks_disable_command;
 unsigned int model;
 unsigned long last;
 unsigned long out_of_sync_cnt;
 unsigned long num_resyncs;
 enum psmouse_state state;
 char devname[64];
 char phys[32];

 unsigned int rate;
 unsigned int resolution;
 unsigned int resetafter;
 unsigned int resync_time;
 bool smartscroll;

 psmouse_ret_t (*protocol_handler)(struct psmouse *psmouse);
 void (*set_rate)(struct psmouse *psmouse, unsigned int rate);
 void (*set_resolution)(struct psmouse *psmouse, unsigned int resolution);

 int (*reconnect)(struct psmouse *psmouse);
 void (*disconnect)(struct psmouse *psmouse);
 void (*cleanup)(struct psmouse *psmouse);
 int (*poll)(struct psmouse *psmouse);

 void (*pt_activate)(struct psmouse *psmouse);
 void (*pt_deactivate)(struct psmouse *psmouse);
};

enum psmouse_type {
 PSMOUSE_NONE,
 PSMOUSE_PS2,
 PSMOUSE_PS2PP,
 PSMOUSE_THINKPS,
 PSMOUSE_GENPS,
 PSMOUSE_IMPS,
 PSMOUSE_IMEX,
 PSMOUSE_SYNAPTICS,
 PSMOUSE_ALPS,
 PSMOUSE_LIFEBOOK,
 PSMOUSE_TRACKPOINT,
 PSMOUSE_TOUCHKIT_PS2,
 PSMOUSE_CORTRON,
 PSMOUSE_HGPK,
 PSMOUSE_ELANTECH,
 PSMOUSE_FSP,
 PSMOUSE_SYNAPTICS_RELATIVE,
 PSMOUSE_CYPRESS,
 PSMOUSE_AUTO
};

void psmouse_queue_work(struct psmouse *psmouse, struct delayed_work *work,
  unsigned long delay);
int psmouse_sliced_command(struct psmouse *psmouse, unsigned char command);
int psmouse_reset(struct psmouse *psmouse);
void psmouse_set_state(struct psmouse *psmouse, enum psmouse_state new_state);
void psmouse_set_resolution(struct psmouse *psmouse, unsigned int resolution);
psmouse_ret_t psmouse_process_byte(struct psmouse *psmouse);
int psmouse_activate(struct psmouse *psmouse);
int psmouse_deactivate(struct psmouse *psmouse);

struct psmouse_attribute {
 struct device_attribute dattr;
 void *data;
 ssize_t (*show)(struct psmouse *psmouse, void *data, char *buf);
 ssize_t (*set)(struct psmouse *psmouse, void *data,
   const char *buf, size_t count);
 bool protect;
};


ssize_t psmouse_attr_show_helper(struct device *dev, struct device_attribute *attr,
     char *buf);
ssize_t psmouse_attr_set_helper(struct device *dev, struct device_attribute *attr,
    const char *buf, size_t count);
struct synaptics_mt_state {
 int count;
 int sgm;
 int agm;
};




struct synaptics_hw_state {
 int x;
 int y;
 int z;
 int w;
 unsigned int left:1;
 unsigned int right:1;
 unsigned int middle:1;
 unsigned int up:1;
 unsigned int down:1;
 unsigned char ext_buttons;
 signed char scroll;


 struct synaptics_mt_state mt_state;
};

struct synaptics_data {

 unsigned long int model_id;
 unsigned long int firmware_id;
 unsigned long int board_id;
 unsigned long int capabilities;
 unsigned long int ext_cap;
 unsigned long int ext_cap_0c;
 unsigned long int identity;
 unsigned int x_res, y_res;
 unsigned int x_max, y_max;
 unsigned int x_min, y_min;

 unsigned char pkt_type;
 unsigned char mode;
 int scroll;

 bool absolute_mode;
 bool disable_gesture;

 struct serio *pt_port;

 struct synaptics_mt_state mt_state;
 bool mt_state_lost;





 struct synaptics_hw_state agm;
 bool agm_pending;


 unsigned long press_start;
 bool press;
 bool report_press;
};

void synaptics_module_init(void);
int synaptics_detect(struct psmouse *psmouse, bool set_properties);
int synaptics_init(struct psmouse *psmouse);
int synaptics_init_relative(struct psmouse *psmouse);
void synaptics_reset(struct psmouse *psmouse);
bool synaptics_supported(void);
int ps2pp_init(struct psmouse *psmouse, bool set_properties);
struct input_mt_slot {
 int abs[0x3d - 0x30 + 1];
 unsigned int frame;
 unsigned int key;
};
struct input_mt {
 int trkid;
 int num_slots;
 int slot;
 unsigned int flags;
 unsigned int frame;
 int *red;
 struct input_mt_slot slots[];
};

static inline __attribute__((no_instrument_function)) void input_mt_set_value(struct input_mt_slot *slot,
          unsigned code, int value)
{
 slot->abs[code - 0x30] = value;
}

static inline __attribute__((no_instrument_function)) int input_mt_get_value(const struct input_mt_slot *slot,
         unsigned code)
{
 return slot->abs[code - 0x30];
}

static inline __attribute__((no_instrument_function)) bool input_mt_is_active(const struct input_mt_slot *slot)
{
 return input_mt_get_value(slot, 0x39) >= 0;
}

static inline __attribute__((no_instrument_function)) bool input_mt_is_used(const struct input_mt *mt,
        const struct input_mt_slot *slot)
{
 return slot->frame == mt->frame;
}

int input_mt_init_slots(struct input_dev *dev, unsigned int num_slots,
   unsigned int flags);
void input_mt_destroy_slots(struct input_dev *dev);

static inline __attribute__((no_instrument_function)) int input_mt_new_trkid(struct input_mt *mt)
{
 return mt->trkid++ & 0xffff;
}

static inline __attribute__((no_instrument_function)) void input_mt_slot(struct input_dev *dev, int slot)
{
 input_event(dev, 0x03, 0x2f, slot);
}

static inline __attribute__((no_instrument_function)) bool input_is_mt_value(int axis)
{
 return axis >= 0x30 && axis <= 0x3d;
}

static inline __attribute__((no_instrument_function)) bool input_is_mt_axis(int axis)
{
 return axis == 0x2f || input_is_mt_value(axis);
}

void input_mt_report_slot_state(struct input_dev *dev,
    unsigned int tool_type, bool active);

void input_mt_report_finger_count(struct input_dev *dev, int count);
void input_mt_report_pointer_emulation(struct input_dev *dev, bool use_count);
void input_mt_drop_unused(struct input_dev *dev);

void input_mt_sync_frame(struct input_dev *dev);






struct input_mt_pos {
 s16 x, y;
};

int input_mt_assign_slots(struct input_dev *dev, int *slots,
     const struct input_mt_pos *pos, int num_pos);

int input_mt_get_slot_by_key(struct input_dev *dev, int key);
enum V7_PACKET_ID {
  V7_PACKET_ID_IDLE,
  V7_PACKET_ID_TWO,
  V7_PACKET_ID_MULTI,
  V7_PACKET_ID_NEW,
  V7_PACKET_ID_UNKNOWN,
};
struct alps_model_info {
 unsigned char signature[3];
 unsigned char command_mode_resp;
 unsigned char proto_version;
 unsigned char byte0, mask0;
 int flags;
};
struct alps_nibble_commands {
 int command;
 unsigned char data;
};

struct alps_bitmap_point {
 int start_bit;
 int num_bits;
};
struct alps_fields {
 unsigned int x_map;
 unsigned int y_map;
 unsigned int fingers;

 int pressure;
 struct input_mt_pos st;
 struct input_mt_pos mt[2];

 unsigned int first_mp:1;
 unsigned int is_mp:1;

 unsigned int left:1;
 unsigned int right:1;
 unsigned int middle:1;

 unsigned int ts_left:1;
 unsigned int ts_right:1;
 unsigned int ts_middle:1;
};
struct alps_data {
 struct input_dev *dev2;
 char phys[32];


 const struct alps_nibble_commands *nibble_commands;
 int addr_command;
 unsigned char proto_version;
 unsigned char byte0, mask0;
 unsigned char fw_ver[3];
 int flags;
 int x_max;
 int y_max;
 int x_bits;
 int y_bits;
 unsigned int x_res;
 unsigned int y_res;

 int (*hw_init)(struct psmouse *psmouse);
 void (*process_packet)(struct psmouse *psmouse);
 int (*decode_fields)(struct alps_fields *f, unsigned char *p,
         struct psmouse *psmouse);
 void (*set_abs_params)(struct alps_data *priv, struct input_dev *dev1);

 int prev_fin;
 int multi_packet;
 unsigned char multi_data[6];
 struct alps_fields f;
 u8 quirks;
 struct timer_list timer;
};




int alps_detect(struct psmouse *psmouse, bool set_properties);
int alps_init(struct psmouse *psmouse);
enum hgpk_model_t {
 HGPK_MODEL_PREA = 0x0a,
 HGPK_MODEL_A = 0x14,
 HGPK_MODEL_B = 0x28,
 HGPK_MODEL_C = 0x3c,
 HGPK_MODEL_D = 0x50,
};

enum hgpk_spew_flag {
 NO_SPEW,
 MAYBE_SPEWING,
 SPEW_DETECTED,
 RECALIBRATING,
};



enum hgpk_mode {
 HGPK_MODE_MOUSE,
 HGPK_MODE_GLIDESENSOR,
 HGPK_MODE_PENTABLET,
 HGPK_MODE_INVALID
};

struct hgpk_data {
 struct psmouse *psmouse;
 enum hgpk_mode mode;
 bool powered;
 enum hgpk_spew_flag spew_flag;
 int spew_count, x_tally, y_tally;
 unsigned long recalib_window;
 struct delayed_work recalib_wq;
 int abs_x, abs_y;
 int dupe_count;
 int xbigj, ybigj, xlast, ylast;
 int xsaw_secondary, ysaw_secondary;
};






static inline __attribute__((no_instrument_function)) void hgpk_module_init(void)
{
}
static inline __attribute__((no_instrument_function)) int hgpk_detect(struct psmouse *psmouse, bool set_properties)
{
 return -19;
}
static inline __attribute__((no_instrument_function)) int hgpk_init(struct psmouse *psmouse)
{
 return -19;
}
void lifebook_module_init(void);
int lifebook_detect(struct psmouse *psmouse, bool set_properties);
int lifebook_init(struct psmouse *psmouse);
struct trackpoint_data
{
 unsigned char sensitivity, speed, inertia, reach;
 unsigned char draghys, mindrag;
 unsigned char thresh, upthresh;
 unsigned char ztime, jenks;


 unsigned char press_to_select;
 unsigned char skipback;
 unsigned char ext_dev;
};


int trackpoint_detect(struct psmouse *psmouse, bool set_properties);
static inline __attribute__((no_instrument_function)) int touchkit_ps2_detect(struct psmouse *psmouse,
          bool set_properties)
{
 return -38;
}
struct finger_pos {
 unsigned int x;
 unsigned int y;
};

struct elantech_data {
 struct input_dev *tp_dev;
 char tp_phys[32];
 unsigned char reg_07;
 unsigned char reg_10;
 unsigned char reg_11;
 unsigned char reg_20;
 unsigned char reg_21;
 unsigned char reg_22;
 unsigned char reg_23;
 unsigned char reg_24;
 unsigned char reg_25;
 unsigned char reg_26;
 unsigned char debug;
 unsigned char capabilities[3];
 bool paritycheck;
 bool jumpy_cursor;
 bool reports_pressure;
 bool crc_enabled;
 bool set_hw_resolution;
 unsigned char hw_version;
 unsigned int fw_version;
 unsigned int single_finger_reports;
 unsigned int y_max;
 unsigned int width;
 struct finger_pos mt[5];
 unsigned char parity[256];
 int (*send_cmd)(struct psmouse *psmouse, unsigned char c, unsigned char *param);
};


int elantech_detect(struct psmouse *psmouse, bool set_properties);
int elantech_init(struct psmouse *psmouse);
struct fsp_data {
 unsigned char ver;
 unsigned char rev;
 unsigned int buttons;
 unsigned int flags;


 bool vscroll;
 bool hscroll;

 unsigned char last_reg;
 unsigned char last_val;
 unsigned int last_mt_fgr;
};


extern int fsp_detect(struct psmouse *psmouse, bool set_properties);
extern int fsp_init(struct psmouse *psmouse);
struct cytp_contact {
 int x;
 int y;
 int z;
};


struct cytp_report_data {
 int contact_cnt;
 struct cytp_contact contacts[2];
 unsigned int left:1;
 unsigned int right:1;
 unsigned int middle:1;
 unsigned int tap:1;
};


struct cytp_data {
 int fw_version;

 int pkt_size;
 int mode;

 int tp_min_pressure;
 int tp_max_pressure;
 int tp_width;
 int tp_high;
 int tp_max_abs_x;
 int tp_max_abs_y;

 int tp_res_x;
 int tp_res_y;

 int tp_metrics_supported;
};



int cypress_detect(struct psmouse *psmouse, bool set_properties);
int cypress_init(struct psmouse *psmouse);
bool cypress_supported(void);



static const char __UNIQUE_ID_author0[] __attribute__((__used__)) __attribute__((section(".modinfo"), unused, aligned(1))) = "author" "=" "Vojtech Pavlik <vojtech@suse.cz>";
static const char __UNIQUE_ID_description1[] __attribute__((__used__)) __attribute__((section(".modinfo"), unused, aligned(1))) = "description" "=" "PS/2 mouse driver";
static const char __UNIQUE_ID_license2[] __attribute__((__used__)) __attribute__((section(".modinfo"), unused, aligned(1))) = "license" "=" "GPL";

static unsigned int psmouse_max_proto = PSMOUSE_AUTO;
static int psmouse_set_maxproto(const char *val, const struct kernel_param *);
static int psmouse_get_maxproto(char *buffer, const struct kernel_param *kp);
static struct kernel_param_ops param_ops_proto_abbrev = {
 .set = psmouse_set_maxproto,
 .get = psmouse_get_maxproto,
};

static inline __attribute__((no_instrument_function)) unsigned int __attribute__((unused)) *__check_proto(void) { return(&(psmouse_max_proto)); }; static const char __param_str_proto[] = "proto"; static struct kernel_param const __param_proto __attribute__((__used__)) __attribute__ ((unused,__section__ ("__param"),aligned(sizeof(void *)))) = { __param_str_proto, &param_ops_proto_abbrev, ((sizeof(struct { int:-!!((0644) < 0); })) + (sizeof(struct { int:-!!((0644) > 0777); })) + (sizeof(struct { int:-!!(((0644) >> 6) < (((0644) >> 3) & 7)); })) + (sizeof(struct { int:-!!((((0644) >> 3) & 7) < ((0644) & 7)); })) + (sizeof(struct { int:-!!((0644) & 2); })) + (0644)), -1, { &psmouse_max_proto } }; static const char __UNIQUE_ID_prototype3[] __attribute__((__used__)) __attribute__((section(".modinfo"), unused, aligned(1))) = "parmtype" "=" "proto" ":" "proto_abbrev";
static const char __UNIQUE_ID_proto4[] __attribute__((__used__)) __attribute__((section(".modinfo"), unused, aligned(1))) = "parm" "=" "proto" ":" "Highest protocol extension to probe (bare, imps, exps, any). Useful for KVM switches.";

static unsigned int psmouse_resolution = 200;
static inline __attribute__((no_instrument_function)) unsigned int __attribute__((unused)) *__check_resolution(void) { return(&(psmouse_resolution)); }; static const char __param_str_resolution[] = "resolution"; static struct kernel_param const __param_resolution __attribute__((__used__)) __attribute__ ((unused,__section__ ("__param"),aligned(sizeof(void *)))) = { __param_str_resolution, &param_ops_uint, ((sizeof(struct { int:-!!((0644) < 0); })) + (sizeof(struct { int:-!!((0644) > 0777); })) + (sizeof(struct { int:-!!(((0644) >> 6) < (((0644) >> 3) & 7)); })) + (sizeof(struct { int:-!!((((0644) >> 3) & 7) < ((0644) & 7)); })) + (sizeof(struct { int:-!!((0644) & 2); })) + (0644)), -1, { &psmouse_resolution } }; static const char __UNIQUE_ID_resolutiontype5[] __attribute__((__used__)) __attribute__((section(".modinfo"), unused, aligned(1))) = "parmtype" "=" "resolution" ":" "uint";
static const char __UNIQUE_ID_resolution6[] __attribute__((__used__)) __attribute__((section(".modinfo"), unused, aligned(1))) = "parm" "=" "resolution" ":" "Resolution, in dpi.";

static unsigned int psmouse_rate = 100;
static inline __attribute__((no_instrument_function)) unsigned int __attribute__((unused)) *__check_rate(void) { return(&(psmouse_rate)); }; static const char __param_str_rate[] = "rate"; static struct kernel_param const __param_rate __attribute__((__used__)) __attribute__ ((unused,__section__ ("__param"),aligned(sizeof(void *)))) = { __param_str_rate, &param_ops_uint, ((sizeof(struct { int:-!!((0644) < 0); })) + (sizeof(struct { int:-!!((0644) > 0777); })) + (sizeof(struct { int:-!!(((0644) >> 6) < (((0644) >> 3) & 7)); })) + (sizeof(struct { int:-!!((((0644) >> 3) & 7) < ((0644) & 7)); })) + (sizeof(struct { int:-!!((0644) & 2); })) + (0644)), -1, { &psmouse_rate } }; static const char __UNIQUE_ID_ratetype7[] __attribute__((__used__)) __attribute__((section(".modinfo"), unused, aligned(1))) = "parmtype" "=" "rate" ":" "uint";
static const char __UNIQUE_ID_rate8[] __attribute__((__used__)) __attribute__((section(".modinfo"), unused, aligned(1))) = "parm" "=" "rate" ":" "Report rate, in reports per second.";

static bool psmouse_smartscroll = 1;
static inline __attribute__((no_instrument_function)) bool __attribute__((unused)) *__check_smartscroll(void) { return(&(psmouse_smartscroll)); }; static const char __param_str_smartscroll[] = "smartscroll"; static struct kernel_param const __param_smartscroll __attribute__((__used__)) __attribute__ ((unused,__section__ ("__param"),aligned(sizeof(void *)))) = { __param_str_smartscroll, &param_ops_bool, ((sizeof(struct { int:-!!((0644) < 0); })) + (sizeof(struct { int:-!!((0644) > 0777); })) + (sizeof(struct { int:-!!(((0644) >> 6) < (((0644) >> 3) & 7)); })) + (sizeof(struct { int:-!!((((0644) >> 3) & 7) < ((0644) & 7)); })) + (sizeof(struct { int:-!!((0644) & 2); })) + (0644)), -1, { &psmouse_smartscroll } }; static const char __UNIQUE_ID_smartscrolltype9[] __attribute__((__used__)) __attribute__((section(".modinfo"), unused, aligned(1))) = "parmtype" "=" "smartscroll" ":" "bool";
static const char __UNIQUE_ID_smartscroll10[] __attribute__((__used__)) __attribute__((section(".modinfo"), unused, aligned(1))) = "parm" "=" "smartscroll" ":" "Logitech Smartscroll autorepeat, 1 = enabled (default), 0 = disabled.";

static unsigned int psmouse_resetafter = 5;
static inline __attribute__((no_instrument_function)) unsigned int __attribute__((unused)) *__check_resetafter(void) { return(&(psmouse_resetafter)); }; static const char __param_str_resetafter[] = "resetafter"; static struct kernel_param const __param_resetafter __attribute__((__used__)) __attribute__ ((unused,__section__ ("__param"),aligned(sizeof(void *)))) = { __param_str_resetafter, &param_ops_uint, ((sizeof(struct { int:-!!((0644) < 0); })) + (sizeof(struct { int:-!!((0644) > 0777); })) + (sizeof(struct { int:-!!(((0644) >> 6) < (((0644) >> 3) & 7)); })) + (sizeof(struct { int:-!!((((0644) >> 3) & 7) < ((0644) & 7)); })) + (sizeof(struct { int:-!!((0644) & 2); })) + (0644)), -1, { &psmouse_resetafter } }; static const char __UNIQUE_ID_resetaftertype11[] __attribute__((__used__)) __attribute__((section(".modinfo"), unused, aligned(1))) = "parmtype" "=" "resetafter" ":" "uint";
static const char __UNIQUE_ID_resetafter12[] __attribute__((__used__)) __attribute__((section(".modinfo"), unused, aligned(1))) = "parm" "=" "resetafter" ":" "Reset device after so many bad packets (0 = never).";

static unsigned int psmouse_resync_time;
static inline __attribute__((no_instrument_function)) unsigned int __attribute__((unused)) *__check_resync_time(void) { return(&(psmouse_resync_time)); }; static const char __param_str_resync_time[] = "resync_time"; static struct kernel_param const __param_resync_time __attribute__((__used__)) __attribute__ ((unused,__section__ ("__param"),aligned(sizeof(void *)))) = { __param_str_resync_time, &param_ops_uint, ((sizeof(struct { int:-!!((0644) < 0); })) + (sizeof(struct { int:-!!((0644) > 0777); })) + (sizeof(struct { int:-!!(((0644) >> 6) < (((0644) >> 3) & 7)); })) + (sizeof(struct { int:-!!((((0644) >> 3) & 7) < ((0644) & 7)); })) + (sizeof(struct { int:-!!((0644) & 2); })) + (0644)), -1, { &psmouse_resync_time } }; static const char __UNIQUE_ID_resync_timetype13[] __attribute__((__used__)) __attribute__((section(".modinfo"), unused, aligned(1))) = "parmtype" "=" "resync_time" ":" "uint";
static const char __UNIQUE_ID_resync_time14[] __attribute__((__used__)) __attribute__((section(".modinfo"), unused, aligned(1))) = "parm" "=" "resync_time" ":" "How long can mouse stay idle before forcing resync (in seconds, 0 = never).";

static ssize_t psmouse_attr_show_protocol(struct psmouse *, void *, char *); static ssize_t psmouse_attr_set_protocol(struct psmouse *, void *, const char *, size_t); static struct psmouse_attribute

 psmouse_attr_protocol
 = { .dattr = { .attr = { .name = "protocol", .mode = 00200 | (00400|00040|00004), }, .show = psmouse_attr_show_helper, .store = psmouse_attr_set_helper, }, .data = ((void *)0), .show = psmouse_attr_show_protocol, .set = psmouse_attr_set_protocol, .protect = true, }

                                                         ;
static ssize_t psmouse_show_int_attr(struct psmouse *, void *, char *); static ssize_t psmouse_attr_set_rate(struct psmouse *, void *, const char *, size_t); static struct psmouse_attribute

 psmouse_attr_rate
 = { .dattr = { .attr = { .name = "rate", .mode = 00200 | (00400|00040|00004), }, .show = psmouse_attr_show_helper, .store = psmouse_attr_set_helper, }, .data = (void *) __builtin_offsetof(struct psmouse,rate), .show = psmouse_show_int_attr, .set = psmouse_attr_set_rate, .protect = true, }

                                                ;
static ssize_t psmouse_show_int_attr(struct psmouse *, void *, char *); static ssize_t psmouse_attr_set_resolution(struct psmouse *, void *, const char *, size_t); static struct psmouse_attribute

 psmouse_attr_resolution
 = { .dattr = { .attr = { .name = "resolution", .mode = 00200 | (00400|00040|00004), }, .show = psmouse_attr_show_helper, .store = psmouse_attr_set_helper, }, .data = (void *) __builtin_offsetof(struct psmouse,resolution), .show = psmouse_show_int_attr, .set = psmouse_attr_set_resolution, .protect = true, }

                                                      ;
static ssize_t psmouse_show_int_attr(struct psmouse *, void *, char *); static ssize_t psmouse_set_int_attr(struct psmouse *, void *, const char *, size_t); static struct psmouse_attribute

 psmouse_attr_resetafter
 = { .dattr = { .attr = { .name = "resetafter", .mode = 00200 | (00400|00040|00004), }, .show = psmouse_attr_show_helper, .store = psmouse_attr_set_helper, }, .data = (void *) __builtin_offsetof(struct psmouse,resetafter), .show = psmouse_show_int_attr, .set = psmouse_set_int_attr, .protect = true, }

                                               ;
static ssize_t psmouse_show_int_attr(struct psmouse *, void *, char *); static ssize_t psmouse_set_int_attr(struct psmouse *, void *, const char *, size_t); static struct psmouse_attribute

 psmouse_attr_resync_time
 = { .dattr = { .attr = { .name = "resync_time", .mode = 00200 | (00400|00040|00004), }, .show = psmouse_attr_show_helper, .store = psmouse_attr_set_helper, }, .data = (void *) __builtin_offsetof(struct psmouse,resync_time), .show = psmouse_show_int_attr, .set = psmouse_set_int_attr, .protect = true, }

                                               ;

static struct attribute *psmouse_attributes[] = {
 &psmouse_attr_protocol.dattr.attr,
 &psmouse_attr_rate.dattr.attr,
 &psmouse_attr_resolution.dattr.attr,
 &psmouse_attr_resetafter.dattr.attr,
 &psmouse_attr_resync_time.dattr.attr,
 ((void *)0)
};

static struct attribute_group psmouse_attribute_group = {
 .attrs = psmouse_attributes,
};
static struct mutex psmouse_mutex = { .count = { (1) } , .wait_lock = (spinlock_t ) { { .rlock = { .raw_lock = { { 0 } }, } } } , .wait_list = { &(psmouse_mutex.wait_list), &(psmouse_mutex.wait_list) } };

static struct workqueue_struct *kpsmoused_wq;

struct psmouse_protocol {
 enum psmouse_type type;
 bool maxproto;
 bool ignore_parity;
 const char *name;
 const char *alias;
 int (*detect)(struct psmouse *, bool);
 int (*init)(struct psmouse *);
};






psmouse_ret_t psmouse_process_byte(struct psmouse *psmouse)
{
 struct input_dev *dev = psmouse->dev;
 unsigned char *packet = psmouse->packet;

 if (psmouse->pktcnt < psmouse->pktsize)
  return PSMOUSE_GOOD_DATA;
 if (psmouse->type == PSMOUSE_IMPS || psmouse->type == PSMOUSE_GENPS)
  input_report_rel(dev, 0x08, -(signed char) packet[3]);





 if (psmouse->type == PSMOUSE_IMEX) {
  switch (packet[3] & 0xC0) {
  case 0x80:
   input_report_rel(dev, 0x08, (int) (packet[3] & 32) - (int) (packet[3] & 31));
   break;
  case 0x40:
   input_report_rel(dev, 0x06, (int) (packet[3] & 32) - (int) (packet[3] & 31));
   break;
  case 0x00:
  case 0xC0:
   input_report_rel(dev, 0x08, (int) (packet[3] & 8) - (int) (packet[3] & 7));
   input_report_key(dev, 0x113, (packet[3] >> 4) & 1);
   input_report_key(dev, 0x114, (packet[3] >> 5) & 1);
   break;
  }
 }





 if (psmouse->type == PSMOUSE_GENPS) {
  input_report_key(dev, 0x113, (packet[0] >> 6) & 1);
  input_report_key(dev, 0x114, (packet[0] >> 7) & 1);
 }




 if (psmouse->type == PSMOUSE_THINKPS) {
  input_report_key(dev, 0x114, (packet[0] >> 3) & 1);

  packet[1] |= (packet[0] & 0x40) << 1;
 }





 if (psmouse->type == PSMOUSE_CORTRON) {
  input_report_key(dev, 0x113, (packet[0] >> 3) & 1);
  packet[0] |= 0x08;
 }





 input_report_key(dev, 0x110, packet[0] & 1);
 input_report_key(dev, 0x112, (packet[0] >> 2) & 1);
 input_report_key(dev, 0x111, (packet[0] >> 1) & 1);

 input_report_rel(dev, 0x00, packet[1] ? (int) packet[1] - (int) ((packet[0] << 4) & 0x100) : 0);
 input_report_rel(dev, 0x01, packet[2] ? (int) ((packet[0] << 3) & 0x100) - (int) packet[2] : 0);

 input_sync(dev);

 return PSMOUSE_FULL_PACKET;
}

void psmouse_queue_work(struct psmouse *psmouse, struct delayed_work *work,
  unsigned long delay)
{
 queue_delayed_work(kpsmoused_wq, work, delay);
}





static inline __attribute__((no_instrument_function)) void __psmouse_set_state(struct psmouse *psmouse, enum psmouse_state new_state)
{
 psmouse->state = new_state;
 psmouse->pktcnt = psmouse->out_of_sync_cnt = 0;
 psmouse->ps2dev.flags = 0;
 psmouse->last = jiffies;
}
void psmouse_set_state(struct psmouse *psmouse, enum psmouse_state new_state)
{
 serio_pause_rx(psmouse->ps2dev.serio);
 __psmouse_set_state(psmouse, new_state);
 serio_continue_rx(psmouse->ps2dev.serio);
}






static int psmouse_handle_byte(struct psmouse *psmouse)
{
 psmouse_ret_t rc = psmouse->protocol_handler(psmouse);

 switch (rc) {
 case PSMOUSE_BAD_DATA:
  if (psmouse->state == PSMOUSE_ACTIVATED) {
   dev_warn(&(psmouse)->ps2dev.serio->dev, "%s at %s lost sync at byte %d\n", psmouse->name, psmouse->phys, psmouse->pktcnt)


                         ;
   if (++psmouse->out_of_sync_cnt == psmouse->resetafter) {
    __psmouse_set_state(psmouse, PSMOUSE_IGNORE);
    dev_notice(&(psmouse)->ps2dev.serio->dev, "issuing reconnect request\n")
                                    ;
    serio_reconnect(psmouse->ps2dev.serio);
    return -1;
   }
  }
  psmouse->pktcnt = 0;
  break;

 case PSMOUSE_FULL_PACKET:
  psmouse->pktcnt = 0;
  if (psmouse->out_of_sync_cnt) {
   psmouse->out_of_sync_cnt = 0;
   dev_notice(&(psmouse)->ps2dev.serio->dev, "%s at %s - driver resynced.\n", psmouse->name, psmouse->phys)

                                  ;
  }
  break;

 case PSMOUSE_GOOD_DATA:
  break;
 }
 return 0;
}






static irqreturn_t psmouse_interrupt(struct serio *serio,
  unsigned char data, unsigned int flags)
{
 struct psmouse *psmouse = serio_get_drvdata(serio);

 if (psmouse->state == PSMOUSE_IGNORE)
  goto out;

 if (__builtin_expect(!!((flags & 1) || ((flags & 2) && !psmouse->ignore_parity)), 0)
                                                           ) {

  if (psmouse->state == PSMOUSE_ACTIVATED)
   dev_warn(&(psmouse)->ps2dev.serio->dev, "bad data from KBC -%s%s\n", flags & 1 ? " timeout" : "", flags & 2 ? " bad parity" : "")


                                                   ;
  ps2_cmd_aborted(&psmouse->ps2dev);
  goto out;
 }

 if (__builtin_expect(!!(psmouse->ps2dev.flags & 1), 0))
  if (ps2_handle_ack(&psmouse->ps2dev, data))
   goto out;

 if (__builtin_expect(!!(psmouse->ps2dev.flags & 2), 0))
  if (ps2_handle_response(&psmouse->ps2dev, data))
   goto out;

 if (psmouse->state <= PSMOUSE_RESYNCING)
  goto out;

 if (psmouse->state == PSMOUSE_ACTIVATED &&
     psmouse->pktcnt && (({ unsigned long __dummy; typeof(jiffies) __dummy2; (void)(&__dummy == &__dummy2); 1; }) && ({ unsigned long __dummy; typeof(psmouse->last + 250/2) __dummy2; (void)(&__dummy == &__dummy2); 1; }) && ((long)((psmouse->last + 250/2) - (jiffies)) < 0))) {
  _dev_info(&(psmouse)->ps2dev.serio->dev, "%s at %s lost synchronization, throwing %d bytes away.\n", psmouse->name, psmouse->phys, psmouse->pktcnt)
                                                      ;
  psmouse->badbyte = psmouse->packet[0];
  __psmouse_set_state(psmouse, PSMOUSE_RESYNCING);
  psmouse_queue_work(psmouse, &psmouse->resync_work, 0);
  goto out;
 }

 psmouse->packet[psmouse->pktcnt++] = data;



 if (__builtin_expect(!!(psmouse->packet[0] == 0xaa && psmouse->pktcnt <= 2), 0)) {
  if (psmouse->pktcnt == 1) {
   psmouse->last = jiffies;
   goto out;
  }

  if (psmouse->packet[1] == 0x00 ||
      (psmouse->type == PSMOUSE_HGPK &&
       psmouse->packet[1] == 0xaa)) {
   __psmouse_set_state(psmouse, PSMOUSE_IGNORE);
   serio_reconnect(serio);
   goto out;
  }



  psmouse->pktcnt = 1;
  if (psmouse_handle_byte(psmouse))
   goto out;

  psmouse->packet[psmouse->pktcnt++] = data;
 }




 if (psmouse->state == PSMOUSE_ACTIVATED &&
     psmouse->pktcnt == 1 && psmouse->resync_time &&
     (({ unsigned long __dummy; typeof(jiffies) __dummy2; (void)(&__dummy == &__dummy2); 1; }) && ({ unsigned long __dummy; typeof(psmouse->last + psmouse->resync_time * 250) __dummy2; (void)(&__dummy == &__dummy2); 1; }) && ((long)((psmouse->last + psmouse->resync_time * 250) - (jiffies)) < 0))) {
  psmouse->badbyte = psmouse->packet[0];
  __psmouse_set_state(psmouse, PSMOUSE_RESYNCING);
  psmouse_queue_work(psmouse, &psmouse->resync_work, 0);
  goto out;
 }

 psmouse->last = jiffies;
 psmouse_handle_byte(psmouse);

 out:
 return IRQ_HANDLED;
}
int psmouse_sliced_command(struct psmouse *psmouse, unsigned char command)
{
 int i;

 if (ps2_command(&psmouse->ps2dev, ((void *)0), 0x00e6))
  return -1;

 for (i = 6; i >= 0; i -= 2) {
  unsigned char d = (command >> i) & 3;
  if (ps2_command(&psmouse->ps2dev, &d, 0x10e8))
   return -1;
 }

 return 0;
}





int psmouse_reset(struct psmouse *psmouse)
{
 unsigned char param[2];

 if (ps2_command(&psmouse->ps2dev, param, 0x02ff))
  return -1;

 if (param[0] != 0xaa && param[1] != 0x00)
  return -1;

 return 0;
}





void psmouse_set_resolution(struct psmouse *psmouse, unsigned int resolution)
{
 static const unsigned char params[] = { 0, 1, 2, 2, 3 };
 unsigned char p;

 if (resolution == 0 || resolution > 200)
  resolution = 200;

 p = params[resolution / 50];
 ps2_command(&psmouse->ps2dev, &p, 0x10e8);
 psmouse->resolution = 25 << p;
}





static void psmouse_set_rate(struct psmouse *psmouse, unsigned int rate)
{
 static const unsigned char rates[] = { 200, 100, 80, 60, 40, 20, 10, 0 };
 unsigned char r;
 int i = 0;

 while (rates[i] > rate) i++;
 r = rates[i];
 ps2_command(&psmouse->ps2dev, &r, 0x10f3);
 psmouse->rate = r;
}





static int psmouse_poll(struct psmouse *psmouse)
{
 return ps2_command(&psmouse->ps2dev, psmouse->packet,
      0x00eb | (psmouse->pktsize << 8));
}





static int genius_detect(struct psmouse *psmouse, bool set_properties)
{
 struct ps2dev *ps2dev = &psmouse->ps2dev;
 unsigned char param[4];

 param[0] = 3;
 ps2_command(ps2dev, param, 0x10e8);
 ps2_command(ps2dev, ((void *)0), 0x00e6);
 ps2_command(ps2dev, ((void *)0), 0x00e6);
 ps2_command(ps2dev, ((void *)0), 0x00e6);
 ps2_command(ps2dev, param, 0x03e9);

 if (param[0] != 0x00 || param[1] != 0x33 || param[2] != 0x55)
  return -1;

 if (set_properties) {
  __set_bit(0x112, psmouse->dev->keybit);
  __set_bit(0x114, psmouse->dev->keybit);
  __set_bit(0x113, psmouse->dev->keybit);
  __set_bit(0x08, psmouse->dev->relbit);

  psmouse->vendor = "Genius";
  psmouse->name = "Mouse";
  psmouse->pktsize = 4;
 }

 return 0;
}




static int intellimouse_detect(struct psmouse *psmouse, bool set_properties)
{
 struct ps2dev *ps2dev = &psmouse->ps2dev;
 unsigned char param[2];

 param[0] = 200;
 ps2_command(ps2dev, param, 0x10f3);
 param[0] = 100;
 ps2_command(ps2dev, param, 0x10f3);
 param[0] = 80;
 ps2_command(ps2dev, param, 0x10f3);
 ps2_command(ps2dev, param, 0x02f2);

 if (param[0] != 3)
  return -1;

 if (set_properties) {
  __set_bit(0x112, psmouse->dev->keybit);
  __set_bit(0x08, psmouse->dev->relbit);

  if (!psmouse->vendor)
   psmouse->vendor = "Generic";
  if (!psmouse->name)
   psmouse->name = "Wheel Mouse";
  psmouse->pktsize = 4;
 }

 return 0;
}




static int im_explorer_detect(struct psmouse *psmouse, bool set_properties)
{
 struct ps2dev *ps2dev = &psmouse->ps2dev;
 unsigned char param[2];

 intellimouse_detect(psmouse, 0);

 param[0] = 200;
 ps2_command(ps2dev, param, 0x10f3);
 param[0] = 200;
 ps2_command(ps2dev, param, 0x10f3);
 param[0] = 80;
 ps2_command(ps2dev, param, 0x10f3);
 ps2_command(ps2dev, param, 0x02f2);

 if (param[0] != 4)
  return -1;


 param[0] = 200;
 ps2_command(ps2dev, param, 0x10f3);
 param[0] = 80;
 ps2_command(ps2dev, param, 0x10f3);
 param[0] = 40;
 ps2_command(ps2dev, param, 0x10f3);

 if (set_properties) {
  __set_bit(0x112, psmouse->dev->keybit);
  __set_bit(0x08, psmouse->dev->relbit);
  __set_bit(0x06, psmouse->dev->relbit);
  __set_bit(0x113, psmouse->dev->keybit);
  __set_bit(0x114, psmouse->dev->keybit);

  if (!psmouse->vendor)
   psmouse->vendor = "Generic";
  if (!psmouse->name)
   psmouse->name = "Explorer Mouse";
  psmouse->pktsize = 4;
 }

 return 0;
}




static int thinking_detect(struct psmouse *psmouse, bool set_properties)
{
 struct ps2dev *ps2dev = &psmouse->ps2dev;
 unsigned char param[2];
 static const unsigned char seq[] = { 20, 60, 40, 20, 20, 60, 40, 20, 20 };
 int i;

 param[0] = 10;
 ps2_command(ps2dev, param, 0x10f3);
 param[0] = 0;
 ps2_command(ps2dev, param, 0x10e8);
 for (i = 0; i < (sizeof(seq) / sizeof((seq)[0]) + (sizeof(struct { int:-!!(__builtin_types_compatible_p(typeof((seq)), typeof(&(seq)[0]))); }))); i++) {
  param[0] = seq[i];
  ps2_command(ps2dev, param, 0x10f3);
 }
 ps2_command(ps2dev, param, 0x02f2);

 if (param[0] != 2)
  return -1;

 if (set_properties) {
  __set_bit(0x112, psmouse->dev->keybit);
  __set_bit(0x114, psmouse->dev->keybit);

  psmouse->vendor = "Kensington";
  psmouse->name = "ThinkingMouse";
 }

 return 0;
}




static int ps2bare_detect(struct psmouse *psmouse, bool set_properties)
{
 if (set_properties) {
  if (!psmouse->vendor)
   psmouse->vendor = "Generic";
  if (!psmouse->name)
   psmouse->name = "Mouse";





  __set_bit(0x112, psmouse->dev->keybit);
 }

 return 0;
}





static int cortron_detect(struct psmouse *psmouse, bool set_properties)
{
 if (set_properties) {
  psmouse->vendor = "Cortron";
  psmouse->name = "PS/2 Trackball";

  __set_bit(0x112, psmouse->dev->keybit);
  __set_bit(0x113, psmouse->dev->keybit);
 }

 return 0;
}






static void psmouse_apply_defaults(struct psmouse *psmouse)
{
 struct input_dev *input_dev = psmouse->dev;

 memset(input_dev->evbit, 0, sizeof(input_dev->evbit));
 memset(input_dev->keybit, 0, sizeof(input_dev->keybit));
 memset(input_dev->relbit, 0, sizeof(input_dev->relbit));
 memset(input_dev->absbit, 0, sizeof(input_dev->absbit));
 memset(input_dev->mscbit, 0, sizeof(input_dev->mscbit));

 __set_bit(0x01, input_dev->evbit);
 __set_bit(0x02, input_dev->evbit);

 __set_bit(0x110, input_dev->keybit);
 __set_bit(0x111, input_dev->keybit);

 __set_bit(0x00, input_dev->relbit);
 __set_bit(0x01, input_dev->relbit);

 __set_bit(0x00, input_dev->propbit);

 psmouse->set_rate = psmouse_set_rate;
 psmouse->set_resolution = psmouse_set_resolution;
 psmouse->poll = psmouse_poll;
 psmouse->protocol_handler = psmouse_process_byte;
 psmouse->pktsize = 3;
 psmouse->reconnect = ((void *)0);
 psmouse->disconnect = ((void *)0);
 psmouse->cleanup = ((void *)0);
 psmouse->pt_activate = ((void *)0);
 psmouse->pt_deactivate = ((void *)0);
}





static int psmouse_do_detect(int (*detect)(struct psmouse *psmouse,
        bool set_properties),
        struct psmouse *psmouse, bool set_properties)
{
 if (set_properties)
  psmouse_apply_defaults(psmouse);

 return detect(psmouse, set_properties);
}






static int psmouse_extensions(struct psmouse *psmouse,
         unsigned int max_proto, bool set_properties)
{
 bool synaptics_hardware = false;





 if (psmouse_do_detect(lifebook_detect, psmouse, set_properties) == 0) {
  if (max_proto > PSMOUSE_IMEX) {
   if (!set_properties || lifebook_init(psmouse) == 0)
    return PSMOUSE_LIFEBOOK;
  }
 }






 if (max_proto > PSMOUSE_IMEX &&
     psmouse_do_detect(thinking_detect, psmouse, set_properties) == 0) {
  return PSMOUSE_THINKPS;
 }






 if (max_proto > PSMOUSE_PS2 &&
     psmouse_do_detect(synaptics_detect, psmouse, set_properties) == 0) {
  synaptics_hardware = true;

  if (max_proto > PSMOUSE_IMEX) {




   if (synaptics_supported() &&
       (!set_properties || synaptics_init(psmouse) == 0)) {
    return PSMOUSE_SYNAPTICS;
   }






   max_proto = PSMOUSE_IMEX;
  }



  synaptics_reset(psmouse);
 }






 if (max_proto > PSMOUSE_IMEX &&
   cypress_detect(psmouse, set_properties) == 0) {
  if (cypress_supported()) {
   if (cypress_init(psmouse) == 0)
    return PSMOUSE_CYPRESS;






   return PSMOUSE_PS2;
  }

  max_proto = PSMOUSE_IMEX;
 }




 if (max_proto > PSMOUSE_IMEX) {
  ps2_command(&psmouse->ps2dev, ((void *)0), 0x00f6);
  if (psmouse_do_detect(alps_detect,
          psmouse, set_properties) == 0) {
   if (!set_properties || alps_init(psmouse) == 0)
    return PSMOUSE_ALPS;



   max_proto = PSMOUSE_IMEX;
  }
 }




 if (max_proto > PSMOUSE_IMEX &&
     psmouse_do_detect(hgpk_detect, psmouse, set_properties) == 0) {
  if (!set_properties || hgpk_init(psmouse) == 0)
   return PSMOUSE_HGPK;



  max_proto = PSMOUSE_IMEX;
 }




 if (max_proto > PSMOUSE_IMEX &&
     psmouse_do_detect(elantech_detect, psmouse, set_properties) == 0) {
  if (!set_properties || elantech_init(psmouse) == 0)
   return PSMOUSE_ELANTECH;



  max_proto = PSMOUSE_IMEX;
 }

 if (max_proto > PSMOUSE_IMEX) {
  if (psmouse_do_detect(genius_detect,
          psmouse, set_properties) == 0)
   return PSMOUSE_GENPS;

  if (psmouse_do_detect(ps2pp_init,
          psmouse, set_properties) == 0)
   return PSMOUSE_PS2PP;

  if (psmouse_do_detect(trackpoint_detect,
          psmouse, set_properties) == 0)
   return PSMOUSE_TRACKPOINT;

  if (psmouse_do_detect(touchkit_ps2_detect,
          psmouse, set_properties) == 0)
   return PSMOUSE_TOUCHKIT_PS2;
 }





 if (max_proto > PSMOUSE_IMEX) {
  if (psmouse_do_detect(fsp_detect,
          psmouse, set_properties) == 0) {
   if (!set_properties || fsp_init(psmouse) == 0)
    return PSMOUSE_FSP;



   max_proto = PSMOUSE_IMEX;
  }
 }






 ps2_command(&psmouse->ps2dev, ((void *)0), 0x00f6);
 psmouse_reset(psmouse);

 if (max_proto >= PSMOUSE_IMEX &&
     psmouse_do_detect(im_explorer_detect,
         psmouse, set_properties) == 0) {
  return PSMOUSE_IMEX;
 }

 if (max_proto >= PSMOUSE_IMPS &&
     psmouse_do_detect(intellimouse_detect,
         psmouse, set_properties) == 0) {
  return PSMOUSE_IMPS;
 }





 psmouse_do_detect(ps2bare_detect, psmouse, set_properties);

 if (synaptics_hardware) {






  psmouse_reset(psmouse);
 }

 return PSMOUSE_PS2;
}

static const struct psmouse_protocol psmouse_protocols[] = {
 {
  .type = PSMOUSE_PS2,
  .name = "PS/2",
  .alias = "bare",
  .maxproto = true,
  .ignore_parity = true,
  .detect = ps2bare_detect,
 },

 {
  .type = PSMOUSE_PS2PP,
  .name = "PS2++",
  .alias = "logitech",
  .detect = ps2pp_init,
 },

 {
  .type = PSMOUSE_THINKPS,
  .name = "ThinkPS/2",
  .alias = "thinkps",
  .detect = thinking_detect,
 },

 {
  .type = PSMOUSE_CYPRESS,
  .name = "CyPS/2",
  .alias = "cypress",
  .detect = cypress_detect,
  .init = cypress_init,
 },

 {
  .type = PSMOUSE_GENPS,
  .name = "GenPS/2",
  .alias = "genius",
  .detect = genius_detect,
 },
 {
  .type = PSMOUSE_IMPS,
  .name = "ImPS/2",
  .alias = "imps",
  .maxproto = true,
  .ignore_parity = true,
  .detect = intellimouse_detect,
 },
 {
  .type = PSMOUSE_IMEX,
  .name = "ImExPS/2",
  .alias = "exps",
  .maxproto = true,
  .ignore_parity = true,
  .detect = im_explorer_detect,
 },

 {
  .type = PSMOUSE_SYNAPTICS,
  .name = "SynPS/2",
  .alias = "synaptics",
  .detect = synaptics_detect,
  .init = synaptics_init,
 },
 {
  .type = PSMOUSE_SYNAPTICS_RELATIVE,
  .name = "SynRelPS/2",
  .alias = "synaptics-relative",
  .detect = synaptics_detect,
  .init = synaptics_init_relative,
 },


 {
  .type = PSMOUSE_ALPS,
  .name = "AlpsPS/2",
  .alias = "alps",
  .detect = alps_detect,
  .init = alps_init,
 },


 {
  .type = PSMOUSE_LIFEBOOK,
  .name = "LBPS/2",
  .alias = "lifebook",
  .init = lifebook_init,
 },


 {
  .type = PSMOUSE_TRACKPOINT,
  .name = "TPPS/2",
  .alias = "trackpoint",
  .detect = trackpoint_detect,
 },
 {
  .type = PSMOUSE_ELANTECH,
  .name = "ETPS/2",
  .alias = "elantech",
  .detect = elantech_detect,
  .init = elantech_init,
 },


 {
  .type = PSMOUSE_FSP,
  .name = "FSPPS/2",
  .alias = "fsp",
  .detect = fsp_detect,
  .init = fsp_init,
 },

 {
  .type = PSMOUSE_CORTRON,
  .name = "CortronPS/2",
  .alias = "cortps",
  .detect = cortron_detect,
 },
 {
  .type = PSMOUSE_AUTO,
  .name = "auto",
  .alias = "any",
  .maxproto = true,
 },
};

static const struct psmouse_protocol *psmouse_protocol_by_type(enum psmouse_type type)
{
 int i;

 for (i = 0; i < (sizeof(psmouse_protocols) / sizeof((psmouse_protocols)[0]) + (sizeof(struct { int:-!!(__builtin_types_compatible_p(typeof((psmouse_protocols)), typeof(&(psmouse_protocols)[0]))); }))); i++)
  if (psmouse_protocols[i].type == type)
   return &psmouse_protocols[i];

 ({ int __ret_warn_on = !!(1); if (__builtin_expect(!!(__ret_warn_on), 0)) warn_slowpath_null("/home/bai/Kern3.17.2/psmouse/psmouse-base.c", 1052); __builtin_expect(!!(__ret_warn_on), 0); });
 return &psmouse_protocols[0];
}

static const struct psmouse_protocol *psmouse_protocol_by_name(const char *name, size_t len)
{
 const struct psmouse_protocol *p;
 int i;

 for (i = 0; i < (sizeof(psmouse_protocols) / sizeof((psmouse_protocols)[0]) + (sizeof(struct { int:-!!(__builtin_types_compatible_p(typeof((psmouse_protocols)), typeof(&(psmouse_protocols)[0]))); }))); i++) {
  p = &psmouse_protocols[i];

  if ((strlen(p->name) == len && !strncmp(p->name, name, len)) ||
      (strlen(p->alias) == len && !strncmp(p->alias, name, len)))
   return &psmouse_protocols[i];
 }

 return ((void *)0);
}






static int psmouse_probe(struct psmouse *psmouse)
{
 struct ps2dev *ps2dev = &psmouse->ps2dev;
 unsigned char param[2];
 param[0] = 0xa5;
 if (ps2_command(ps2dev, param, 0x02f2))
  return -1;

 if (param[0] != 0x00 && param[0] != 0x03 &&
     param[0] != 0x04 && param[0] != 0xff)
  return -1;





 if (ps2_command(ps2dev, ((void *)0), 0x00f6))
  dev_warn(&(psmouse)->ps2dev.serio->dev, "Failed to reset mouse on %s\n", ps2dev->serio->phys)
                            ;

 return 0;
}





static void psmouse_initialize(struct psmouse *psmouse)
{




 if (psmouse_max_proto != PSMOUSE_PS2) {
  psmouse->set_rate(psmouse, psmouse->rate);
  psmouse->set_resolution(psmouse, psmouse->resolution);
  ps2_command(&psmouse->ps2dev, ((void *)0), 0x00e6);
 }
}





int psmouse_activate(struct psmouse *psmouse)
{
 if (ps2_command(&psmouse->ps2dev, ((void *)0), 0x00f4)) {
  dev_warn(&(psmouse)->ps2dev.serio->dev, "Failed to enable mouse on %s\n", psmouse->ps2dev.serio->phys)
                                    ;
  return -1;
 }

 psmouse_set_state(psmouse, PSMOUSE_ACTIVATED);
 return 0;
}






int psmouse_deactivate(struct psmouse *psmouse)
{
 if (ps2_command(&psmouse->ps2dev, ((void *)0), 0x00f5)) {
  dev_warn(&(psmouse)->ps2dev.serio->dev, "Failed to deactivate mouse on %s\n", psmouse->ps2dev.serio->phys)
                                    ;
  return -1;
 }

 psmouse_set_state(psmouse, PSMOUSE_CMD_MODE);
 return 0;
}






static void psmouse_resync(struct work_struct *work)
{
 struct psmouse *parent = ((void *)0), *psmouse =
  ({ const typeof( ((struct psmouse *)0)->resync_work.work ) *__mptr = (work); (struct psmouse *)( (char *)__mptr - __builtin_offsetof(struct psmouse,resync_work.work) );});
 struct serio *serio = psmouse->ps2dev.serio;
 psmouse_ret_t rc = PSMOUSE_GOOD_DATA;
 bool failed = false, enabled = false;
 int i;

 mutex_lock(&psmouse_mutex);

 if (psmouse->state != PSMOUSE_RESYNCING)
  goto out;

 if (serio->parent && serio->id.type == 0x05) {
  parent = serio_get_drvdata(serio->parent);
  psmouse_deactivate(parent);
 }
 psmouse->num_resyncs++;

 if (ps2_sendbyte(&psmouse->ps2dev, 0x00f5, 20)) {
  if (psmouse->num_resyncs < 3 || psmouse->acks_disable_command)
   failed = true;
 } else
  psmouse->acks_disable_command = true;
 if (!failed) {
  if (psmouse->poll(psmouse))
   failed = true;
  else {
   psmouse_set_state(psmouse, PSMOUSE_CMD_MODE);
   for (i = 0; i < psmouse->pktsize; i++) {
    psmouse->pktcnt++;
    rc = psmouse->protocol_handler(psmouse);
    if (rc != PSMOUSE_GOOD_DATA)
     break;
   }
   if (rc != PSMOUSE_FULL_PACKET)
    failed = true;
   psmouse_set_state(psmouse, PSMOUSE_RESYNCING);
  }
 }





 for (i = 0; i < 5; i++) {
  if (!ps2_command(&psmouse->ps2dev, ((void *)0), 0x00f4)) {
   enabled = true;
   break;
  }
  msleep(200);
 }

 if (!enabled) {
  dev_warn(&(psmouse)->ps2dev.serio->dev, "failed to re-enable mouse on %s\n", psmouse->ps2dev.serio->phys)
                                    ;
  failed = true;
 }

 if (failed) {
  psmouse_set_state(psmouse, PSMOUSE_IGNORE);
  _dev_info(&(psmouse)->ps2dev.serio->dev, "resync failed, issuing reconnect request\n")
                                                     ;
  serio_reconnect(serio);
 } else
  psmouse_set_state(psmouse, PSMOUSE_ACTIVATED);

 if (parent)
  psmouse_activate(parent);
 out:
 mutex_unlock(&psmouse_mutex);
}





static void psmouse_cleanup(struct serio *serio)
{
 struct psmouse *psmouse = serio_get_drvdata(serio);
 struct psmouse *parent = ((void *)0);

 mutex_lock(&psmouse_mutex);

 if (serio->parent && serio->id.type == 0x05) {
  parent = serio_get_drvdata(serio->parent);
  psmouse_deactivate(parent);
 }

 psmouse_set_state(psmouse, PSMOUSE_INITIALIZING);




 if (ps2_command(&psmouse->ps2dev, ((void *)0), 0x00f5))
  dev_warn(&(psmouse)->ps2dev.serio->dev, "Failed to disable mouse on %s\n", psmouse->ps2dev.serio->phys)
                                    ;

 if (psmouse->cleanup)
  psmouse->cleanup(psmouse);




 ps2_command(&psmouse->ps2dev, ((void *)0), 0x00f6);





 ps2_command(&psmouse->ps2dev, ((void *)0), 0x00f4);

 if (parent) {
  if (parent->pt_deactivate)
   parent->pt_deactivate(parent);

  psmouse_activate(parent);
 }

 mutex_unlock(&psmouse_mutex);
}





static void psmouse_disconnect(struct serio *serio)
{
 struct psmouse *psmouse, *parent = ((void *)0);

 psmouse = serio_get_drvdata(serio);

 sysfs_remove_group(&serio->dev.kobj, &psmouse_attribute_group);

 mutex_lock(&psmouse_mutex);

 psmouse_set_state(psmouse, PSMOUSE_CMD_MODE);


 mutex_unlock(&psmouse_mutex);
 flush_workqueue(kpsmoused_wq);
 mutex_lock(&psmouse_mutex);

 if (serio->parent && serio->id.type == 0x05) {
  parent = serio_get_drvdata(serio->parent);
  psmouse_deactivate(parent);
 }

 if (psmouse->disconnect)
  psmouse->disconnect(psmouse);

 if (parent && parent->pt_deactivate)
  parent->pt_deactivate(parent);

 psmouse_set_state(psmouse, PSMOUSE_IGNORE);

 serio_close(serio);
 serio_set_drvdata(serio, ((void *)0));
 input_unregister_device(psmouse->dev);
 kfree(psmouse);

 if (parent)
  psmouse_activate(parent);

 mutex_unlock(&psmouse_mutex);
}

static int psmouse_switch_protocol(struct psmouse *psmouse,
       const struct psmouse_protocol *proto)
{
 const struct psmouse_protocol *selected_proto;
 struct input_dev *input_dev = psmouse->dev;

 input_dev->dev.parent = &psmouse->ps2dev.serio->dev;

 if (proto && (proto->detect || proto->init)) {
  psmouse_apply_defaults(psmouse);

  if (proto->detect && proto->detect(psmouse, true) < 0)
   return -1;

  if (proto->init && proto->init(psmouse) < 0)
   return -1;

  psmouse->type = proto->type;
  selected_proto = proto;
 } else {
  psmouse->type = psmouse_extensions(psmouse,
         psmouse_max_proto, true);
  selected_proto = psmouse_protocol_by_type(psmouse->type);
 }

 psmouse->ignore_parity = selected_proto->ignore_parity;






 if (psmouse->pktsize == 3)
  psmouse->resync_time = 0;







 if (psmouse->resync_time && psmouse->poll(psmouse))
  psmouse->resync_time = 0;

 snprintf(psmouse->devname, sizeof(psmouse->devname), "%s %s %s",
   selected_proto->name, psmouse->vendor, psmouse->name);

 input_dev->name = psmouse->devname;
 input_dev->phys = psmouse->phys;
 input_dev->id.bustype = 0x11;
 input_dev->id.vendor = 0x0002;
 input_dev->id.product = psmouse->type;
 input_dev->id.version = psmouse->model;

 return 0;
}





static int psmouse_connect(struct serio *serio, struct serio_driver *drv)
{
 struct psmouse *psmouse, *parent = ((void *)0);
 struct input_dev *input_dev;
 int retval = 0, error = -12;

 mutex_lock(&psmouse_mutex);





 if (serio->parent && serio->id.type == 0x05) {
  parent = serio_get_drvdata(serio->parent);
  psmouse_deactivate(parent);
 }

 psmouse = kzalloc(sizeof(struct psmouse), ((( gfp_t)0x10u) | (( gfp_t)0x40u) | (( gfp_t)0x80u)));
 input_dev = input_allocate_device();
 if (!psmouse || !input_dev)
  goto err_free;

 ps2_init(&psmouse->ps2dev, serio);
 do { do { do { __init_work(((&(&psmouse->resync_work)->work)), 0); ((&(&psmouse->resync_work)->work))->data = (atomic_long_t) { (WORK_STRUCT_NO_POOL) }; INIT_LIST_HEAD(&((&(&psmouse->resync_work)->work))->entry); ((&(&psmouse->resync_work)->work))->func = (((psmouse_resync))); } while (0); } while (0); do { init_timer_key(((&(&psmouse->resync_work)->timer)), (((0) | 0x2LU)), ((void *)0), ((void *)0)); (&(&psmouse->resync_work)->timer)->function = (delayed_work_timer_fn); (&(&psmouse->resync_work)->timer)->data = ((unsigned long)(&psmouse->resync_work)); } while (0); } while (0);
 psmouse->dev = input_dev;
 snprintf(psmouse->phys, sizeof(psmouse->phys), "%s/input0", serio->phys);

 psmouse_set_state(psmouse, PSMOUSE_INITIALIZING);

 serio_set_drvdata(serio, psmouse);

 error = serio_open(serio, drv);
 if (error)
  goto err_clear_drvdata;

 if (psmouse_probe(psmouse) < 0) {
  error = -19;
  goto err_close_serio;
 }

 psmouse->rate = psmouse_rate;
 psmouse->resolution = psmouse_resolution;
 psmouse->resetafter = psmouse_resetafter;
 psmouse->resync_time = parent ? 0 : psmouse_resync_time;
 psmouse->smartscroll = psmouse_smartscroll;

 psmouse_switch_protocol(psmouse, ((void *)0));

 psmouse_set_state(psmouse, PSMOUSE_CMD_MODE);
 psmouse_initialize(psmouse);

 error = input_register_device(psmouse->dev);
 if (error)
  goto err_protocol_disconnect;

 if (parent && parent->pt_activate)
  parent->pt_activate(parent);

 error = sysfs_create_group(&serio->dev.kobj, &psmouse_attribute_group);
 if (error)
  goto err_pt_deactivate;

 psmouse_activate(psmouse);

 out:

 if (parent)
  psmouse_activate(parent);

 mutex_unlock(&psmouse_mutex);
 return retval;

 err_pt_deactivate:
 if (parent && parent->pt_deactivate)
  parent->pt_deactivate(parent);
 input_unregister_device(psmouse->dev);
 input_dev = ((void *)0);
 err_protocol_disconnect:
 if (psmouse->disconnect)
  psmouse->disconnect(psmouse);
 psmouse_set_state(psmouse, PSMOUSE_IGNORE);
 err_close_serio:
 serio_close(serio);
 err_clear_drvdata:
 serio_set_drvdata(serio, ((void *)0));
 err_free:
 input_free_device(input_dev);
 kfree(psmouse);

 retval = error;
 goto out;
}


static int psmouse_reconnect(struct serio *serio)
{
 struct psmouse *psmouse = serio_get_drvdata(serio);
 struct psmouse *parent = ((void *)0);
 struct serio_driver *drv = serio->drv;
 unsigned char type;
 int rc = -1;

 if (!drv || !psmouse) {
  ({ if (0) dev_printk("\001" "7", &(psmouse)->ps2dev.serio->dev, "reconnect request, but serio is disconnected, ignoring...\n"); 0; })
                                                                     ;
  return -1;
 }

 mutex_lock(&psmouse_mutex);

 if (serio->parent && serio->id.type == 0x05) {
  parent = serio_get_drvdata(serio->parent);
  psmouse_deactivate(parent);
 }

 psmouse_set_state(psmouse, PSMOUSE_INITIALIZING);

 if (psmouse->reconnect) {
  if (psmouse->reconnect(psmouse))
   goto out;
 } else {
  psmouse_reset(psmouse);

  if (psmouse_probe(psmouse) < 0)
   goto out;

  type = psmouse_extensions(psmouse, psmouse_max_proto, false);
  if (psmouse->type != type)
   goto out;
 }





 psmouse_set_state(psmouse, PSMOUSE_CMD_MODE);

 psmouse_initialize(psmouse);

 if (parent && parent->pt_activate)
  parent->pt_activate(parent);

 psmouse_activate(psmouse);
 rc = 0;

out:

 if (parent)
  psmouse_activate(parent);

 mutex_unlock(&psmouse_mutex);
 return rc;
}

static struct serio_device_id psmouse_serio_ids[] = {
 {
  .type = 0x01,
  .proto = 0xff,
  .id = 0xff,
  .extra = 0xff,
 },
 {
  .type = 0x05,
  .proto = 0xff,
  .id = 0xff,
  .extra = 0xff,
 },
 { 0 }
};

extern const struct serio_device_id __mod_serio__psmouse_serio_ids_device_table __attribute__ ((unused, alias("psmouse_serio_ids")));

static struct serio_driver psmouse_drv = {
 .driver = {
  .name = "psmouse",
 },
 .description = "PS/2 mouse driver",
 .id_table = psmouse_serio_ids,
 .interrupt = psmouse_interrupt,
 .connect = psmouse_connect,
 .reconnect = psmouse_reconnect,
 .disconnect = psmouse_disconnect,
 .cleanup = psmouse_cleanup,
};

ssize_t psmouse_attr_show_helper(struct device *dev, struct device_attribute *devattr,
     char *buf)
{
 struct serio *serio = ({ const typeof( ((struct serio *)0)->dev ) *__mptr = (dev); (struct serio *)( (char *)__mptr - __builtin_offsetof(struct serio,dev) );});
 struct psmouse_attribute *attr = ({ const typeof( ((struct psmouse_attribute *)0)->dattr ) *__mptr = ((devattr)); (struct psmouse_attribute *)( (char *)__mptr - __builtin_offsetof(struct psmouse_attribute,dattr) );});
 struct psmouse *psmouse;

 psmouse = serio_get_drvdata(serio);

 return attr->show(psmouse, attr->data, buf);
}

ssize_t psmouse_attr_set_helper(struct device *dev, struct device_attribute *devattr,
    const char *buf, size_t count)
{
 struct serio *serio = ({ const typeof( ((struct serio *)0)->dev ) *__mptr = (dev); (struct serio *)( (char *)__mptr - __builtin_offsetof(struct serio,dev) );});
 struct psmouse_attribute *attr = ({ const typeof( ((struct psmouse_attribute *)0)->dattr ) *__mptr = ((devattr)); (struct psmouse_attribute *)( (char *)__mptr - __builtin_offsetof(struct psmouse_attribute,dattr) );});
 struct psmouse *psmouse, *parent = ((void *)0);
 int retval;

 retval = mutex_lock_interruptible(&psmouse_mutex);
 if (retval)
  goto out;

 psmouse = serio_get_drvdata(serio);

 if (attr->protect) {
  if (psmouse->state == PSMOUSE_IGNORE) {
   retval = -19;
   goto out_unlock;
  }

  if (serio->parent && serio->id.type == 0x05) {
   parent = serio_get_drvdata(serio->parent);
   psmouse_deactivate(parent);
  }

  psmouse_deactivate(psmouse);
 }

 retval = attr->set(psmouse, attr->data, buf, count);

 if (attr->protect) {
  if (retval != -19)
   psmouse_activate(psmouse);

  if (parent)
   psmouse_activate(parent);
 }

 out_unlock:
 mutex_unlock(&psmouse_mutex);
 out:
 return retval;
}

static ssize_t psmouse_show_int_attr(struct psmouse *psmouse, void *offset, char *buf)
{
 unsigned int *field = (unsigned int *)((char *)psmouse + (size_t)offset);

 return sprintf(buf, "%u\n", *field);
}

static ssize_t psmouse_set_int_attr(struct psmouse *psmouse, void *offset, const char *buf, size_t count)
{
 unsigned int *field = (unsigned int *)((char *)psmouse + (size_t)offset);
 unsigned int value;
 int err;

 err = kstrtouint(buf, 10, &value);
 if (err)
  return err;

 *field = value;

 return count;
}

static ssize_t psmouse_attr_show_protocol(struct psmouse *psmouse, void *data, char *buf)
{
 return sprintf(buf, "%s\n", psmouse_protocol_by_type(psmouse->type)->name);
}

static ssize_t psmouse_attr_set_protocol(struct psmouse *psmouse, void *data, const char *buf, size_t count)
{
 struct serio *serio = psmouse->ps2dev.serio;
 struct psmouse *parent = ((void *)0);
 struct input_dev *old_dev, *new_dev;
 const struct psmouse_protocol *proto, *old_proto;
 int error;
 int retry = 0;

 proto = psmouse_protocol_by_name(buf, count);
 if (!proto)
  return -22;

 if (psmouse->type == proto->type)
  return count;

 new_dev = input_allocate_device();
 if (!new_dev)
  return -12;

 while (!list_empty(&serio->children)) {
  if (++retry > 3) {
   dev_warn(&(psmouse)->ps2dev.serio->dev, "failed to destroy children ports, protocol change aborted.\n")
                                                                        ;
   input_free_device(new_dev);
   return -5;
  }

  mutex_unlock(&psmouse_mutex);
  serio_unregister_child_port(serio);
  mutex_lock(&psmouse_mutex);

  if (serio->drv != &psmouse_drv) {
   input_free_device(new_dev);
   return -19;
  }

  if (psmouse->type == proto->type) {
   input_free_device(new_dev);
   return count;
  }
 }

 if (serio->parent && serio->id.type == 0x05) {
  parent = serio_get_drvdata(serio->parent);
  if (parent->pt_deactivate)
   parent->pt_deactivate(parent);
 }

 old_dev = psmouse->dev;
 old_proto = psmouse_protocol_by_type(psmouse->type);

 if (psmouse->disconnect)
  psmouse->disconnect(psmouse);

 psmouse_set_state(psmouse, PSMOUSE_IGNORE);

 psmouse->dev = new_dev;
 psmouse_set_state(psmouse, PSMOUSE_INITIALIZING);

 if (psmouse_switch_protocol(psmouse, proto) < 0) {
  psmouse_reset(psmouse);

  psmouse_switch_protocol(psmouse, &psmouse_protocols[0]);
 }

 psmouse_initialize(psmouse);
 psmouse_set_state(psmouse, PSMOUSE_CMD_MODE);

 error = input_register_device(psmouse->dev);
 if (error) {
  if (psmouse->disconnect)
   psmouse->disconnect(psmouse);

  psmouse_set_state(psmouse, PSMOUSE_IGNORE);
  input_free_device(new_dev);
  psmouse->dev = old_dev;
  psmouse_set_state(psmouse, PSMOUSE_INITIALIZING);
  psmouse_switch_protocol(psmouse, old_proto);
  psmouse_initialize(psmouse);
  psmouse_set_state(psmouse, PSMOUSE_CMD_MODE);

  return error;
 }

 input_unregister_device(old_dev);

 if (parent && parent->pt_activate)
  parent->pt_activate(parent);

 return count;
}

static ssize_t psmouse_attr_set_rate(struct psmouse *psmouse, void *data, const char *buf, size_t count)
{
 unsigned int value;
 int err;

 err = kstrtouint(buf, 10, &value);
 if (err)
  return err;

 psmouse->set_rate(psmouse, value);
 return count;
}

static ssize_t psmouse_attr_set_resolution(struct psmouse *psmouse, void *data, const char *buf, size_t count)
{
 unsigned int value;
 int err;

 err = kstrtouint(buf, 10, &value);
 if (err)
  return err;

 psmouse->set_resolution(psmouse, value);
 return count;
}


static int psmouse_set_maxproto(const char *val, const struct kernel_param *kp)
{
 const struct psmouse_protocol *proto;

 if (!val)
  return -22;

 proto = psmouse_protocol_by_name(val, strlen(val));

 if (!proto || !proto->maxproto)
  return -22;

 *((unsigned int *)kp->arg) = proto->type;

 return 0;
}

static int psmouse_get_maxproto(char *buffer, const struct kernel_param *kp)
{
 int type = *((unsigned int *)kp->arg);

 return sprintf(buffer, "%s", psmouse_protocol_by_type(type)->name);
}

static int __attribute__ ((__section__(".init.text"))) __attribute__((__cold__)) __attribute__((no_instrument_function)) psmouse_init(void)
{
 int err;

 lifebook_module_init();
 synaptics_module_init();
 hgpk_module_init();

 kpsmoused_wq = __alloc_workqueue_key(("%s"), (WQ_UNBOUND | __WQ_ORDERED | (WQ_MEM_RECLAIM)), (1), ((void *)0), ((void *)0),"kpsmoused");
 if (!kpsmoused_wq) {
  printk("\001" "3" "psmouse" ": " "failed to create kpsmoused workqueue\n");
  return -12;
 }

 err = __serio_register_driver(&psmouse_drv, (&__this_module), "psmouse");
 if (err)
  destroy_workqueue(kpsmoused_wq);

 return err;
}

static void __attribute__ ((__section__(".exit.text"))) __attribute__((__cold__)) __attribute__((no_instrument_function)) psmouse_exit(void)
{
 serio_unregister_driver(&psmouse_drv);
 destroy_workqueue(kpsmoused_wq);
}

static inline __attribute__((no_instrument_function)) initcall_t __inittest(void) { return psmouse_init; } int init_module(void) __attribute__((alias("psmouse_init")));;
static inline __attribute__((no_instrument_function)) exitcall_t __exittest(void) { return psmouse_exit; } void cleanup_module(void) __attribute__((alias("psmouse_exit")));;
