"""Execute the actual serial loops with mocked UART/radio/ticks on an isolated QEMU RISC-V board.

Extracts only the two production loop bodies; does not duplicate their timing
logic. Checks settings, live changes, tick wrap and packet-size flushes. Runs a
fixed-20ms negative control to prove the checks detect the original defect.
"""
from pathlib import Path
import argparse
import json
import shutil
import tempfile
import os
import re
import subprocess

REPO=Path(__file__).resolve().parents[1]


def find_tool(name: str, override: str | None, pattern: str) -> Path:
    """Resolve an explicit tool, PATH entry or installed ESP-IDF tool; fail without downloading anything."""
    located=override or shutil.which(name) or shutil.which(name+'.exe')
    if located:return Path(located)
    tools=Path(os.environ.get('IDF_TOOLS_PATH',str(Path.home()/'.espressif')))
    matches=sorted(p for p in tools.glob(pattern) if p.is_file() and p.name in (name,name+'.exe'))
    if not matches:raise SystemExit(f'{name} not found; activate ESP-IDF or supply its explicit path.')
    return matches[-1]


PREFIX=r'''
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
typedef uint32_t TickType_t;
typedef int udp_conn_list_t;
typedef int fmav_message_t;
typedef int fmav_status_t;
typedef struct {unsigned frame_len; unsigned msgid; int res;} fmav_result_t;
typedef struct {struct {struct {uint16_t value;} db_param_u16;} value;} db_parameter_t;
db_parameter_t db_param_serial_read_timeout;
#define DB_PARAM_SERIAL_PACK_SIZE 576U
#define DB_SERIAL_PACK_SIZE_MAX 1056U
#define pdMS_TO_TICKS(ms) ((TickType_t)(ms)) /* Both configured projects use 1000 Hz ticks. */
#define FASTMAVLINK_PARSE_RESULT_OK 0
#define FASTMAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL 110
#define DB_MAVLINK_DATA_ORIGIN_SERIAL 0
#define ESP_LOGD(...) ((void)0)
static fmav_message_t msg;
static fmav_status_t fmav_status_serial;
static uint32_t serial_total_byte_count,serial_total_decoded_mav_msgs;
static TickType_t now;
static unsigned sends;
static int next_read;
static unsigned char buffer[2112];
static unsigned buffered;
/* Return controlled scheduler ticks to the unmodified production loop bodies. */
TickType_t xTaskGetTickCount(void) {return now;}
/* Supply a requested byte count once, or an idle UART thereafter. */
int db_read_serial(uint8_t *data,unsigned count) {int n=next_read;next_read=0;for(int i=0;i<n;i++)data[i]=0;return n;}
/* Record outgoing datagrams without touching a network. */
void db_send_to_all_clients(int *clients,udp_conn_list_t *conns,uint8_t *data,unsigned count) {sends++;}
/* Keep the mocked serial interface available. */
bool db_serial_is_available(void) {return true;}
/* No complete MAVLink frame is needed to exercise already-buffered timeout flushes. */
bool fmav_parse_and_check_to_frame_buf(fmav_result_t *r,uint8_t *b,fmav_status_t *s,uint8_t c) {return false;}
/* Unused parser-dependent production branches remain linkable. */
void fmav_frame_buf_to_msg(fmav_message_t *m,fmav_result_t *r,uint8_t *b) {}
/* Unused parser-dependent production branches remain linkable. */
bool fmav_msg_is_for_me(unsigned s,unsigned c,fmav_message_t *m) {return false;}
/* Return a dummy bridge system identity. */
unsigned db_get_mav_sys_id(void) {return 1;}
/* Return a dummy bridge component identity. */
unsigned db_get_mav_comp_id(void) {return 240;}
/* Ignore local messages; no valid frames are injected by these timeout checks. */
void handle_mavlink_message(fmav_message_t *m,int *c,udp_conn_list_t *u,fmav_status_t *s,int origin) {}
/* Ignore statistics while exercising the production loop's send decisions. */
void db_serial_stats_add(unsigned a,unsigned b,unsigned c,unsigned d) {}
/* Ignore statistics while exercising the production loop's send decisions. */
void db_serial_set_ticks_last_mav_msg(void) {}
/* Ignore statistics while exercising the production loop's send decisions. */
void db_serial_update_mavlink_rx_stats(fmav_message_t *m) {}
/* Ignore LED side effects in unused parser-dependent branches. */
void db_status_led_mark_serial_mavlink_rx(void) {}
/* Supply freestanding memcpy for branches retained by the compiler. */
void *memcpy(void *d,const void *s,size_t n) {unsigned char *a=d;const unsigned char *b=s;while(n--)*a++=*b++;return d;}
/* Supply freestanding memset for automatic production parser state. */
void *memset(void *d,int c,size_t n) {unsigned char *a=d;while(n--)*a++=(unsigned char)c;return d;}
'''
TESTS=r'''
typedef void (*loop_t)(int *,udp_conn_list_t *,uint8_t *,unsigned *);
#define CHECK(c) do {if(!(c))return __LINE__;} while(0)
/* Synchronize each loop's private timer and clear pending timeout state through a real flush. */
static void prime(loop_t loop,uint16_t timeout) {
    db_param_serial_read_timeout.value.db_param_u16.value=timeout;
    now+=1000000U;buffered=3;sends=0;next_read=0;
    loop(0,0,buffer,&buffered);sends=0;buffered=3;
}
/* Exercise a selected value just before and exactly at its expiration boundary. */
static int boundary(loop_t loop,uint16_t timeout) {
    prime(loop,timeout);TickType_t start=now;
    now=start+timeout-1U;loop(0,0,buffer,&buffered);CHECK(sends==0 && buffered==3);
    now=start+timeout;loop(0,0,buffer,&buffered);CHECK(sends==1 && buffered==0);
    return 0;
}
/* Run 10 behavioral scenarios per mode against the production function body. */
static int check_loop(loop_t loop) {
    uint16_t values[]={1,20,50,100,65535};
    for(unsigned i=0;i<5;i++){int result=boundary(loop,values[i]);if(result)return result;}
    /* A longer live setting must postpone an already pending flush. */
    prime(loop,20);TickType_t start=now;
    db_param_serial_read_timeout.value.db_param_u16.value=100;
    now=start+20;loop(0,0,buffer,&buffered);CHECK(sends==0 && buffered==3);
    now=start+100;loop(0,0,buffer,&buffered);CHECK(sends==1 && buffered==0);
    /* A shorter live setting must advance an already pending flush. */
    prime(loop,100);start=now;
    db_param_serial_read_timeout.value.db_param_u16.value=20;
    now=start+20;loop(0,0,buffer,&buffered);CHECK(sends==1 && buffered==0);
    /* Lowering the configured value back to one millisecond must take effect immediately. */
    prime(loop,100);start=now;
    db_param_serial_read_timeout.value.db_param_u16.value=1;
    now=start+1;loop(0,0,buffer,&buffered);CHECK(sends==1 && buffered==0);
    /* Full packet buffers must still flush without waiting for the timeout. */
    prime(loop,65535);buffered=DB_PARAM_SERIAL_PACK_SIZE;now++;
    loop(0,0,buffer,&buffered);CHECK(sends==1 && buffered==0);
    /* Unsigned tick wrap must not change configured expiration behavior. */
    db_param_serial_read_timeout.value.db_param_u16.value=50;
    now=0xfffffff0U;buffered=3;loop(0,0,buffer,&buffered);sends=0;buffered=3;
    now=0xfffffff0U+49U;loop(0,0,buffer,&buffered);CHECK(sends==0 && buffered==3);
    now=0xfffffff0U+50U;loop(0,0,buffer,&buffered);CHECK(sends==1 && buffered==0);
    return 0;
}
/* Return zero only after both transparent and MAVLink production loops pass. */
int run_tests(void) {
    int result=check_loop(db_read_serial_parse_transparent);if(result)return result;
    return check_loop(db_read_serial_parse_mavlink);
}
'''
START=r'''
.section .text.entry
.global _start
_start:
  la sp, __stack_top
  call run_tests
  slli a0, a0, 16
  beqz a0, 1f
  li t1, 0x3333
  j 2f
1:
  li t1, 0x5555
2:
  or a0, a0, t1
  li t0, 0x100000
  sw a0, 0(t0)
3:
  wfi
  j 3b
'''
LINK='''ENTRY(_start)
SECTIONS {
 . = 0x80000000;
 .text : { *(.text.entry) *(.text*) }
 .rodata : { *(.rodata*) *(.srodata*) }
 .data : { *(.data*) *(.sdata*) }
 .bss : { *(.bss*) *(.sbss*) *(COMMON) }
 . = ALIGN(16); . += 65536; __stack_top = .;
}
'''


def extract(source: str,name: str) -> str:
    """Extract a complete production function ending at a column-zero closing brace, or fail."""
    found=re.search(r'void '+name+r'\([\s\S]*?^\}',source,re.M)
    assert found,name
    return found.group(0)


def run_checks(repo: Path,cc: Path,qemu: Path,out: Path) -> list[dict]:
    """Compile the production loops with mocked I/O and assert configured and negative-control outcomes."""
    env=os.environ.copy()
    tools=Path(os.environ.get('IDF_TOOLS_PATH',str(Path.home()/'.espressif')))
    dll_dirs=sorted(tools.glob('idf-git/*/mingw64/bin'))
    if dll_dirs:env['PATH']=str(dll_dirs[-1])+os.pathsep+env.get('PATH','')
    (out/'start.S').write_text(START);(out/'link.ld').write_text(LINK)
    results=[]
    source=(repo/'main/db_serial.c').read_text()
    header=(repo/'main/db_parameters.h').read_text()
    macro=re.search(r'^#define DB_PARAM_SERIAL_TIMEOUT_MS .*$',header,re.M).group(0)
    loops='\n'.join(extract(source,name) for name in ('db_read_serial_parse_mavlink','db_read_serial_parse_transparent'))
    for negative in (False,True):
        label='fixed20-control' if negative else 'configured'
        c=out/(label+'.c');elf=out/(label+'.elf')
        c.write_text(PREFIX+'\n'+('#define DB_PARAM_SERIAL_TIMEOUT_MS 20U' if negative else macro)+'\n'+loops+'\n'+TESTS)
        command=[str(cc),'-march=rv32imac_zicsr_zifencei','-mabi=ilp32','-mcmodel=medany','-mno-relax','-O0','-fno-builtin','-nostdlib','-Wl,--no-relax','-Wl,-T,'+str(out/'link.ld'),str(out/'start.S'),str(c),'-o',str(elf)]
        compiled=subprocess.run(command,env=env,capture_output=True,text=True,timeout=30)
        assert compiled.returncode==0,compiled.stderr
        run=subprocess.run([str(qemu),'-machine','virt','-bios','none','-kernel',str(elf),'-display','none','-monitor','none','-serial','none'],env=env,capture_output=True,text=True,timeout=10)
        if negative:
            assert 1<=run.returncode<=65535,(label,run.returncode,run.stderr)
        else:
            assert run.returncode==0,(label,run.returncode,run.stderr)
        results.append(dict(control=label,exit_code=run.returncode,expected='fail' if negative else 'pass'))
        print(label,run.returncode,'original defect detected' if negative else '20 scenarios passed')
    return results


def main() -> None:
    """Run serial timeout regression checks without devices or writes to the repository."""
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cc',help='Path to riscv32-esp-elf-gcc')
    parser.add_argument('--qemu',help='Path to qemu-system-riscv32')
    args=parser.parse_args()
    cc=find_tool('riscv32-esp-elf-gcc',args.cc,'riscv32-esp-elf/*/riscv32-esp-elf/bin/riscv32-esp-elf-gcc*')
    qemu=find_tool('qemu-system-riscv32',args.qemu,'qemu-riscv32/*/qemu/bin/qemu-system-riscv32*')
    with tempfile.TemporaryDirectory(prefix='db-serial-timeout-') as temp:
        results=run_checks(REPO,cc,qemu,Path(temp))
    print(json.dumps(results))


if __name__=='__main__':main()
