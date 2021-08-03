// XtensaShrink.cpp

// Blinky first

// TODO: where do vectors at 0x40000000 go to?
//    esptool.py dump_mem 0x40000000 l024 vectors.bin
// build firmware with VTABLES_IN_ROM 
// 
//  
// TODO: data arrays: addr + ofs code ??

// ?? no use of LITBASE? for L32R?  Special Reg 5


const bool doCompact = 1;

#define EspTool "C:\\Users\\Admin\\.platformio\\packages\\tool-esptool\\esptool.exe -v -cd nodemcu -cp COM23 -cb 115200 -cf "

/*
https://github.com/esp8266/esp8266-wiki/wiki/Memory-Map

 I/O: 0x3FF00000..0x3FFC0000
 RAM: 0x3FFE0000..0x40000000   (<50 KB of 80KB available??) stack/heap
 ROM: 0x40000000..0x40100000   (x2)
IRAM: 0x40100000..0x40140000   32KB, loaded from flash by bootloader
IROM: 0x40200000..0x40280000   SPI flash
 cfg: 0x60000000..0x60002000

Local Memory     
XLMI start address / size                                     0x3ff00000 / 256K max 
Data ROM start address / size                                 0x3ff40000 / 256K
Data RAM [1] start address / size                             0x3ff80000 / 256K
Data RAM [0] start address / size                             0x3ffc0000 / 256K
   ESP8285/6: 96KB of DRAM space, ~80KB available to user

Instruction RAM [0] start address / size                      0x40000000 / 1M
Exception Vectors:
Level 2 Interrupt Vector (Debug) start address / size         0x40000010 / 0xc
Level 3 Interrupt Vector (NMI vector) start address / size    0x40000020 / 0xc
Kernel (Stacked) Exception Vector start address / size        0x40000030 / 0x1c
User (Program) Exception Vector start address / size          0x40000050 / 0x1c
Double Exception Vector start address / size                  0x40000070 / 0x10
Reset                                                         0x40000080

Instruction RAM [1] start address / size                      0x40100000 / 1M
64K IRAM has one dedicated 32K block of IRAM and two 16K blocks of IRAM. The last two 16K blocks of IRAM can cache flash memory, ICACHE.
Data access in IRAM or ICACHE must always be a full 32-bit word and aligned.

Instruction ROM start address / size                          0x40200000 / 1M

Reset Vector start address / size                             0x50000000 / 0x300

Memmory-Mapped I/O Registers
Base      Size	Name	Description
60000000h	80h	  uart0	UART0 config registers, see examples/IoT_Demo/include/drivers/uart_register.h
60000100h	100h	spi1	SPI controller registers, see examples/IoT_Demo/include/driver/spi_register.h
60000200h	100h	spi0	SPI controller registers, see examples/IoT_Demo/include/driver/spi_register.h
60000300h	74h	  gpio	Timer config registers, see include/eagle_soc.h
60000600h	28h	  timer	Tmer config registers, see include/eagle_soc.h
60000700h	A4h	  rtc	  RTC config registers, see include/eagle_soc.h
60000800h	44h	  iomux	IO MUX config registers, see include/eagle_soc.h

iomux Pin Registers (60000804h–60000843h)
31    24       16        8        0
-------- -ffff--- -------- ud--UDEe
          `- Function      ||  |||`- Output Enable
                           ||  ||`- Output Enable during sleep
                           ||  |`- Pull-down during sleep
                           ||  `- Pull-up during sleep
                           |`- Pull-down
                           `- Pull-up

60000d00h	>=8	  i2c	  I2C controller registers, see ROM functions rom_i2c_readReg, rom_i2c_writeReg
60000F00h	80h	  uart1	UART1 config registers, see examples/IoT_Demo/include/drivers/uart_register.h
60001000h	100h	rtcb	RTC backup memory, see rtc_mem_backup
60001100h	100h	rtcs	RTC system memory, see system_rtc_mem_write


https://arduino-esp8266.readthedocs.io/en/latest/mmu.html

Code in IROM runs at 1/4 - 1/12 speed since SPI flash (four data lines) is not nearly as fast as the internal static IRAM.
Interrupt handlers and code that writes to flash must be run from IRAM.

ICACHE_FLASH_ATTR -> place in IROM
*/

#include <direct.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <windows.h>
#include <share.h>

// platform-espressif8266-develop\examples\esp8266-nonos-sdk-blink\.pio\build\nodemcuv2\firmware.bin     Blinky
// Tasmota-development-9.5\.pio\build\tasmota-lite\firmware.bin      Function pointers

bool errorTrace;

const int base = 0x40100000;
int topAddr; // relative to base

typedef enum {cnst, io, dram, rom, iram, irom, cfg} Region;

Region region(int addr, bool noBase = false) {
  if (!noBase && addr >= -(0x100000) && addr < 0x180000) addr += base;  // imm18 -> 19 bits into irom

  switch (addr & 0xFFF00000) {
    case 0x3FF00000: return addr >= 0x3FFE0000 ? dram : io;
    case 0x40000000: return rom;
    case 0x40100000: return iram;
    case 0x40200000: return irom;
    case 0x60000000: return cfg;
    default: return cnst; // const
  }
}

const int MaxCodeSize = 4000000;  // could malloc / compact segments 0x3FFE0000 0x40100000 0x40200000

typedef enum {unk, data, ptr, ill, err, code, other, l32r, call0, callx0, callxn, ret, branch, jmp, jx, code1, code2, imm6, imm8, imm12, imm16, imm18, imm20} Mark;

BYTE text[MaxCodeSize];
Mark mark[MaxCodeSize];

const int ShownShift = 0;  // min routine 8 bytes
BYTE shown[MaxCodeSize >> ShownShift];  // better a bit field

// https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/system/app_image_format.html

#define ESP_IMAGE_HEADER_MAGIC 0xE9 /*!< magic word for the esp_image_header_t structure. */

typedef BYTE uint8_t;
typedef DWORD uint32_t;

#pragma pack(1)
struct {
  uint8_t magic;              /*!< Magic word ESP_IMAGE_HEADER_MAGIC  */
  uint8_t segment_count;      /*!< Count of memory segments */
  uint8_t spi_mode;           /*!< flash read mode (esp_image_spi_mode_t as uint8_t) */
  uint8_t spi_speed : 4;      /*!< flash frequency (esp_image_spi_freq_t as uint8_t) */
  uint8_t spi_size  : 4;      /*!< flash chip size (esp_image_flash_size_t as uint8_t) */
  uint32_t entry_addr;        /*!< Entry address */

#ifdef ESP32
  uint8_t wp_pin;             /*!< WP pin when SPI pins set via efuse (read by ROM bootloader,
                              * the IDF bootloader uses software to configure the WP
                              * pin and sets this field to 0xEE=disabled) */
  uint8_t spi_pin_drv[3];     /*!< Drive settings for the SPI flash pins (read by ROM bootloader) */
  short chip_id;               /*!< Chip identification number */
  uint8_t min_chip_rev;       /*!< Minimum chip revision supported by image */
  uint8_t reserved[8];        /*!< Reserved bytes in additional header space, currently unused */
  uint8_t hash_appended;      /*!< If 1, a SHA256 digest "simple hash" (of the entire image) is appended after the checksum.
                              * Included in image length. This digest
                              * is separate to secure boot and only used for detecting corruption.
                              * For secure boot signed images, the signature
                              * is appended after this (and the simple hash is included in the signed data). */
#endif
} esp_image_header;

#define ESP_IMAGE_MAX_SEGMENTS 16 

struct {
  uint32_t load_addr;     /*!< Address of segment */
  uint32_t data_len;      /*!< Length of data */
} esp_image_segment_header[ESP_IMAGE_MAX_SEGMENTS];

//offset for 0 Segment = sizeof(esp_image_header_t) + sizeof(esp_image_segment_header_t).
//offset for 1 Segment = offset for 0 Segment + length of 0 Segment + sizeof(esp_image_segment_header_t).
//offset for 2 Segment = offset for 1 Segment + length of 1 Segment + sizeof(esp_image_segment_header_t).
//...

// https://github.com/espressif/esptool/wiki/Firmware-Image-Format

const int RAMbase = 0x3FFE8000;
int ram[2048 / sizeof(int)];

BYTE* segBase(int addr) {
  return region(addr) == iram ? text - base : (BYTE*)ram - RAMbase;
}

void readBin(const char* binPath) {
  int totalLen = 0;
  FILE* bin = fopen(binPath, "rb");
  fseek(bin, 0, SEEK_END);
  int fLen = ftell(bin);
  fseek(bin, 0, SEEK_SET);

  while (1) {
    if (fread(&esp_image_header, sizeof(esp_image_header), 1, bin) != 1) break;
    if (esp_image_header.magic != 0xE9) break;

    printf("Entry @ %X\n", esp_image_header.entry_addr);

    for (int i = 0; i < esp_image_header.segment_count; ++i) {
      fread(&esp_image_segment_header[i], sizeof(esp_image_segment_header[0]), 1, bin);
      int endAddr = esp_image_segment_header[i].load_addr + esp_image_segment_header[i].data_len;
      printf("Segment @ %X to %X, len %6d\n", esp_image_segment_header[i].load_addr, endAddr, esp_image_segment_header[i].data_len);
      totalLen += esp_image_segment_header[i].data_len;
      if (endAddr > topAddr) {
        topAddr = endAddr;
        if (topAddr > base + MaxCodeSize) {
          printf("increase MaxCodeSize\n");
          exit(9);
        }
      }

      int ofs = esp_image_segment_header[i].load_addr;
      fread(segBase(ofs) + ofs, 1, esp_image_segment_header[i].data_len, bin);
    }

    int fPos = ftell(bin);
    if (fPos < 0) break;
    fPos = (fPos | 0xFFF) + 1;  // concatenation: 4K boundaries
    if (fPos >= fLen) break;

    if (fseek(bin, fPos, SEEK_SET)) break;
  }

  topAddr -= base;
  fclose(bin);

  printf("\t\t\t  Total len %6d\n\n", totalLen);
}

int checksum = 0xEF; // seed

void writeAndSum(const void* data, int len, FILE* bin) {
  fwrite(data, len, 1, bin);
  BYTE* d = (BYTE*)data;
  for (int i = len; i--;)
    checksum ^= *d++;
}

int first, last;

void writeBin(const char* binPath) {
  FILE* bin = fopen(binPath, "wb");

#if 0
  esp_image_segment_header[0].load_addr = (base + first + 3) & 0xFFFFFFFC;
  esp_image_segment_header[0].data_len =  (last + 1 - first + 3) & 0xFFFFFFFC;
  // esp_image_header.segment_count = 1; // DRAM content - for loader??
#endif

  fwrite(&esp_image_header, sizeof(esp_image_header), 1, bin);

  for (int i = 0; i < esp_image_header.segment_count; ++i) {
    fwrite(&esp_image_segment_header[i], sizeof(esp_image_segment_header[0]), 1, bin);
    int ofs = esp_image_segment_header[i].load_addr;
    writeAndSum(segBase(ofs) + ofs, esp_image_segment_header[i].data_len, bin);
  }

  int fill = 15 - ftell(bin) % 16;
  const int filler[8] = {0};
  fwrite(&filler, 1, fill, bin);
  fwrite(&checksum, 1, 1, bin);  // checksum byte at end of a sixteen byte padded boundary
  fclose(bin);
}

// outputs differ - FPU option
const char objDump[] = "C:\\users\\Admin\\.platformio\\packages\\toolchain-xtensa\\bin\\xtensa-lx106-elf-objdump.exe"; 
// const char objDump[] = "xtensa-esp32-elf\\bin\\xtensa-esp32-elf-objdump.exe"; // FPU
// const char objDump[] = "C:\\users\\Admin\\.platformio\\packages\\toolchain-xtensa@2.40802.200502\\bin\\xtensa-lx106-elf-objdump.exe";
// const char objDump[] = "C:\\users\\Admin\\.platformio\\packages\\toolchain-xtensa@1.40802.0\\bin\\xtensa-lx106-elf-objdump.exe";  // 2015

char disasmElfPath[256];

void disasmAt(const char* symbol) {
  printf("%s\n", symbol);
  char disasmCmd[512];
  sprintf_s(disasmCmd, sizeof(disasmCmd), "%s --disassemble=%s %s >> disasm.txt", objDump, symbol, disasmElfPath);
  system(disasmCmd);
  system("echo . >> disasm.txt");  // separator 
}

bool traversed;

void doDisasm(int startAddr, int stopAddr) {
  char disasmCmd[512];
  sprintf_s(disasmCmd, sizeof(disasmCmd), 
            "%s -d --start-address=0x%X --stop-address=0x%X %s >> disasm.txt", objDump, base + startAddr, base + stopAddr, disasmElfPath);
  system(disasmCmd);
  system("echo . >> disasm.txt");  // separator 
}

void disasmAt(int startAddr, int stopAddr = 0) {
  if (startAddr > 0x30000000) startAddr -= base;
  if (stopAddr > 0x30000000) stopAddr -= base;
  if (!stopAddr) stopAddr = startAddr + 32;

  //if (shown[startAddr >> ShownShift]) return;
  shown[startAddr >> ShownShift] = 1;

  if (!traversed) {
    doDisasm(startAddr, stopAddr);
    return;
  }

  int blockEnd = startAddr;
  do {
    char hex[64 * 3] = "echo ";  // command line limit
    _itoa(base + startAddr, hex + 5, 16); 
    hex[5 + 8] = ':';
    char* pHex = hex + 5 + 8 + 1;
    while (mark[blockEnd] < code && blockEnd < stopAddr) { // errorTrace data
      if (pHex < hex + sizeof(hex) - 32) 
        pHex += sprintf(pHex, " %02X", text[blockEnd]);
      // else ... once
      blockEnd++;
    }
    if (blockEnd > startAddr) {
      strcpy(pHex, " >> disasm.txt");
      system(hex);
    }

    if (blockEnd >= stopAddr) return;
    startAddr = blockEnd;

    while (mark[blockEnd] >= code && blockEnd < stopAddr) ++blockEnd;
    doDisasm(startAddr, blockEnd);
    startAddr = blockEnd;

  } while (blockEnd < stopAddr);
}

void cleanDisasm() {
  FILE* disasm = _fsopen("disasm.txt", "rt", _SH_DENYNO);
  if (!disasm) exit(6);

  FILE* clean = fopen("disasm.clean.txt", "wt");
  if (!clean) exit(6);

  char l[256];
  while (fgets(l, sizeof(l), disasm)) {
    if (isdigit(l[0])) {
      char* l32rPos = strstr(l, "l32r");
      if (l32rPos && *(l32rPos + 9 + 8) != ':') {
        int ref;
        sscanf(l32rPos + 9, "%X", &ref);
        if (region(ref, true) == iram) {
          int load = *(int*)(text + ref - base);
          *(l32rPos + 9 + 8) = ':';
          _itoa(load, l32rPos + 9 + 9, 16);
          strcat(l, "\n");
        }
      }
      fputs(l, clean);
    } else if (l[0] > ' ' && l[0] != 'D' && l[0] != 'p' && l[0] != 'T')
      fputs(l, clean);

    if (strstr(l, ", f")) fputs("                      NO FPU !!           ^^^\n", clean);
  }

  fclose(clean);
  fclose(disasm);
}

int imm6Dest(int addr) {
  int ofs = *(unsigned short*)(text + addr);
  ofs = (ofs & 0x30) | (ofs & 0xF000) >> 12; //  6 bits forward only
  return addr + ofs + 4;
}

int imm8Dest(int addr) {
  return addr + 4 + (signed char)text[addr + 2]; // most - signed 8 bits
}

int imm12Dest(int addr) {
  return addr + 4 + (*(short*)(text+addr+1) >> 4); // sign extended
}

int branchDest(int addr) {
  if (text[addr] & 0x8) { // narrow BEQZ.N BNEZ.N 
    mark[addr + 1] = imm6;
    return imm6Dest(addr);
  } else if ((text[addr] & 0x1F) == 0x16  // BEQZ BNEZ BGEZ BLTZ signed imm12   ?? other instrs?
    && text[addr] != 0xB6 // BLTUI
    && text[addr] != 0xF6 ) { // BGEUI    
    mark[addr+1] = imm12;
    return imm12Dest(addr);
  } else {
    mark[addr+ 1] = imm8;
    return imm8Dest(addr);
  }
}

int imm16Dest(int addr) {
  return ((addr + 3) & ~3) + ((0xFFFF0000 | *(short*)(text + addr + 1)) << 2);  // imm16 backwards
}

int imm18ofs(int addr) {
  return (*(int*)(text + addr) << 8) >> (32 - 18); // sign-extended 18 bits
}

int jmpDest(int addr) {
  mark[addr + 1] = imm18;
  return addr + imm18ofs(addr) + 4;
}

int call0Dest(int addr) {
  mark[addr + 1] = imm20;
  return (addr & ~3) + (imm18ofs(addr) << 2) + 4;
}

int l32rDest(int addr) {
  mark[addr + 1] = imm16;
  int dest = imm16Dest(addr);
  if (dest < 0) {
    printf("%X:%X l32r ref ??\n", base + addr, base + dest); 
    return dest;
  }
  mark[dest] = ptr; // TODO: plus maybe more following in array

  int target = *(int*)(text + dest);
  return target - base;  // indirect
}

void addrAt(int addr) {
  if (addr < base) addr += base;
  printf("%X:%X %d\n", addr, *(int*)(text + addr - base), mark[addr - base]);
}

unsigned short mapOfs[MaxCodeSize >> 2]; // compaction offset >> 2

int displ(int addr) { // compaction displacement
  if (region(base + addr, true) != iram) {  // call to library
    return 0;
  }
  return mapOfs[addr >> 2] << 2;
}

void compact() {
  int ofs = 0;
  int addr;
  Mark lastMark = code;
  for (addr = first; addr <= last; addr += 4) {
    mapOfs[addr >> 2] = ofs;
    if (mark[addr]) lastMark = mark[addr];
    if (lastMark >= code && !mark[addr] && !mark[addr+1] && !mark[addr+2] && !mark[addr+3])
      ++ofs; // hole
    // else printf("%4X->%4X  ", addr & 0xFFFF, (addr - (ofs << 2)) & 0xFFFF);  // map
  }

  int relo = first;
  for (addr = first; addr <= last;) {
    bool narrow = text[addr] & 0x8;  // MSB set = narrow p 575
    if (mark[addr] == unk || mark[addr] == err) {
      ++addr;
      continue;
    }

    if (addr == 0x2035)
      printf(".");  

    int iDispl = displ(addr);
    relo = addr - iDispl; // relocated location

    switch (mark[addr]) {
      case data:
      case ptr : {
        int dest = *(int*)(text + addr);
        *(int*)(text + relo) = dest - (region(dest, true) == iram ? displ(dest - base) : 0);
        addr += 4;
        }
        break;

      default:
        text[relo]   = text[addr];
        text[relo+1] = text[addr+1];
        if (!narrow)
          text[relo+2] = text[addr+2];        

        int ofs;
        switch (mark[addr + 1]) { 
          case imm6:  // forward offset
            ofs = imm6Dest(addr) - displ(imm6Dest(addr)) - (relo + 4);
            text[relo] = text[relo] & 0xCF | ofs & 0x30;  // MS 2 bits
            text[relo+1] = text[relo + 1] & 0x0F | (ofs & 0xF) << 4; // LS nibble -> MS nibble
            break; 
          case imm8: text[relo+2] += iDispl - displ(imm8Dest(addr)); break; // signed
          case imm12: *(int*)(text+relo) += (iDispl - displ(imm12Dest(addr))) << (24 - 12); break; 
          case imm16: *(signed short*)(text+relo+1) += (iDispl - displ(imm16Dest(addr))) >> 2; break;  // back offset (one-extended negative)
          case imm18: *(int*)(text+relo) += (iDispl - displ(jmpDest(addr))) << (24 - 18); break;
          case imm20 : *(int*)(text+relo) += (iDispl - displ(call0Dest(addr))) << (24 - 20); break;
        }
        addr += narrow ? 2 : 3;
        break;
    }   
  }
  last = relo;

  printf("\nPacked to %d bytes\n", last - first);

  traversed = false;
  memset(mark, unk, sizeof(mark));  // or move marks
  memset(shown, 0, sizeof(shown));
}

Mark instrType(int addr) { 
  if (addr < 0) return ill;
  int op = text[addr];
  if ((op & 0x0F) == 1) return l32r;
  if ((op & 0x3F) == 5) return call0;
  if ((op & 0x3F) == 6) return jmp;
  if (op == 0x36) return other;  // entry
  if (op == 0x76) return other;  // lsi
  // more??
  if ((op & 0x0E) == 6) return branch;  // (after jmp, entry handled above)
  if ((op & 0x8F) == 0x8C) return branch; // BEQZ.N BNEZ.N

  int instr = *(int*)(text + addr) & 0xFFFFFF;
  if (instr == 0x80) return ret; // normal encoding
  if (instr == 0x00) return ill;

  int anyReg = instr & 0xFFF0FF;
  if (anyReg == 0x80) return ret;  // in data
  if (anyReg == 0xA0) return jx;
  if (anyReg == 0xC0) return callx0;
  if (anyReg == 0x3010) return ret; // rfi
  if (anyReg == 0x3000) return ret; // rfe
  if (anyReg == 0xF17010) return ret; // rfdd
  // TODO: more return types!!


  if ((anyReg & 0xFFFFCF) == 0xC0) return callxn;  // not used

  int narrow = instr & 0xFFFF;
  if (narrow == 0xF00D) return ret;  // ret.n
  if (narrow == 0xF06D) return ill;
  return other;
}

int origAddr(int addr) {
  int origAddr = 0;
  while (1) {
    int ofs = addr - (origAddr - displ(origAddr));
    if (ofs <= 0) break;
    ++origAddr; // better faster convergence!!
  }
  return origAddr;
}

void onError(int addr, const char* type) {
  char cmd[64];
  sprintf_s(cmd, sizeof(cmd), "echo %s: >> disasm.txt", type);
  system(cmd);

  int stopAddr = addr;
  for (int i = 4; i--;)
    stopAddr += text[stopAddr] & 0x8 ? 2 : 3;

  // mark[addr] = err;

  printf("%6s @ %X(%X)\n", type, base + addr, base + origAddr(addr));
  disasmAt(addr, stopAddr);
}

void onError(int addr, int dest, int failedAt, const char* type) {
  char cmd[128];
  int orig = base + origAddr(addr);
  sprintf_s(cmd, sizeof(cmd), "echo %s @ %X(%X) to %X..%X >> disasm.txt", type, addr + base, orig, dest + base, failedAt + base);
  system(cmd);

  if (dest >= 0 && failedAt >= dest) {
    for (int i = 4; i--;)
      failedAt += text[failedAt] & 0x8 ? 2 : 3;
    disasmAt(dest, failedAt);
  } else disasmAt(addr);

  printf("%6s @ %X(%X)->%X..%X\n", type, base + addr, orig, base + dest, base + failedAt);
}

void dumpCode(int addr) { // disasm from addr to ret, ill, or jmp
  static int endAddr;
  if (abs(addr - endAddr) > 3)
    system("echo }{ >> disasm.txt");

  endAddr = addr;
  while (1) {
    int iType = instrType(endAddr);
    endAddr += text[endAddr] & 8 ? 2 : 3;
    if (iType == ret || iType == ill || iType == jmp) break;
  }
  disasmAt(addr, endAddr);
}


int routineAt(int start, int stop = 0) {
  switch (region(start)) {
    case iram:
    case irom:
    case rom : return 0;
  }

  return start;

  start &= 0xFFFFFFFC;
  int retPos = start;
  while (1) {
    while (*(unsigned short*)(text + retPos) != 0xF00D) ++retPos; // or ill
    if (stop && retPos > stop) return 0;
    disasmAt(start, retPos + 2);
    if (!stop) return 0;
    retPos = start = (retPos + 2 + 3) & 0xFFFFFFFC;
  }
  return 0; // OK
}

int traverse(int addr) {  // returns fail addr
  while (1) {
    if (region(addr) != iram) return 0;

    switch (mark[addr]) {
      case unk: break;  // still to traverse
      case imm6:
      case imm8:
      case imm12:
      case imm16:
      case imm18:
      case imm20:
      case code1: onError(addr - 1, "mid1"); return addr;  
      case code2: onError(addr - 2, "mid2"); return addr;  
      case ptr: 
      case data: return 0;
      case err: return 0; // already reported
      default : return 0; // previously traversed
    }

    bool narrow = text[addr] & 0x8;  // MSB set = narrow p 575
    mark[addr] = instrType(addr);
    mark[addr+1] = code1;
    if (!narrow) mark[addr+2] = code2;

    int dest = 0;
    int failedAt = 0;

    switch (mark[addr]) {
      case other : break;
      case l32r:
        dest = l32rDest(addr);
        if (region(dest) == iram) {
          if ((failedAt = traverse(dest))) { // fn ptr, ...
            onError(addr, dest, failedAt, "l32r");
            return addr;
          }
        }
        break;

      case call0: 
        dest = call0Dest(addr);
        if (routineAt(dest)) {
          onError(addr, dest, addr, "call0 oob");
          return addr;
        }
        
        if ((failedAt = traverse(dest))) {
          onError(addr, dest, failedAt, "call0");
          return addr;
        }
        break;

      case callx0:  
        if ((text[addr - 3] & 0xF) == 1
         && (text[addr - 3] & 0xF0) >> 4 == (text[addr + 1] & 0xF)) { // l32r same register
          dest = l32rDest(addr - 3);  // indirect   // ? intervening instructions possible?
          if (routineAt(dest)) {
            onError(addr - 3, dest, l32rDest(addr - 3), "callx0 oob");
            return errorTrace ? addr : 0;  
          }
          
          if ((failedAt = traverse(dest))) { // already traversed by l32r
            onError(addr, dest, failedAt, "callx0");
            return addr;
          }
        } // else printf("%X: callx0 well after l32r\n", base + addr); // OK - could keep track of registers
        break;
      case ret: return 0;

      case branch: 
        dest = branchDest(addr);
        if ((failedAt = traverse(dest))) {
          onError(addr, dest, failedAt, "branch");
          return addr;
        }
        break;

      case jx: 
      #if 1
        {
          int switchTbl = 0;
          // TODO: other possibile intervening instructions ****
          if (instrType(addr-8) == l32r) 
            switchTbl = l32rDest(addr - 8); 
          else if (instrType(addr-13) == l32r) 
            switchTbl = l32rDest(addr - 13); 
                    
          // table of jmps: 3 bytes per entry
          if (region(switchTbl) == iram) while (1) {
            int iType = instrType(switchTbl);
            if (iType < call0 || iType > jx) return 0;
            // printf("s%X ", base + switchTbl);
            if ((failedAt = traverse(switchTbl))) {
              onError(addr, dest, failedAt, "jx tbl");
              return addr;
            }  
            switchTbl += 3;      
          }
          return 0;
        } 
      #else
        onError(-base, addr - 8, addr, "jx");  return 0;  // errorTrace more context before jx   jmp to base + at * 3  (jx instrs)
      #endif
        break;

      case jmp:
        if (text[addr] == 0xA0) { // jx <reg>
          onError(addr, "jx reg"); 
          return addr;
        }
        addr = jmpDest(addr);
        continue;

      case ill: 
        if (addr < 0x80) return 0; // vector calls don't return
        onError(addr, "ill");
        return errorTrace ? addr : 0;

      default: onError(addr, "mark"); return addr;
    }

    addr += narrow ? 2 : 3;
  }
  return 0;
}

char elfPath[256];

void copyToElf(int start = 0, int len = 0) {  // only for objdump disasm!!
  FILE* origElf = fopen(elfPath, "rb");
  FILE* shrinkElf = fopen(disasmElfPath, "wb");
  char elfHeader[0x7E0]; // first seg @ offset 7E0; checksums??

  fread(elfHeader, sizeof(elfHeader), 1, origElf);
  fwrite(elfHeader, sizeof(elfHeader), 1, shrinkElf);

  fseek(origElf, len, SEEK_CUR);
  fwrite(text + start, 1, len, shrinkElf);

  while (1) { //copy rest
    size_t len = fread(elfHeader, 1, sizeof(elfHeader), origElf);
    fwrite(elfHeader, 1, len, shrinkElf);
    if (len < sizeof(elfHeader)) break;
  }
  fclose(origElf);
  fclose(shrinkElf);

  // https://github.com/jsandin/esp-bin2elf
  // https://github.com/jsandin/esp-bin2elf/blob/master/esp_bin2elf.py
}

int countMarked() {
  int marked = 0;
  for (int addr = 0; addr < topAddr; addr += 4)
    if (mark[addr]) {
      if (!marked) {
        first = addr;
        printf("\nFirst %X\n", base+first);
      }
      marked += 4;
      last = addr;
    }
  last += 2; // ret
  printf("Last  %X\n", base + last);
  printf("%d bytes marked\n\n", marked);
  return marked;
}

void reloRam() {
  for (int a = 0; a < sizeof(ram) / sizeof(ram[0]); ++a) {
    if (region(ram[a], true) == iram) {
      printf("ram %X:%X\n", RAMbase + a, ram[a]);
      ram[a] -= displ(ram[a]);
    }
  }
}

int main(int argc, char** argv) {
  if (_chdir("..")) exit(3);
  system("echo . > disasm.txt");

  for (int bin = 1; bin < argc; bin++)
    readBin(argv[bin]);

  char binPath[256];
  strcpy(binPath, argv[1]);

  strcpy(elfPath, argv[1]);
  strcpy(strrchr(elfPath, '.'), ".elf");

  strcpy(disasmElfPath, elfPath);
  strcat(disasmElfPath, ".shrink.elf");
  copyToElf();

  int failedAt;
  if ((failedAt = traverse(esp_image_header.entry_addr - base)))
    onError(failedAt, "top");
  for (int vect = 0x10; vect <= 0x80; vect += 0x10)   // relocated via wsr.vecbase 0x40100000
    if (*(int*)(text + vect)) 
      traverse(vect);
    // else printf("Vect %X no code\n", vect);

  traversed = true;
  int markedOrig = countMarked();

  disasmAt(first, last + 1);
  cleanDisasm();
  system("copy disasm.clean.txt disasm.txt >> NUL");

  system("echo . >> disasm.txt");
  system("echo ********* PACKED BELOW ************* >> disasm.txt");
  system("echo . >> disasm.txt");

  if (doCompact) {
    compact();
    reloRam();
  }

  copyToElf(first, last - first + 1);
  errorTrace = true;
  traverse(esp_image_header.entry_addr - base - displ(esp_image_header.entry_addr - base));
  traversed = true;
  int markedShrink = countMarked();
  disasmAt(first, last + 1);

  cleanDisasm();

  if (markedOrig == markedShrink) {
    strcat(binPath, ".shrink.bin");
    writeBin(binPath);
    char espCmd[512] = EspTool;
    strcat(espCmd, binPath);
    system(espCmd);
  }
}

/*
Flashed to 4096 byte sectors according to partition table
 
Compressed 196136 bytes to 144322...  
Writing at 0x00010000... (11 %)
Writing at 0x00030000... (100 %)
Wrote 196136 bytes (144322 compressed) at 0x00010000 in 1.6 seconds (effective 990.7 kbit/s)...     irom0text.bin  (once - no magic E9)

Compressed 128 bytes to 75...
Writing at 0x003fc000... (100 %)
Wrote 128 bytes (75 compressed) at 0x003fc000 in 0.0 seconds (effective 258.5 kbit/s)...   config 

Compressed 4096 bytes to 26...
Writing at 0x003fe000... (100 %)
Wrote 4096 bytes (26 compressed) at 0x003fe000 in 0.0 seconds (effective 10950.4 kbit/s)... config 

Compressed 26464 bytes to 19810...
Writing at 0x00000000... (50 %)
Writing at 0x00004000... (100 %)
Wrote 26464 bytes (19810 compressed) at 0x00000000 in 0.2 seconds (effective 1329.6 kbit/s)...
*/




// https://richard.burtons.org/2015/05/17/esp8266-boot-process/
// https://github.com/raburton/esptool2  Windows C


