/* régions mémoire */
MEMORY
{
FLASH (rx)     : ORIGIN = 0x08000000, LENGTH = 64K
RAM (xrw)      : ORIGIN = 0x20000000, LENGTH = 20K
}


SECTIONS
{
    .  = 0x0;         /* From 0x00000000 */

    .text : 
    {
        *(vectors)    /* Vector table */
        *(.text)      /* Program code */
    } >FLASH
    .rodata : 
    {
        *(.rodata)    /* Read only data */
    } >FLASH   
    _DATA_ROM_START = .;

    .  = 0x20000000;  /* From 0x20000000 */      

    _DATA_RAM_START = .;
    .data : AT(_DATA_ROM_START) 
    {
        *(.data)      /* Data memory */
    } >RAM AT> FLASH
    _DATA_RAM_END = .;

    _BSS_START = .;   /* Indicates where BSS section starts in RAM */
    .bss :
    {
        *(.bss)       /* Zero-filled run time allocate data memory */
    } >RAM
    _BSS_END = .;     /* Indicates where BSS section ends in RAM */
}  
