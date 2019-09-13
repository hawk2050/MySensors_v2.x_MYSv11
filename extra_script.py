Import('env')
env.Replace(FUSESCMD="avrdude $UPLOADERFLAGS -e -Ulock:w:0x3F:m -Uhfuse:w:0xDA:m -Uefuse:w:0xFE:m -Ulfuse:w:0xFF:m")