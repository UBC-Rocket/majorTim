# majorTim
The 'Tim' implementation of majorTom.


## Helpful tools

### Build and flash:
```bash
./dmake.sh sharetest && cp builds/sharetest.bin /run/media/${USER}/NODE_F446RE/
```

### Remote debugging
From terminal 1
```bash
st-util
```
From terminal 2
```bash
gdb -ex "target extended-remote localhost:4242" sharetest/BUILD-SHARETEST/mbed.elf
```