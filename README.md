# majorTim
The 'Tim' implementation of majorTom.


## Helpful tools

### Build and flash:
Linux:
```bash
./dmake.sh sharetest && cp builds/sharetest.bin /run/media/${USER}/NODE_F446RE/
```
Mac:
```bash
./dmake.sh sharetest && cp builds/sharetest.bin /Volumes/NODE_F446RE
```

### Remote debugging
From terminal 1
```bash
st-util
```
From terminal 2
```bash
cd sharetest/BUILD-SHARETEST && arm-none-eabi-gdb -ex "target extended-remote localhost:4242" mbed.elf && cd -
```

### Serial output
Mac:
```bash
picocom --imap lfcrlf /dev/tty.usbmodem1413
```
Note that the usbmodem#### may be different on different machines. 
Use 'ctrl-a ctrl-x' to terminate.
