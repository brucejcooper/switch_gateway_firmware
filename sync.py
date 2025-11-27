#!/usr/bin/env python
import serial
import sys
import os
import time

print()

def escape_string(s):
    # Define escape mappings
    escape_map = {
        '\\': '\\\\',  # Backslash must be first
        '"': '\\"',    # Double quote
        "'": "\\'",    # Single quote
        '\n': '\\n',   # Newline
        '\r': '\\r',   # Carriage return
        '\t': '\\t',   # Tab
        '\b': '\\b',   # Backspace
        '\f': '\\f',   # Form feed
    }
    
    result = []
    for char in s:
        # Use the escape mapping if it exists, otherwise keep the character
        result.append(escape_map.get(char, char))
    
    return ''.join(result)



with serial.Serial('/dev/tty.usbmodem11201', 115200) as ser:
    for file in sys.argv[1:]:
        filename = f"/lua/{os.path.basename(file)}"
        print(f"Writing {file} to {filename}")
        
        
        with open(file, 'r') as f:
            cmd = f'tmpf = io.open("{filename}", "w")\n'.encode('ascii')
            ser.write(cmd)            
            res = ser.readline()
            print(res.decode('ascii'))
            
            for line in f.readlines():
                ser.write(f'tmpf:write("{escape_string(line.rstrip("\r\n"))}\\n")\n'.encode('ascii'))            
                res = ser.readline()
                print(res.decode('ascii'), end="")
                res = ser.readline()
                # time.sleep(0.1)

            ser.write(b"tmpf:flush()\n")
            res = ser.readline()
                
            ser.write(b"tmpf:close()\n")
            res = ser.readline()
            print(res.decode('ascii'))
            res = ser.readline()


                    
    ser.write(b"reboot()\n")
    res = ser.readline()
    print(res.decode('ascii'))
