from serial import Serial

if __name__ == "__main__":
    s = Serial("/dev/ttyUSB0", 38400, timeout=0)
    while True:
        s.write("\xf1\xff\xff\xff\xff")
        print s.read(1024), 
