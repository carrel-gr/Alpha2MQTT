AlphaSniffer is a quick and dirty RS485 sniffer.  It is basically a modbus sniffer, but it makes some useful assumptions based on the specifics of the AlphaESS modbus protocol.

To run this, place this device on an RS485 bus that has a master and a slave (the AlphaESS).  It will connect to your WiFi (as configured in Definitions.h) and will open a port to a server (also defined with an IP/port in Definitions.h).  It will then send copies of all RS485 traffic in both directions.

You can create a very simple receiving server by running 'netcap -l -p 19999'
or 'nc -k -l 19999 | ts "%b %d %H:%M:%.S" | tee -a .../file.log'

This tool recognizes the 3 modbus message types used by Alpha:  ReadDataRegisters, WriteDataRegisters, and WriteSingleRegister
This tool tries to determine the message direction: TX=master-to-slave, RX=slave-to-master, and UK=unknown/undetermined

Limitations;
- It currently assumes that ReadDataRegisters is always writing just a single register.
- It currently assumes that WriteDataRegisters is writing one or two registers, but can't handle more.
- For WriteDataRegisters, it can't perfectly determine the direction so it will be wrong if a CRC from an RX message has a CRC upper byte value of 0x02 or 0x04.
- For WriteSingleRegister it is unable to determine direction. Thankfully both directions are the same length.
