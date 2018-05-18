import serial.tools.list_ports

def list_ports():
    ports = ['/dev/'+port.name for port in serial.tools.list_ports.comports()]
    return ports
