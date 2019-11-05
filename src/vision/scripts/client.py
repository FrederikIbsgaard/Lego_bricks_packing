#!/usr/bin/env python

from pymodbus.client.sync import ModbusTcpClient as ModbusClient
import modbus_utilities as utilities
# --------------------------------------------------------------------------- #
# configure the service logging
# --------------------------------------------------------------------------- #
import logging
log = logging.getLogger()
log.setLevel(logging.DEBUG)

UNIT = 0x1

def run_sync_client():
    client = ModbusClient(utilities.IP, port=utilities.PORT)
    client.connect()

    rr = client.read_holding_registers(address=0x10, count=3, unit=UNIT)
    print(rr.registers[0])
    rq = client.write_register(address=0x10, value=2, unit=UNIT)    
    rr = client.read_holding_registers(address=0x10, count=3, unit=UNIT)
    print(rr.registers[0])

    assert(not rq.isError())     # test that we are not an error
    
    client.close()


if __name__ == "__main__":
    run_sync_client()


