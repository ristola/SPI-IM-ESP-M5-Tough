# -------------------------------------------------------------------------------
# Name:        xbee3-modbus-spi-gw
# Purpose:     To Translate SPI-IM protocol to Modbus Holding registers.
#
# Author:     Loren Ristola
#
# Created:     15.03.2023
# Copyright:   Ristola Technical Services, LLC
# Licence:     <your licence>
# -------------------------------------------------------------------------------
import errno
import gc
import time
import xbee
import config
import model as m
import modbus
from machine import Pin
import micropython

version = 200


# micropython.kbd_intr(0xFB)
micropython.kbd_intr(-1)

# Pin Setup for Digital Input Monitoring
user_button = Pin("D4", Pin.IN, Pin.PULL_UP)


def print_mem_info():
    print('allocated:%d free:%d' % (gc.mem_alloc(), gc.mem_free()))


def gw_dispatcher():
    gc.collect()

    rssi = xbee.atcmd("DB")
    board_temp = xbee.atcmd("TP")
    board_temp_f = board_temp * 9.0 / 5.0 + 32.0

    modbus.regs[40001] = version
    modbus.regs[40002] = config.current['spi']['id']
    modbus.regs[40003] = config.current['spi']['baudrate']
    modbus.regs[40004] = config.current['spi']['model_type']
    modbus.regs[40005] = rssi
    modbus.regs[40006] = config.current['modbus']['write_enable']
    modbus.regs[40007] = board_temp_f  # Board Temp F
    modbus.regs[40008] = xbee.atcmd("%V") / 100  # Xbee Supply Voltage
    modbus.regs[40009] = config.current['spi']['err_cnt']
    gc.collect()


gc.enable()
print_mem_info()
gc.collect()
print_mem_info()
gc.collect()

tt = config.init()
if tt is not None:
    config.current = tt
    config.current['modbus']['write_enable'] = False
    if 'spi' in config.current and 'model_type' in config.current['spi']:
        new_model_type = config.current['spi']['model_type']
        m.model = m.create(new_model_type)

if m.model is None:
    m.model = m.create(config.current['spi']['model_type'])

print_mem_info()
gc.collect()

config.set_baudrate(baudrate=config.current['spi']['baudrate'])

print_mem_info()
gc.collect()
print_mem_info()

while True:

    if user_button.value():
        pass
    else:
        print("Button Pressed !")
        # Restore Ctrl-C Break Function
        micropython.kbd_intr(3)
        break

    gw_dispatcher()
    rx = xbee.receive()
    if rx:
        # print('rx:', rx)
        sender = rx['sender_eui64']
        payload = rx['payload']
        resp = modbus.rtu(req=payload)
        gc.collect()

        if len(resp) > 0:
            # print("Response %s" % (''.join('{:02x}'.format(x).upper() for x in resp)))
            # print(type(payload), type(resp))
            try:
                xbee.transmit(sender, resp)
            except OSError as e:
                if hasattr(e, 'errno'):
                    if e.errno == errno.ENOTCONN:
                        time.sleep(1)

    # uart_rx = stdin.buffer.read()
    # if uart_rx is not None:
    #    print('uart rx[', len(uart_rx),']:', uart_rx)
    # Wait 100 ms before checking for data again.

    if config.current is not None and 'spi' in config.current and 'new_model_type' in config.current['spi']:
        new_model_type = config.current['spi']['new_model_type']
        del config.current['spi']['new_model_type']
        del m.model
        m.model = m.create(new_model_type)
        config.update()
        print_mem_info()
        gc.collect()
        print_mem_info()
    else:
        gc.collect()
        # mem_free2 = gc.mem_free()
        # if mem_free != mem_free2:
        #    print('freed:%d' % (mem_free2 - mem_free))
        time.sleep_ms(10)
        if m.model is not None:
            m.model.process()
        gc.collect()
