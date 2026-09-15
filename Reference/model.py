# from sys import stdin, stdout
import sys
import struct
# from machine import Pin
import machine
import time
import config
import modbus
import spiccp

model = None

rts_pin = machine.Pin.board.D7
rts_pin.mode(machine.Pin.OUT)
rts_pin.value(0)


def create_queries(m):
    if m.model_type == 0:  # TypeFC
        m.queries = [
            spiccp.Query(name='machine status', cmd=b'\xC2\x48'),
            spiccp.Query(name='process status', cmd=b'\xC2\x40'),
            spiccp.Query(name='process setpoint', cmd=b'\xC2\x30'),
            spiccp.Query(name='process limit delta', cmd=b'\xC2\x32'),
            spiccp.Query(name='dew trigger', cmd=b'\xC2\x80'),
            spiccp.Query(name='blanket', cmd=b'\xED\x90'),
        ]
    elif m.model_type == 1:  # TypeFD
        m.queries = [
            spiccp.Query(name='machine status', cmd=b'\x20\x48'),
            spiccp.Query(name='process status', cmd=b'\x20\x40'),
            spiccp.Query(name='process setpoint', cmd=b'\x20\x30'),
            spiccp.Query(name='dew trigger', cmd=b'\x20\x80'),
            spiccp.Query(name='process limit delta', cmd=b'\x20\x32'),
            spiccp.Query(name='blanket', cmd=b'\xC2\x2E'),
        ]
    elif m.model_type == 2:  # TypeFN
        m.queries = [
            spiccp.Query(name='machine status', cmd=b'\x20\x48'),
            spiccp.Query(name='process status', cmd=b'\x20\x40'),
            spiccp.Query(name='process setpoint', cmd=b'\x20\x30'),
            spiccp.Query(name='dew trigger', cmd=b'\x20\x80'),
            spiccp.Query(name='process limit delta', cmd=b'\x20\x32'),
            spiccp.Query(name='blanket', cmd=b'\xED\x90'),
        ]
    elif m.model_type == 3:  # TypeADV
        m.queries = [
            spiccp.Query(name='machine status', cmd=b'\x20\x48'),
            spiccp.Query(name='process status', cmd=b'\x20\x40'),
            spiccp.Query(name='process setpoint', cmd=b'\x20\x30'),
            spiccp.Query(name='process temperature', cmd=b'\x20\x70'),
            spiccp.Query(name='dew point', cmd=b'\x20\x7C'),
            spiccp.Query(name='dew trigger', cmd=b'\x20\x80'),
            spiccp.Query(name='process limit delta', cmd=b'\x20\x32'),
            spiccp.Query(name='blanket', cmd=b'\xD0\x84'),
        ]
    elif m.model_type == 4:  # CD
        m.queries = [
            spiccp.Query(name='machine status', cmd=b'\x20\x48'),
            spiccp.Query(name='process status', cmd=b'\x20\x40'),
            spiccp.Query(name='process setpoint', cmd=b'\x20\x30'),
            # spiccp.Query(name='dew trigger', cmd=b'\x20\x80'),
            spiccp.Query(name='process limit delta', cmd=b'\x20\x32'),
            spiccp.Query(name='process temperature', cmd=b'\x20\x70'),
            spiccp.Query(name='return temperature', cmd=b'\x20\x72'),
            # spiccp.Query(name='dew point', cmd=b'\x20\x7C')
        ]
    else:
        pass


def create(model_type):
    print('creating type:', model_type)

    # TODO: init model related queries
    if model_type == 0:
        new_model = TypeFC()
    elif model_type == 1:
        new_model = TypeFD()
    elif model_type == 2:
        new_model = TypeFN()
    elif model_type == 3:
        new_model = TypeADV()
    elif model_type == 4:
        new_model = TypeCD()
    else:
        new_model = None

    if new_model is not None:
        new_model.model_type = model_type
        create_queries(new_model)
        config.current['spi']['model_type'] = model_type

    return new_model


class SpiModel:
    def __init__(self):
        print('initializing', self.__class__.__name__)
        self.data = {}
        self.station_id = config.current['spi']['id']
        self.baudrate = config.current['spi']['baudrate']
        self.model_type = config.current['spi']['model_type']

        self.queries = [
            # spiccp.Query(name='process status', cmd=b'\xC2\x40'),
            # spiccp.Query(name='process setpoint', cmd=b'\xC2\x30'),
            # spiccp.Query(name='dew trigger', cmd=b'\xC2\x80'),
        ]
        self.current_query = 0
        self.counters = {'ok': 0, 'bad_crc': 0}
        # print(self.query, self.counters)
        self.elapsed_time = 0
        self.prev_time = None
        self.rx = bytes([])

    def get_query(self, name):
        q = None
        index = self.get_query_index(name)
        if index is not None:
            q = self.queries[index]
        return q

    def get_query_index(self, name):
        index = None
        try:
            index = [x.name for x in self.queries].index(name)
        except ValueError:
            pass
        return index

    def process(self):
        curr_time = time.ticks_ms()
        if self.prev_time is None:
            self.prev_time = curr_time
            return

        self.elapsed_time += curr_time - self.prev_time
        self.prev_time = curr_time
        if self.elapsed_time < 100:
            return
        self.elapsed_time = 0

        if len(self.queries) == 0:
            return

        q = self.queries[self.current_query]
        if q.switched_at is not None:
            if q.name != 'blanket' and curr_time - q.switched_at < 2000:
                return
            if q.name == 'blanket' and curr_time - q.switched_at < 3000:
                return
            else:
                q.switched_at = None
                q.error = False
                q.completed = False

        rx_chunk = sys.stdin.buffer.read()
        if rx_chunk is None:
            pass
        elif self.rx is None:
            self.rx = rx_chunk
        else:
            self.rx += rx_chunk

        if q.sent:
            expected = q.expected_rx_len()
            # print('expected:', expected)
            if self.rx is not None:
                # print(len(self.rx), '<>', expected)
                pass

            if self.rx is not None and len(self.rx) < expected and not q.is_timeout():
                # q.select and q.selecting and q.selected
                # print('exiting')
                return

        if self.rx is not None:
            print('')
            print('rx%d: ' % len(self.rx), end='')
            print(''.join('{:02x}'.format(x).upper() for x in self.rx))
            # print('rx%d: %s' % (len(self.rx), ''.join('{:02x}'.format(x).upper() for x in self.rx)))

        if q.value2 is not None and not q.select:
            # print('enabling pending write request')
            q.select = True
        # print('%02x' % q.cmd[1], q.select, q.sent, q.selecting, q.selected)

        tx = bytearray([])
        if not q.sent:
            if not q.select or (q.select and not q.selecting and not q.selected):
                tx = q.request(q.select)
                q.sent = True
                q.selecting = q.select
                q.selected = False
                q.completed = False
                # print("req tx%d: %s" % (len(tx), ''.join('{:02x}'.format(x).upper() for x in tx)))
                q.sent_at = curr_time
            elif q.select and q.selected:
                tx = q.request2()
                # print('tx::', tx)
                q.sent_at = curr_time
                q.sent = True
            else:
                print('tp#0')

        elif q.sent:
            if q.is_timeout():
                print('')
                print('TIMEOUT, ',
                      'received:', len(self.rx) if self.rx is not None else '0', 'expected', q.expected_rx_len())
                q.error = True

            if not q.select and not q.selecting:
                # fd_blanket = q.cmd[1] == 0x2E

                if self.rx is not None and len(self.rx) >= q.expected_rx_len():
                    data = (self.rx[10] << 8) + self.rx[11]

                if self.rx is not None and len(self.rx) > 0:
                    crc, ok = q.calc_crc(self.rx)
                    if ok:
                        q.completed = True
                    else:
                        q.error = True
                        config.current['spi']['err_cnt'] = config.current['spi']['err_cnt'] + 1
                        print('crc:%04x' % crc, 'ok:', ok)
            elif q.select and q.selecting and not q.selected:
                # if _rx is not None and len(_rx) >= 6:
                #    print(_rx[5], _rx[6], '0', 0x30, ('0' == 0x30))
                if self.rx is None:
                    # print('tp#1.1')
                    pass
                elif len(self.rx) >= 6 and self.rx[5] == q.DLE and self.rx[6] == 0x84:
                    print('selected', self.rx, q.completed, q.error)
                    q.selected = True
                    q.sent = False
                    q.completed = False
                    q.error = False
                elif self.rx[1] == q.NAK:
                    print('code:%02x nak:%02x' % (self.rx[0], self.rx[1]))
                    q.selected = False
                    q.sent = False
                    q.error = True
                    q.value2 = None
                else:
                    q.error = True
                    q.value2 = None
                    print('tp#1')
                self.rx = None
            elif q.select and q.selecting and q.selected:
                q.sent = False
                q.selecting = False
                q.selected = False
                q.completed = True
                q.value2 = None
            elif self.rx is not None:
                print('tp#2', len(self.rx))
                q.error = True
            # print(len(tx), tx)

            if q.completed and not q.select:
                self.dispatch(q)

            if q.completed or q.error:
                q.sent = False
                q.select = False
                q.selecting = False
                q.selected = False
                self.rx = None
                self.current_query = (self.current_query + 1) % len(self.queries)
                _nq = self.queries[self.current_query]
                print('')
                print('next query:', self.current_query + 1, 'of', len(self.queries), '-', _nq.name)
                # _nq.assemble_request()
                # print(_nq, _nq.cmd)
                # print(self.queries)
                self.queries[self.current_query].switched_at = curr_time

        if len(tx) > 0:
            # print('tx:', tx)
            rts_pin.value(1)
            time.sleep_ms(1)
            for i in range(len(tx)):
                b = bytearray([tx[i]])
                sys.stdout.buffer.write(b)
                if b == 0x04 or b == 0x05:
                    time.sleep_ms(100)
                else:
                    # time.sleep_ms(10)
                    pass

            # sys.stdout.buffer.write(tx)
            time.sleep_ms(5)
            rts_pin.value(0)
            print("tx%d: %s" % (len(tx), ''.join('{:02x}'.format(x).upper() for x in tx)))
            self.rx = None

    def dispatch(self, q):
        self.data['last_query'] = q

        if q is None:
            return False

        cmd_b1 = q.cmd[1]
        # Temperature Setpoint
        if cmd_b1 == 0x30:
            # 10 01 22 20 20 30 20 20 10 02 [ 42 00 00 00 ] 10 03 E5 9C
            param_name = 'temperature_setpoint'
            f = struct.unpack_from('>f', self.rx, 10)[0]
            print('%s: %.2f' % (param_name, f))
            self.data[param_name] = f
            modbus.regs[40010] = self.data[param_name]
        elif cmd_b1 == 0x32:
            # 10 01 22 20 20 30 20 20 10 02 [ 42 00 00 00 ] 10 03 E5 9C
            # 10 01 22 20 20 32 20 20 10 02 [ 42 70 00 00 ] 10 03 45 9F
            param_name = 'temperature limit delta'
            f = struct.unpack_from('>f', self.rx, 10)[0]
            # print(self.__class__.__name__, self.model_type)
            print('%s: %.2f' % (param_name, f))
            self.data[param_name] = f
            modbus.regs[40011] = self.data[param_name]
        elif cmd_b1 == 0x40:
            param_name = 'process_status'
            u16 = struct.unpack_from('>H', self.rx, 10)[0]
            print('%s: %4X' % (param_name, u16))
            self.data[param_name] = u16
            modbus.regs[40013] = self.data[param_name]
        elif cmd_b1 == 0x48:
            # 10012220204820201002000010036334
            # print(self.rx[10:12])
            param_name = 'machine_status'
            u16 = struct.unpack_from('>H', self.rx, 10)[0]
            print('%s: %4X' % (param_name, u16))
            self.data[param_name] = u16
            modbus.regs[40014] = self.data[param_name]
        elif cmd_b1 == 0x80:
            # 100122202080202010020000000010032DD1
            param_name = 'dew_trigger'
            f = struct.unpack_from('>f', self.rx, 10)[0]
            print('%s: %.2f' % (param_name, f))
            self.data[param_name] = f
            modbus.regs[40017] = self.data[param_name]

        return True

    def reg_write_cb(self, reg, value):
        f_name = 'reg_write_cb'
        print(f_name, ':', reg, value)
        if reg == 40006 \
                or ('write_enable' in config.current['modbus'] and config.current['modbus']['write_enable']):
            print(config.current['modbus'])
            pass
        else:
            print(f_name, ': WRITES DISABLED')
            return False

        if reg == 40001:
            # config.current['modbus']['sa'] = int(value)
            pass
        elif reg == 40002:
            self.station_id = int(value)
            if config.current['spi']['id'] != self.station_id:
                config.current['spi']['id'] = self.station_id
                modbus.regs[40002] = self.station_id
                config.update()
                # TODO: re-init SPI protocol

                # ### test ####
                if self.station_id == 1:
                    print('RTS ON')
                    rts_pin.value(1)
                else:
                    print('RTS OFF')
                    rts_pin.value(0)

        elif reg == 40003:
            _baudrate = int(value)
            if config.current['spi']['baudrate'] != _baudrate:
                if config.set_baudrate(_baudrate):
                    self.baudrate = _baudrate
                    config.current['spi']['baudrate'] = self.baudrate
                    config.update()
                    # TODO: re-init SPI protocol

        elif reg == 40004:
            self.model_type = int(value)
            if self.model_type > 20:
                return False
            elif config.current['spi']['model_type'] != self.model_type:
                config.current['spi']['new_model_type'] = self.model_type

        elif reg == 40006:
            config.current['modbus']['write_enable'] = int(value) > 0

        elif reg == 40014:
            q = self.get_query(name='machine status')
            q.value2 = int(value)
            q.select = True

        elif reg == 40017:
            config.current['spi']['dew point alarm trigger'] = value
            q = self.get_query(name='dew trigger')
            if q is not None:
                q.value2 = float(value)
                q.select = True

        return True


class TypeFC(SpiModel):
    def __init__(self):
        super().__init__()
        # print('initializing', self.__class__.__name__)

    def dispatch(self, q):
        # print('type fc dispatch:', q.name, q.cmd[1], q.cmd[0])
        if q.name == 'blanket' and q.cmd[0] == 0xED and q.cmd[1] == 0x90:
            offset = 10
            fmt = '>7f'
            if offset + struct.calcsize(fmt) == len(self.rx) - 4:
                blanket = self.rx[offset: -4]
                ff = struct.unpack_from(fmt, blanket)
                print(ff)
                self.data['process temperature'] = ff[0]
                self.data['regen heat'] = ff[1]
                self.data['return temperature'] = ff[2]
                self.data['regen outlet'] = ff[3]
                self.data['aux1 temperature'] = ff[4]
                self.data['aux2 temperature'] = ff[5]
                self.data['dew point'] = ff[6]

                modbus.regs[40012] = self.data['process temperature']
                modbus.regs[40015] = self.data['return temperature']
                modbus.regs[40016] = self.data['dew point']
                modbus.regs[40018] = self.data['regen heat']
                modbus.regs[40019] = self.data['regen outlet']
                modbus.regs[40020] = self.data['aux1 temperature']
                modbus.regs[40021] = self.data['aux2 temperature']

            else:
                print('offset + struct.calcsize(fmt) != self.rx', offset, struct.calcsize(fmt), len(self.rx))
                print('rx:', self.rx)
        else:
            super().dispatch(q)

    def reg_write_cb(self, reg, value):
        print('fc: reg_write_cb:', reg, value)
        if reg == 40010:
            q = self.get_query(name='process setpoint')
            q.value2 = float(value)
            q.select = True
        elif reg == 40011:
            q = self.get_query(name='process limit delta')
            f = float(value)
            q.value2 = float(f)
            q.select = True
        else:
            return super().reg_write_cb(reg=reg, value=value)


class TypeFD(TypeFC):
    def __init__(self):
        super().__init__()
        # print('initializing', self.__class__.__name__)

    def dispatch(self, q):
        if q.name == 'blanket' and q.cmd[0] == 0xC2 and q.cmd[1] == 0x2E:
            offset = 10
            fmt = 'c28hc'
            '''
            uint8_t ugly_byte1;
            int16_t process_temperature;
            int16_t return_temperature;
            int16_t process_temperature2;
            int16_t return_temperature2;								
            int16_t regen_temp;
            int16_t regen_outlet_temp;				
            int16_t dryer_inlet_temp;				
            int16_t hopper_throat_temp;
            int16_t left_bed_temp;
            int16_t right_bed_temp;				
            int16_t hopper_temp[6];
            int16_t dew_point;
            int16_t reserved[11];
            uint8_t ugly_byte2;	    
            '''
            if offset + struct.calcsize(fmt) == len(self.rx) - 4:
                blanket = self.rx[offset: -4]
                ff = struct.unpack_from(fmt, blanket)
                print(ff)
                self.data['process temperature'] = ff[1]
                self.data['return temperature'] = ff[2]
                self.data['process temperature2'] = ff[3]
                self.data['regen temperature2'] = ff[4]
                self.data['return temperature2'] = ff[5]
                self.data['regen heat'] = ff[6]
                self.data['regen outlet'] = ff[7]
                self.data['dryer inlet temp'] = ff[8]
                self.data['hopper throat temp'] = ff[9]
                self.data['left bed temp'] = ff[10]
                self.data['right bed_ emp'] = ff[11]
                for i in range(6):
                    self.data['hopper temp ' + i] = ff[12 + i]
                self.data['dew point'] = ff[18]

                modbus.regs[40012] = self.data['process temperature']
                modbus.regs[40015] = self.data['return temperature']
                modbus.regs[40016] = self.data['dew point']
                modbus.regs[40018] = self.data['regen heat']
                modbus.regs[40019] = self.data['regen outlet']
                modbus.regs[40020] = self.data['dryer inlet temp']
                modbus.regs[40022] = self.data['process temperature2']
                modbus.regs[40023] = self.data['return temperature2']
                modbus.regs[40025] = self.data['hopper throat temp']
                modbus.regs[40026] = self.data['left bed temp']
                modbus.regs[40027] = self.data['right bed temp']
                for i in range(6):
                    modbus.regs[40028 + i] = self.data['hopper temp ' + i]
        else:
            super().dispatch(q)


class TypeFN(TypeFD):
    def __init__(self):
        super().__init__()


class TypeADV(SpiModel):
    def __init__(self):
        super().__init__()

    def dispatch(self, q):
        print('type ADV dispatch:', q.name, q.cmd[0], q.cmd[1])
        if q.name == 'process temperature':
            f = struct.unpack_from('>f', self.rx, 10)[0]
            print('%s: %.2f' % (q.name, f))
            self.data[q.name] = f
            modbus.regs[40012] = self.data[q.name]
        elif q.name == 'dew point':
            f = struct.unpack_from('>f', self.rx, 10)[0]
            print('%s: %.2f' % (q.name, f))
            self.data[q.name] = f
            modbus.regs[40016] = self.data[q.name]

        if q.name == 'blanket' and q.cmd[0] == 0xD0 and q.cmd[1] == 0x84:
            offset = 10
            fmt = '>8f'
            if offset + struct.calcsize(fmt) == len(self.rx) - 4:
                blanket = self.rx[offset: -4]
                ff = struct.unpack_from(fmt, blanket)
                print(ff)
                self.data['process temperature'] = ff[0]
                self.data['return temperature'] = ff[1]
                self.data['process temperature2'] = ff[2]
                self.data['return temperature2'] = ff[3]
                self.data['leftbed outlet'] = ff[4]
                self.data['rightbed outlet'] = ff[5]
                self.data['leftbed heater'] = ff[6]
                self.data['rightbed heater'] = ff[7]

                modbus.regs[40012] = self.data['process temperature']
                modbus.regs[40015] = self.data['return temperature']
                modbus.regs[40022] = self.data['process temperature2']
                modbus.regs[40023] = self.data['return temperature2']
                modbus.regs[40019] = self.data['leftbed outlet']
                modbus.regs[40021] = self.data['rightbed outlet']
                modbus.regs[40018] = self.data['leftbed heater']
                modbus.regs[40020] = self.data['rightbed heater']
            else:
                print('offset + struct.calcsize(fmt) != self.rx', offset, struct.calcsize(fmt), len(self.rx))
                print('rx:', self.rx)
        else:
            super().dispatch(q)


class TypeCD(TypeADV):
    def __init__(self):
        super().__init__()
        # print('initializing', self.__class__.__name__)

    def dispatch(self, q):
        # print('type fc dispatch:', q.name, q.cmd[1], q.cmd[0])
        if q.name == 'process temperature':
            f = struct.unpack_from('>f', self.rx, 10)[0]
            print('%s: %.2f' % (q.name, f))
            self.data[q.name] = f
            modbus.regs[40012] = self.data[q.name]
        elif q.name == 'return temperature':
            f = struct.unpack_from('>f', self.rx, 10)[0]
            print('%s: %.2f' % (q.name, f))
            self.data[q.name] = f
            modbus.regs[40015] = self.data[q.name]
        elif q.name == 'dew point':
            f = struct.unpack_from('>f', self.rx, 10)[0]
            print('%s: %.2f' % (q.name, f))
            self.data[q.name] = f
            modbus.regs[40016] = self.data[q.name]
        else:
            super().dispatch(q)
