import time
import struct
import config
from modbus import crc_lookup_table


my_dict = {}
my_dict.update(dict.fromkeys([0x40, 0x48], 2))
my_dict.update(dict.fromkeys([0x30, 0x32, 0x80, 0x70, 0x72, 0x7C], 4))
my_dict.update(dict.fromkeys([0x90], 7 * 4))  # Model FN
my_dict.update(dict.fromkeys([0x2E], 61))     # Model FD
my_dict.update(dict.fromkeys([0x84], 8 * 4))  # Model ADV


class Query:
    NUL = 0x00
    SOH = 0x01
    STX = 0x02
    ETX = 0x03
    EOT = 0x04
    ENQ = 0x05
    ACK = 0x06
    DLE = 0x10
    NAK = 0x15
    SYN = 0x16
    ETB = 0x17
    RES = 0x20

    def __init__(self, name, cmd, enabled=True, value=0, dev_id=0x22):
        self.name = name
        self.dev_id = dev_id
        self.cmd = cmd
        self.tr1 = 0
        self.value = value
        self.value2 = None
        self.enabled = enabled
        self.sent = False
        self.select = False
        self.selecting = False
        self.selected = False
        self.error = False
        self.completed = False
        self.sent_at = None
        self.switched_at = None
        self.tx = None
        self.assemble_request()

    def __del__(self):
        pass
        # print(self, "deleted")

    def __str__(self):
        # return f"{{{self.name}:{{'cmd':{self.cmd},'value':{self.value}, 'tr1':{self.tr1},'enabled':{self.enabled}}}"
        # return '{%s:{cmd:%s, value:%d, tr1:%d, enabled:%d}}'
        # % (self.name, self.cmd, self.value, self.tr1, self.enabled)
        # ''.join('{:02x}'.format(x).upper() for x in sender
        return self.name

    # def process(self):
    #    pass

    def data_len(self):
        _len = 0
        # print(my_dict)
        if self.cmd[1] in my_dict:
            _len = my_dict[self.cmd[1]]
        else:
            print('missing data len for ', '%.2X %.2X' % (self.cmd[0], self.cmd[1]))
        return _len

    def expected_rx_len(self):
        dl = self.data_len()
        if self.select and not self.selected:
            rx_len = 7
        elif self.select and self.selected:
            rx_len = 2
        elif self.cmd[1] == 0x2E:
            rx_len = dl + 15
        else:
            rx_len = dl + 14
        return rx_len

    def assemble_request(self):
        station_id = config.current['spi']['id']
        self.tx = bytearray([self.EOT, self.dev_id, station_id, self.cmd[0], self.cmd[1], self.RES, self.ENQ])

    def request(self, select=False):
        _tx = self.tx
        if select:
            _tx[4] = _tx[4] | 0x01
        else:
            _tx[4] = _tx[4] & ~0x01
        return _tx

    def request2(self):
        _tx = bytes([self.DLE, self.STX])
        if self.cmd[1] & 0x40 == 0x40:
            data_len = 2
        else:
            data_len = 4

        if type(self.value2) == float:
            b = struct.pack('>f', self.value2)
            print("packed float[%d]: %s" % (len(b), ''.join('{:02x}'.format(x).upper() for x in b)))
            _tx += b
        else:
            for i in range(data_len - 1, -1, -1):
                # print('i=', i)
                _tx += bytes([(self.value2 >> (i*8)) & 0xFF])

        _tx += bytes([self.DLE, self.ETX])
        crc, _ = self.calc_crc(_tx)
        # print(crc)
        _tx += bytes([(crc >> 8) & 0xFF, crc & 0xFF])
        return _tx

    def is_timeout(self):
        if self.sent_at is not None:
            curr_time = time.ticks_ms()
            elapsed = curr_time - self.sent_at
            if elapsed >= 5000:
                return True
        return False

    def calc_crc(self, data):
        ok = False
        crc = 0x0000

        tr_soh = False
        tr_stx = False
        tr_dle = False
        enq_cnt1 = 0
        enq_cnt2 = 0
        eot_cnt1 = 0
        eot_cnt2 = 0

        for i in range(len(data)):
            b = data[i]
            if b == self.ENQ:
                if tr_soh or tr_stx:
                    enq_cnt1 += 1
                else:
                    enq_cnt2 += 1
                    continue
            elif b == self.EOT:
                if tr_soh or tr_stx:
                    eot_cnt1 += 1
                else:
                    eot_cnt2 += 1
                    continue
            elif b == self.DLE:
                tr_dle = True
                continue
            elif b == self.SOH and not tr_soh and tr_dle:
                tr_dle = False
                tr_soh = True
                continue
            elif b == self.STX and not tr_soh and tr_dle:
                tr_dle = False
                tr_stx = True
                continue

            w = crc ^ b
            w = w & 0x00FF
            w = crc_lookup_table[w]
            crc = (crc >> 8) & 0x00FF
            crc = crc ^ w
            # crc = (crc >> 8) ^ spiccp_crc_lookup_table[(crc ^ b) & 0xFF]

            if b == self.ETX and tr_dle:
                tr_dle = False
                if i + 2 < len(data):
                    rx_crc = (data[i+1] << 8) | data[i+2]
                    ok = rx_crc == crc
                break
        return crc, ok
