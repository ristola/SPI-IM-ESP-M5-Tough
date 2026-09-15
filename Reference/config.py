import ujson
import uos
import xbee

current = dict(modbus=dict(sa=1, write_enable=False), spi=dict(id=32, baudrate=9600, model_type=1, err_cnt=0))

config_filename = 'config.json'
cf = None


def init():
    cfg = None
    dir_list = uos.listdir()
    if config_filename in dir_list:
        try:
            _cf = open(config_filename, 'r')
            cfg = ujson.load(_cf)
            # print('config:', cfg)
            _cf.close()
        except OSError as exp:
            print(config_filename, exp)

    return cfg


def update():
    # print(uos.getcwdu())
    dir_list = uos.listdir()
    # print(dir_list)

    if config_filename in dir_list:
        try:
            uos.remove(config_filename)
        except OSError as exp:
            print('Remove:', exp)

    try:
        _cf = open(config_filename, 'w')
        ujson.dump(current, _cf)
        # print(cf)
        # _cf.flush()
        _cf.close()
    except OSError as exp:
        print('Open', exp)


def set_baudrate(baudrate):
    bauds = (1200, 2400, 4800, 9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600)
    if baudrate in bauds:
        i = bauds.index(baudrate)
        if i >= 0:
            # print("BD%d" % i)
            xbee.atcmd('BD', i)
            # print('baudrate:', baudrate)
            return True
    return False
