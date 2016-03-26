from datetime import datetime, date, time, timedelta
import re

def make_data(infile, outfile):
    time_sync, hex_array = parse_hex_array_from_file(infile)
    time_ref = get_reference_time(time_sync)
    print("System start time: %s" % time_ref)
    #time_convert(time_sync)
    #print(hex_array)
    bytes_in_chunk = 4
    aligned_array = []
    combined_dict = {}
    last_ts = 0
    temp_list = []
    for chunk in chunks(hex_array, bytes_in_chunk):
        aligned_array.append(chunk)

    for line in aligned_array:
        ts = int('0x' + line[0], 16)
        rssi = int('0x' + line[2], 16)
        epoch = int('0x' + line[1], 16)
        usec = get_ts_in_usec(ts + (epoch * 16777215));
        addr = int('0x' + line[3], 16)
        rssi = rssi - 256 if rssi > 127 else rssi # originally 8bit signed
        if (not usec in combined_dict.keys()):
            combined_dict[usec] = {}
        if addr in combined_dict[usec].keys():
            combined_dict[usec][addr].append(rssi)
            continue
        combined_dict[usec][addr] = [rssi]
        for usec in sorted(combined_dict.keys()):
            for address in combined_dict[usec]:
                print("%s\t%d\t%.2f" % (time_ref + timedelta(microseconds=usec), address, sum(combined_dict[usec][address]) / len(combined_dict[usec][address])))


def get_reference_time(time_dict):
    sec = int(time_dict['sec'])
    usec = int((time_dict['sec'] - sec) * 1000000)
    telephone_time = time(time_dict['hour'], time_dict['min'], sec, usec)
    telephone_date = date.today()
    telephone_datetime = datetime.combine(telephone_date, telephone_time)
    ''' 32kHz RTC is used'''
    ts_usec = time_dict['ts'] / 32768 * 1000000
    zero_point = telephone_datetime + timedelta(microseconds=ts_usec)
    return zero_point

def get_ts_in_usec(ts):
    '''convert ts to microseconds'''
    '''
     convert 1-byte timestamp to microseconds
     To fit into 1 byte, the 32768Hz clock value was divided
     by 0x10101

    '''
    ts2 = ts * 65793 / 32768 * 1000000
    return ts2


def chunks(l, n):
    """Yield successive n-sized chunks from l."""
    for i in range(0, len(l), n):
        yield l[i:i+n]

def parse_hex_array_from_file(infile):
    time_sync = {}
    hex_array = []
    no_time_sync = True
    section_started = False
    with open(infile, "rb") as f:
        for row in f:
            if no_time_sync:
                match = re.search(b'(\d+):(\d+):(.*)\t"ts:(\d+) epoch:(\d+)', row)
                if match:
                    print(row) #"ts:11635953 epoch:0"
                    print("timestamp")
                    print(match.group(1),match.group(2), match.group(3), match.group(4), match.group(5))
                    time_sync['hour'] = int(match.group(1))
                    time_sync['min'] = int(match.group(2))
                    time_sync['sec'] = float(match.group(3))
                    time_sync['ts'] = int(match.group(4))
                    time_sync['epoch'] = int(match.group(5))
                    no_time_sync = False
            if not section_started:
                match = re.search(b'\*\*\*DATA TRANSFER\*\*\*', row)
                if match:
                    section_started = True
            else:
                match = re.search(b'2A-2A-45-4E-44-2A-2A', row) # **END**
                if match:
                    section_started = False
                    pass
                else:
                    match = re.search(b'value: \(0x\) (.*)', row)
                    if match:
                        hex_array.extend(re.split('-', match.group(1).decode('utf-8')))
                    else:
                        pass
    return (time_sync, hex_array)


if __name__ == "__main__":
    make_data("foo.txt", "bar.txt")
    pass
