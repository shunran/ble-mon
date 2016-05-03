#!python
import matplotlib.pyplot as plt
from matplotlib import dates
from datetime import datetime, date, time, timedelta
import argparse
import numpy as np
def draw_pd(infile, outfilename):
    timeconverter = lambda x: datetime.strptime(x.decode('ascii'), '%Y-%m-%d %H:%M:%S.%f')
    my_data = np.genfromtxt(infile, delimiter='\t', dtype=None, names=['time', 'address', 'avg_rssi', 'median_rssi', 'count'],
                         converters={'time': timeconverter})

    def address_to_color(addresses):
        colors = ['b', 'g', 'r', 'c', 'm', 'y', 'b', 'w']
        result = []
        for address in addresses:
            result.append(colors[address])
        return result

    def convert_addresses(addresses):
        result = []
        for address in addresses:
            result.append(address-16)
        return result

    def get_distinct_sorted_addresses(addresses):
        result = []
        for address in addresses:
            if address not in result:
                result.append(address)
        result.sort()
        return result

    def get_legend_labels(legend_index_list):
        result = []
        for a in legend_index_list:
            result.append("Actor %d" % a)
        return result

    def get_legend_handles(addresses):
        result = []
        colors = address_to_color(addresses)
        for c in colors:
            result.append(plt.Circle((0,0), color=c, alpha=0.5))
        return result

    address_list=convert_addresses(my_data['address'])
    legend_index_list = get_distinct_sorted_addresses(address_list)
    labels=get_legend_labels(legend_index_list)
    handles=get_legend_handles(legend_index_list)
    colors=address_to_color(address_list)
    fig, ax = plt.subplots()
    ax.scatter(my_data['time'], my_data['median_rssi'], s=my_data['count'] * 30, c=colors, label=colors,alpha=0.5)
    max_time = np.amax(my_data['time'][:])
    min_time = np.amin(my_data['time'][:])

    ax.set_xlim(min_time + timedelta(0,-10), max_time + timedelta(0,10))
    ax.set_ylim(-75, -100)


    # format the ticks
    locator = dates.AutoDateLocator()
    locator.intervald[dates.SECONDLY] = [20]

    #ax.set_title("Actor 2")
    ax.set_xlabel("time (min:sec)")
    ax.set_ylabel("RSSI (dBm)")

    formatter = dates.DateFormatter('%M:%S')

    ax.xaxis.set_major_locator(locator)
    ax.xaxis.set_major_formatter(formatter)
    ax.autoscale_view()
    ax.grid(True)
    plt.legend(handles,labels, loc="lower right")
    #plt.show()
    plt.savefig(outfilename, format="png")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Graph ble-mon firmware log.')
    parser.add_argument('infile', type=str,
                       help='input csv file to parse')
    parser.add_argument('outfile', type=str,
                       help='input csv file to parse')
    args = parser.parse_args()
    draw_pd(args.infile, args.outfile)
    pass
