#!python
import matplotlib.pyplot as plt
from matplotlib import dates
from datetime import datetime, timedelta
import argparse
import numpy as np
def draw_pd(infile, outfilename):
    timeConverter = lambda x: datetime.strptime(x.decode('ascii'), '%Y-%m-%d %H:%M:%S.%f')
    contactData = np.genfromtxt(infile, delimiter='\t', dtype=None, names=['time', 'address', 'avg_rssi', 'median_rssi', 'count'],
                         converters={'time': timeConverter})

    def address_to_color(addresses):
        colors = ['#FF9D00', '#8000FF', '#32A08C', '#F03237', '#008CC8', 'b', 'g', 'r', 'c', 'm', 'y', 'b', 'w']
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
            result.append("Actor %d" % (a+1))
        return result

    def get_legend_handles(addresses):
        result = []
        colors = address_to_color(addresses)
        for c in colors:
            result.append(plt.Circle((0,0), color=c, alpha=0.5))
        return result

    address_list=convert_addresses(contactData['address'])
    legend_index_list = get_distinct_sorted_addresses(address_list)
    labels=get_legend_labels(legend_index_list)
    handles=get_legend_handles(legend_index_list)
    colors=address_to_color(address_list)
    fig, ax = plt.subplots()
    ax.scatter(contactData['time'], contactData['median_rssi'], s=contactData['count'] * 15, c=colors, label=colors,alpha=0.5)
    max_time = np.amax(contactData['time'][:])
    min_time = np.amin(contactData['time'][:])

    x_min = min_time - timedelta(0,60 + min_time.second, min_time.microsecond - 500000)
    x_max = max_time  + timedelta(0, 60 - max_time.second, 1000000 - max_time.microsecond )
    ax.set_xlim(x_min, x_max)
    ax.set_ylim(-70, -100)


    #ax.set_title("Actor 2")
    ax.set_xlabel("time (min:sec)")
    ax.set_ylabel("RSSI (dBm)")

    majLocator = dates.SecondLocator(interval=60)
    minLocator = dates.SecondLocator(interval=30)
    formatter = dates.DateFormatter('%M:%S')


    ax.xaxis.set_major_locator(majLocator)
    ax.xaxis.set_minor_locator(minLocator)
    ax.xaxis.set_major_formatter(formatter)
    ax.autoscale_view()
    ax.grid(True, which='both')
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
