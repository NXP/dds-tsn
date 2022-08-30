#!/usr/bin/env python3

# Copyright 2022 NXP
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Sniff network traffic or read from a PCAP capture file and compute various statistics
"""

import argparse
import logging as log
import pyshark
from tabulate import tabulate
from plugin import PluginLoader


def main():
    'module entry: load plugins, process command line and run packet analysis'
    args = processCommandLine() # validate command before loading plugins
    plugin_loader = PluginLoader()
    plugin_loader.load_plugins()

    if args.show_pluginFilters or not validatePluginFilters(args, plugin_loader):
        print('Available plugin filters are:')
        for plugin in plugin_loader.plugins:
            for filter in plugin.availablePacketFilters:
                print('  {}.{: <25}{}'.format(plugin.name, filter[0], filter[1]))
        exit(0)

    if args.captureFilePath is not None:
        packets = pyshark.FileCapture(args.captureFilePath, display_filter=args.wiresharkFilter, use_json=True)
    else:
        assert(args.interface is not None)
        log.debug('sniffing network interface {} for {} seconds'.format(args.interface, args.duration))
        capture = pyshark.LiveCapture(args.interface, display_filter=args.wiresharkFilter, use_json=True,
                                      custom_parameters={'--time-stamp-type': 'adapter_unsynced'})
        capture.sniff(timeout=float(args.duration))
        packets = capture._packets # TODO: any better access method?
    process(packets, plugin_loader, args.pluginFilters)
    print_stats(plugin_loader, args.wiresharkFilter, args.pluginFilters)
    write_csv(plugin_loader, args.csvOutput)


def processCommandLine():
    'process the command line arguments'
    description = 'Sniff network traffic or read from a file and compute various statistics'
    parser = argparse.ArgumentParser(description=description)
    parser.add_argument('-i', '--interface', metavar='NETWORK_INTERFACE',
                        help='network interface to open for packet sniffing')
    parser.add_argument('-d', '--duration', metavar='DURATION', default=10,
                        help='duration in seconds of live packet sniffing on the specified network interface')
    parser.add_argument('-p', '--captureFilePath', metavar='FILE_PATH',
                        help='PCAP capture file to be processed')
    parser.add_argument('-f', '--wiresharkFilter', metavar='FILTER', default=None,
                        help='the filter used in Wireshark to focus processing on specific packets')
    parser.add_argument('-x', '--pluginFilters', metavar=('pluginFilter', 'value'),
                        action='append', nargs=2, default=[],
                        help='display filters on plugin level')
    parser.add_argument('-c', '--csvOutput', metavar=('pluginName', 'csvFileName'),
                        action='append', nargs=2, default=[],
                        help='display filters on plugin level')
    parser.add_argument('-s', '--show-pluginFilters', default=False, action='store_true',
                        help='show available display filters on plugin level')
    parser.add_argument('-v', '--verbose', default=False, action='store_true',
                        help='verbose mode with extra logging')
    args = parser.parse_args()
    if args.verbose:
        logLevel = log.DEBUG
    else:
        logLevel = log.WARNING
    log.basicConfig(level=logLevel)
    log.debug(args)
    if args.captureFilePath is None and args.interface is None and not args.show_pluginFilters:
        print('Error! Specify either -i or -p argument.')
        parser.print_help()
        exit(-1)
    return args


def validatePluginFilters(args, plugin_loader):
    for user_filter in args.pluginFilters:
        is_found = False
        for plugin in plugin_loader.plugins:
            for filter in plugin.availablePacketFilters:
                if (plugin.name + '.' + filter[0]) == user_filter[0]:
                    is_found = True
                    plugin.hasEnabledFilters = True
        if not is_found:
            log.error('Error! Was not able to find plugin filter {}'.format(user_filter))
            return False
    return True


def process(packets, plugin_loader, filters):
    'traverse packets in the packets object and run plugin process()'
    try:
        for idx, p in enumerate(packets):
            for plugin in plugin_loader.plugins:
                doPacketProcessing = True
                # check if the implemented plugin filters are used
                if plugin.hasEnabledFilters:
                    for filter in filters:
                        if not plugin.processFilter(p, filter[0], filter[1]):
                            doPacketProcessing = False
                if doPacketProcessing:
                    plugin.processPacket(p)
            log.debug('frame content:\n' + str(p))
            if (idx + 1) % 10000 == 0:
                print('processed {} Ethernet frames'.format(idx + 1))
    except KeyboardInterrupt:
        print('\nStopped processing due to a keyboard interrupt, statistics will be incomplete')


def print_stats(plugin_loader, filter, pluginFilters):
    'print statistics based on processed packets for all plugins'
    print('Statistics with a global pyshark filter: {}'.format(filter))
    if len(pluginFilters) > 0:
        print('  Enabled plugins filters:')
        for i in pluginFilters:
            print('    {} == {}'.format(i[0], i[1]))
        print('  Note: plugins with activated filters are marked with a (*)')
    tabular_output = []
    for plugin in plugin_loader.plugins:
        stats = plugin.reportStatistics()
        if stats is None:
            log.warning('Skipping plugin {}, which reports no statistics'.format(plugin))
            continue

        leading_row = plugin.name
        if plugin.hasEnabledFilters:
            leading_row += " (*)"
        for key, value in stats.items():
            tabular_output.append((leading_row, key, value))
            leading_row = '' # just print the plugin name once

    print(tabulate(tabular_output, headers=['plugin', 'name', 'value'], tablefmt='presto'))

def write_csv(plugin_loader, enabledCsvOutput):
    if len(enabledCsvOutput) < 1:
        return
    print('Writing CSV output:')
    for i in enabledCsvOutput:
        csv_plugin_name = i[0]
        csv_file_path = i[1]
        print(' {} to {}'.format(csv_plugin_name, csv_file_path))
        for plugin in plugin_loader.plugins:
            if plugin.name != csv_plugin_name:
                continue
            csv_raw_output = plugin.reportCsvOutput()
            column_count = len(csv_raw_output)
            # TODO: check length of each column
            if column_count < 1:
                log.warning('Skipping CSV ouput for plugin {}, reported output empty!'.format(csv_plugin_name))
                continue
            row_count = len(csv_raw_output[0])
            with open(csv_file_path, 'w') as output_file:
                for row_index in range(0, row_count):
                    for column_index in range(0, column_count):
                        output_file.write(str(csv_raw_output[column_index][row_index]) + ';')
                    output_file.write('\n')
    return

if __name__ == '__main__':
    main()
