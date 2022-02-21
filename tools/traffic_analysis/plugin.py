# Copyright 2022 NXP. All rights reserved.
# Disclaimer
# 1. The NXP Software/Source Code is provided to Licensee "AS IS" without any
# warranties of any kind. NXP makes no warranties to Licensee and shall not
# indemnify Licensee or hold it harmless or any reason related to the NXP
# Software/Source Code or otherwise be liable to the NXP customer. The NXP
# customer acknowledges and agrees that the NXP Software/Source Code is
# provided AS-IS and accepts all risks of utilizing the NXP Software under the
# conditions set forth according to this disclaimer.
# *
# 2. NXP EXPRESSLY DISCLAIMS ALL WARRANTIES, EXPRESS OR IMPLIED, INCLUDING, BUT
# NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
# PARTICULAR PURPOSE, AND NON-INFRINGEMENT OF INTELLECTUAL PROPERTY RIGHTS. NXP
# SHALL HAVE NO LIABILITY TO THE NXP CUSTOMER, OR ITS SUBSIDIARIES, AFFILIATES,
# OR ANY OTHER THIRD PARTY FOR ANY DAMAGES, INCLUDING WITHOUT LIMITATION,
# DAMAGES RESULTING OR ALLEGED TO HAVE RESULTED FROM ANY DEFECT, ERROR OR
# OMISSION IN THE NXP SOFTWARE/SOURCE CODE, THIRD PARTY APPLICATION SOFTWARE
# AND/OR DOCUMENTATION, OR AS A RESULT OF ANY INFRINGEMENT OF ANY INTELLECTUAL
# PROPERTY RIGHT OF ANY THIRD PARTY. IN NO EVENT SHALL NXP
# BE LIABLE FOR ANY INCIDENTAL, INDIRECT, SPECIAL, EXEMPLARY, PUNITIVE, OR
# CONSEQUENTIAL DAMAGES (INCLUDING LOST PROFITS) SUFFERED BY NXP CUSTOMER OR
# ITS SUBSIDIARIES, AFFILIATES, OR ANY OTHER THIRD PARTY ARISING OUT OF OR
# RELATED TO THE NXP SOFTWARE/SOURCE CODE EVEN IF NXP HAS BEEN ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGES.

"plugin loader and plugin base class implementation"

import importlib
import pkgutil
import inspect


class AnalyzerPlugin(object):
    """
    Plugin interface class for analyzing network packets
    """

    def __init__(self):
        self._name = 'unknown'
        self._filters = []
        self.hasEnabledFilters = False

    @property
    def name(self):
        """
        Name of the plugin, normally specifying which protocol or protocol-specific
        topic it covers. Topics should be named like <protocol>.<topic> to make
        wildcards possible.
        """
        return self._name

    @property
    def availablePacketFilters(self):
        """
        Optional array property that specifies which optional packet filter the plugin
        provides. Useful if you want to filter in a speicific plugin, but don't want to
        influence e.g. the Ethernet-based statistics with a restrictive global filter.

        Example:
        return [("udp_port", "filter for specific UDP ports in RTPS datagrams")]
        """
        return self._filters

    def processFilter(self, p, filter_name, filter_value):
        """
        Optional function to check if user defined plugin filter applies. Returns True
        if a packet should be further analyzed and False if the packet is filtered.
        """
        return True

    def processPacket(self, p):
        """
        This function will be called for every packet in the recorded network traffic,
        as long as the plugin filter applies or is set to None. The plugin can analyze
        the packet further and calculate statistics.
        """
        raise NotImplementedError

    def reportStatistics(self):
        """
        This function will be called after all packets in the recorded network traffic
        are processed. Plugin should return a dict with its calculated statistics.
        """
        raise NotImplementedError

    def reportCsvOutput(self):
        """
        This function will be called if the user requested a detailed CSV output for the
        plugin ([sequence_numbers, capture_timestamps, production2sniff_intervals, production_intervals]).
        Plugin should return an array of arrays. E.g. the following array:
            [[0,1],[2,3],[4,5]]
        will be outputed like:
            0;2;4;
            1;4;5;
        """
        return []


class PluginLoader(object):
    def __init__(self, plugins_path='plugins', plugins=[]):
        self.plugins_module_path = plugins_path
        self._plugins = plugins

    @property
    def plugins(self):
        return self._plugins

    def load_plugins(self):
        modules = importlib.__import__(self.plugins_module_path)
        for _, plugin_name, is_pkg in pkgutil.iter_modules(modules.__path__, modules.__name__ + '.'):
            if is_pkg:
                continue
            self.__import_plugin(plugin_name)

    def __import_plugin(self, import_path):
        module = importlib.import_module(import_path)
        class_list = inspect.getmembers(module, inspect.isclass)

        for (_, plugin_class) in class_list:
            if issubclass(plugin_class, AnalyzerPlugin) and (plugin_class is not AnalyzerPlugin):
                instance = plugin_class()
                print('found plugin: ' + instance.name)
                self._plugins.append(instance)
