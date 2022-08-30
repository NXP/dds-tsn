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
