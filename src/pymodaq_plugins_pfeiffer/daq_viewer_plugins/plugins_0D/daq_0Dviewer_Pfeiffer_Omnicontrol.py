import numpy as np

from pymodaq_utils.utils import ThreadCommand
from pymodaq_data.data import DataToExport
from pymodaq_gui.parameter import Parameter

from pymodaq.control_modules.viewer_utility_classes import DAQ_Viewer_base, comon_parameters, main
from pymodaq.utils.data import DataFromPlugins

from pymodaq_plugins_pfeiffer.hardware.Omnicontrol import Omnicontrol
from serial.tools import list_ports

def get_com_ports() -> list[str]:
    """Détecte les ports COM connectés et les renvoie sous forme de liste de strings."""
    return [port.device for port in list_ports.comports()]

class DAQ_0DViewer_Pfeiffer_Omnicontrol(DAQ_Viewer_base):
    """ Instrument plugin class for a OD viewer.
    
    This object inherits all functionalities to communicate with PyMoDAQ’s DAQ_Viewer module through inheritance via
    DAQ_Viewer_base. It makes a bridge between the DAQ_Viewer module and the Python wrapper of a particular instrument.

    TODO Complete the docstring of your plugin with:
        * The set of instruments that should be compatible with this instrument plugin.
        * With which instrument it has actually been tested.
        * The version of PyMoDAQ during the test.
        * The version of the operating system.
        * Installation instructions: what manufacturer’s drivers should be installed to make it run?

    """
    params = comon_parameters+[
        {'title': 'COM port :', 'name': 'com_port', 'type': 'list', 'limits': get_com_ports()},
        {'title': 'Channel 1', 'name': 'ch_1', 'type': 'bool', 'value':True},
        {'title': 'Channel 2', 'name': 'ch_2', 'type': 'bool', 'value':True},
        ]

    def _active_channels(self):
        """Names of the currently checked channel booleans, in tree order."""
        return [ch.name()
            for ch in self.settings.children()
            if ch.name().startswith('ch_') and ch.value()]


    def ini_attributes(self):
        self.controller: Omnicontrol = None
        self.port=None
        pass

    def ini_detector(self, controller=None):
        """Detector communication initialization"""
        self.settings.child('com_port').setLimits(get_com_ports())
        if self.is_master:
            self.port=self.settings.child('com_port').value()
            self.port="COM9"
            self.controller = Omnicontrol(self.port)
            initialized = True
        else:
            self.controller = controller
            initialized = True

        self.pressure_data=[np.array([0]) for ch in self._active_channels()]
        label=[]
        for ch in self._active_channels():
            if ch == 'ch_1':
                label.append('Channel 1')
            elif ch == 'ch_2':
                label.append('Channel 2')
        self.dte_signal_temp.emit(DataToExport(name='OmniControl',
                                               data=[DataFromPlugins(name='Pressure',
                                                                    data=self.pressure_data,
                                                                    dim='Data0D',
                                                                    labels=label)]))
        # labels = ['Channel 1', 'Channel 2'])]))

        info = self.controller.read_name()  # lève PfeifferError si pas de réponse
        return info, initialized

    def close(self):
        """Terminate the communication protocol"""
        if self.is_master and self.controller is not None:
            self.controller.close()
            self.controller = None

    def grab_data(self, Naverage=1, **kwargs):
        """Start a grab from the detector

        Parameters
        ----------
        Naverage: int
            Number of hardware averaging (if hardware averaging is possible, self.hardware_averaging should be set to
            True in class preamble and you should code this implementation)
        kwargs: dict
            others optionals arguments
        """
        pressure_data=[]
        try:
            for ch in self._active_channels():
                if ch == 'ch_1':
                    pressure_data.append(np.array([self.controller.read_pressure(122)]))
                elif ch == 'ch_2':
                    pressure_data.append(np.array([self.controller.read_pressure(132)]))
        except Exception as e:
            self.emit_status(ThreadCommand('Update_Status', [str(e), 'log']))
            return

        label=[]
        for ch in self._active_channels():
            if ch == 'ch_1':
                label.append('Channel 1')
            elif ch == 'ch_2':
                label.append('Channel 2')

        self.dte_signal.emit(DataToExport(
            name='OmniControl',
            data=[DataFromPlugins(name='Pressure',
                                  data=pressure_data,
                                  dim='Data0D',
                                  labels=label)]))

    def stop(self):
        """Stop the current grab hardware wise if necessary"""
        return ''

    def commit_settings(self, param: Parameter):
        """Apply the consequences of a change of value in the detector settings

        Parameters
        ----------
        param: Parameter
            A given parameter (within detector_settings) whose value has been changed by the user
        """
        ## TODO for your custom plugin
        if param.name() == "com_port":
           self.port=self.settings.child('com_port').value()


if __name__ == '__main__':
    main(__file__)
