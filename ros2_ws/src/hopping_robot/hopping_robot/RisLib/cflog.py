import cflib.crtp
from cflib.crazyflie import Crazyflie
from cflib.crazyflie.log import LogConfig
import logging
import time


class LoggingCore:
    """
    Simple logging example class that logs the Stabilizer from a supplied
    link uri and disconnects after 5s.
    """

    def __init__(self, link_uri, sample_time, logging_dict):
        logging.basicConfig(level=logging.ERROR)
        self.is_connected = False

        cflib.crtp.init_drivers()

        """ Initialize and run the example with the specified link_uri """
        self._logging_dict = logging_dict

        self._logging_output = {}
        for i in logging_dict:
            self._logging_output[i] = logging_dict[i]  # element by elemnet copying

        for i in self._logging_output:
            self._logging_output[i] = 0.0

        self.cf = Crazyflie(rw_cache='./cache')

        self.sample_time = sample_time

        # Connect some callbacks from the Crazyflie API
        self.cf.connected.add_callback(self._connected)
        self.cf.disconnected.add_callback(self._disconnected)
        self.cf.connection_failed.add_callback(self._connection_failed)
        self.cf.connection_lost.add_callback(self._connection_lost)

        print('Connecting to %s' % link_uri)

        # Try to connect to the Crazyflie
        self.cf.open_link(link_uri)

        # Variable used to keep main loop occupied until disconnect
        # self.is_connected = True

        time.sleep(0.2)

        temp_dict = {}
        for i in logging_dict:
            temp_dict[i] = logging_dict[i]  # element by elemnet copying
        self.temp_keys = list(temp_dict)
        # print(self.temp_keys)
        for key in self.temp_keys:
            # print(key)
            self.temp_keys[self.temp_keys.index(key)] = key.replace(".", "_")
        time.sleep(0.1)


    def get_logged_data(self):
        return self._logging_output

    def pre_set_parameter(self, param_dict):
        while not self.is_connected:
            print('waiting for connection...')
            time.sleep(1)

        print('setting Crazyflie parameters...')
        time.sleep(3)

        for key in param_dict:
            self.cf.param.set_value(key, param_dict[key])
            time.sleep(0.1)
        print('parameters set')


    def stop(self):
        self._lg_stab.stop()
        self.cf.close_link()

    def _connected(self, link_uri):
        """ This callback is called form the Crazyflie API when a Crazyflie
        has been connected and the TOCs have been downloaded."""
        print('Connected to %s' % link_uri)

        # The definition of the logconfig can be made before connecting
        self._lg_stab = LogConfig(name='Stabilizer', period_in_ms=self.sample_time)

        for logging_variable in self._logging_dict:
            self._lg_stab.add_variable(logging_variable, self._logging_dict[logging_variable])

        # Adding the configuration cannot be done until a Crazyflie is
        # connected, since we need to check that the variables we
        # would like to log are in the TOC.
        try:
            self.cf.log.add_config(self._lg_stab)
            # This callback will receive the data
            self._lg_stab.data_received_cb.add_callback(self._stab_log_data)
            # This callback will be called on errors
            self._lg_stab.error_cb.add_callback(self._stab_log_error)
            # Start the logging
            self._lg_stab.start()
        except KeyError as e:
            print('Could not start log configuration,'
                  '{} not found in TOC'.format(str(e)))
        except AttributeError:
            print('Could not add Stabilizer log config, bad configuration.')

        self.is_connected = True

    def _stab_log_error(self, logconf, msg):
        """Callback from the log API when an error occurs"""
        print('Error when logging %s: %s' % (logconf.name, msg))

    def _stab_log_data(self, timestamp, data, logconf):
        """Callback from a the log API when data arrives"""
        # print(f'[{timestamp}][{logconf.name}]: ', end='')
        # for name, value in data.items():
        #     print(f'{name}: {value:3.3f} ', end='')
        # print()
        # print('aaa' , data)
        for name, value in data.items():
            # print(f'{name}: {value:3.3f} ', end='')
            self._logging_output[name] = value
        # print(self._logging_output)
        # print()

    def _connection_failed(self, link_uri, msg):
        """Callback when connection initial connection fails (i.e no Crazyflie
        at the specified address)"""
        print('Connection to %s failed: %s' % (link_uri, msg))
        self.is_connected = False

    def _connection_lost(self, link_uri, msg):
        """Callback when disconnected after a connection has been made (i.e
        Crazyflie moves out of range)"""
        print('Connection to %s lost: %s' % (link_uri, msg))

    def _disconnected(self, link_uri):
        """Callback when the Crazyflie is disconnected (called in all cases)"""
        print('Disconnected from %s' % link_uri)
        self.is_connected = False


    # def remove_dot(target_dict):
    #     temp_dict = {}
    #     for i in target_dict:
    #         temp_dict[i] = target_dict[i]  # element by elemnet copying
    #     temp_keys = list(temp_dict)
    #     for key in temp_keys:
    #         temp_keys[temp_keys.index(key)] = key.replace(".", "_")
    #     return list(temp_keys)


class LoggingCore2:
    """
    Simple logging example class that logs the Stabilizer from a supplied
    link uri and disconnects after 5s.
    """

    def __init__(self, link_uri, sample_time, logging_list_array,):
        logging.basicConfig(level=logging.ERROR)

        cflib.crtp.init_drivers()

        """ Initialize and run the example with the specified link_uri """
        self.logging_list_array = logging_list_array
        # self._logging_dict = logging_dict
        # self._logging_dict2 = logging_dict2
        self._logging_output = [{}]
        for j in range(len(logging_list_array)):
            if j == 0:
                pass
            else:
                self._logging_output.append({})

            for i in logging_list_array[j]:
                self._logging_output[j][i] = logging_list_array[j][i]  # element by elemnet copying
                self._logging_output[j][i] = 0.0


        # self._logging_output = {}
        # for i in logging_dict:
        #     self._logging_output[i] = logging_dict[i]  # element by elemnet copying
        #
        # self._logging_output2 = {}
        # for i in logging_dict2:
        #     self._logging_output2[i] = logging_dict2[i]  # element by elemnet copying

        # for i in self._logging_output:
        #     self._logging_output[i] = 0.0
        # for i in self._logging_output2:
        #     self._logging_output2[i] = 0.0

        self.cf = Crazyflie(rw_cache='./cache')

        self.sample_time = sample_time

        # Connect some callbacks from the Crazyflie API
        self.cf.connected.add_callback(self._connected)
        self.cf.disconnected.add_callback(self._disconnected)
        self.cf.connection_failed.add_callback(self._connection_failed)
        self.cf.connection_lost.add_callback(self._connection_lost)

        print('Connecting to %s' % link_uri)

        # Try to connect to the Crazyflie
        self.cf.open_link(link_uri)

        # Variable used to keep main loop occupied until disconnect
        # self.is_connected = True

        time.sleep(0.2)

        temp_dict = [{}]
        for j in range(len(logging_list_array)):
            if j == 0:
                pass
            else:
                temp_dict.append({})

            for i in logging_list_array[j]:
                temp_dict[j][i] = logging_list_array[j][i]  # element by elemnet copying

        self.temp_keys = list(temp_dict[0])
        for j in range(len(logging_list_array)):
            if j == 0:
                pass
            else:
                temp = list(temp_dict[j])
                for k in range(len(temp)):
                    a = temp[k] + '_cfg_' + str(j)
                    temp[k] = a
                self.temp_keys = self.temp_keys + temp
        for key in self.temp_keys:
            self.temp_keys[self.temp_keys.index(key)] = key.replace(".", "_")
        time.sleep(0.1)


        # temp_dict = {}
        # for i in logging_dict:
        #     temp_dict[i] = logging_dict[i]  # element by elemnet copying
        # self.temp_keys = list(temp_dict)
        # for key in self.temp_keys:
        #     self.temp_keys[self.temp_keys.index(key)] = key.replace(".", "_")
        # time.sleep(0.1)

    def get_logged_data(self):
        # savemat_tuple = *tuple(self._logging_output[0].values()), *tuple(self._logging_output[1].values())
        return self._logging_output

    def pre_set_parameter(self, param_dict):
        print('setting Crazyflie parameters...')
        time.sleep(0.3)
        for key in param_dict:
            self.cf.param.set_value(key, param_dict[key])
            time.sleep(0.1)
        print('parameters set')

    def stop(self):
        for i in self._lg_stab:
            i.stop()
        self.cf.close_link()

    def _connected(self, link_uri):
        """ This callback is called form the Crazyflie API when a Crazyflie
        has been connected and the TOCs have been downloaded."""
        print('Connected to %s' % link_uri)

        # The definition of the logconfig can be made before connecting
        self._lg_stab = [LogConfig(name=str(0), period_in_ms=self.sample_time)]
        for j in range(len(self.logging_list_array)):
            if j == 0:
                pass
            else:
                self._lg_stab.append(LogConfig(name=str(j), period_in_ms=self.sample_time))

        # self._lg_stab = LogConfig(name='Stabilizer', period_in_ms=self.sample_time)
        # self._lg_stab2 = LogConfig(name='Stabilizer2', period_in_ms=self.sample_time)

        for j in range(len(self.logging_list_array)):
            for logging_variable in self.logging_list_array[j]:
                self._lg_stab[j].add_variable(logging_variable, self.logging_list_array[j][logging_variable])

        # for logging_variable in self._logging_dict:
        #     self._lg_stab.add_variable(logging_variable, self._logging_dict[logging_variable])
        # for logging_variable in self._logging_dict2:
        #     self._lg_stab2.add_variable(logging_variable, self._logging_dict2[logging_variable])

        # Adding the configuration cannot be done until a Crazyflie is
        # connected, since we need to check that the variables we
        # would like to log are in the TOC.
        for j in range(len(self.logging_list_array)):
            try:
                self.cf.log.add_config(self._lg_stab[j])
                # This callback will receive the data
                self._lg_stab[j].data_received_cb.add_callback(self._stab_log_data_general)
                # This callback will be called on errors
                self._lg_stab[j].error_cb.add_callback(self._stab_log_error)
                # Start the logging
                self._lg_stab[j].start()
            except KeyError as e:
                print('Could not start log configuration,'
                      '{} not found in TOC'.format(str(e)))
            except AttributeError:
                print('Could not add Stabilizer log config, bad configuration.')

        # try:
        #     self.cf.log.add_config(self._lg_stab)
        #     # This callback will receive the data
        #     self._lg_stab.data_received_cb.add_callback(self._stab_log_data)
        #     # This callback will be called on errors
        #     self._lg_stab.error_cb.add_callback(self._stab_log_error)
        #     # Start the logging
        #     self._lg_stab.start()
        # except KeyError as e:
        #     print('Could not start log configuration,'
        #           '{} not found in TOC'.format(str(e)))
        # except AttributeError:
        #     print('Could not add Stabilizer log config, bad configuration.')
        #
        # try:
        #     self.cf.log.add_config(self._lg_stab2)
        #     # This callback will receive the data
        #     self._lg_stab2.data_received_cb.add_callback(self._stab_log_data2)
        #     # This callback will be called on errors
        #     self._lg_stab2.error_cb.add_callback(self._stab_log_error)
        #     # Start the logging
        #     self._lg_stab2.start()
        # except KeyError as e:
        #     print('Could not start log configuration,'
        #           '{} not found in TOC'.format(str(e)))
        # except AttributeError:
        #     print('Could not add Stabilizer log config, bad configuration.')

        self.is_connected = True

    def _stab_log_error(self, logconf, msg):
        """Callback from the log API when an error occurs"""
        print('Error when logging %s: %s' % (logconf.name, msg))

    def _stab_log_data_general(self, timestamp, data, logconf):
        """Callback from a the log API when data arrives"""
        # print(f'[{timestamp}][{logconf.name}]: ', end='')
        # for name, value in data.items():
        #     print(f'{name}: {value:3.3f} ', end='')
        # print()
        # print('aaa' , data)
        j = int(logconf.name)
        for name, value in data.items():
            # print(f'{name}: {value:3.3f} ', end='')
            self._logging_output[j][name] = value
        # print(self._logging_output)
        # print()

    # def _stab_log_data(self, timestamp, data, logconf):
    #     """Callback from a the log API when data arrives"""
    #     # print(f'[{timestamp}][{logconf.name}]: ', end='')
    #     # for name, value in data.items():
    #     #     print(f'{name}: {value:3.3f} ', end='')
    #     # print()
    #     # print('aaa' , data)
    #
    #     for name, value in data.items():
    #         # print(f'{name}: {value:3.3f} ', end='')
    #         self._logging_output[name] = value
    #     # print(self._logging_output)
    #     # print()
    # def _stab_log_data2(self, timestamp, data, logconf):
    #     """Callback from a the log API when data arrives"""
    #     # print(f'[{timestamp}][{logconf.name}]: ', end='')
    #     # for name, value in data.items():
    #     #     print(f'{name}: {value:3.3f} ', end='')
    #     # print()
    #     # print('aaa' , data)
    #
    #     for name, value in data.items():
    #         # print(f'{name}: {value:3.3f} ', end='')
    #         self._logging_output2[name] = value
    #     # print(self._logging_output)
    #     # print()

    def _connection_failed(self, link_uri, msg):
        """Callback when connection initial connection fails (i.e no Crazyflie
        at the specified address)"""
        print('Connection to %s failed: %s' % (link_uri, msg))
        self.is_connected = False

    def _connection_lost(self, link_uri, msg):
        """Callback when disconnected after a connection has been made (i.e
        Crazyflie moves out of range)"""
        print('Connection to %s lost: %s' % (link_uri, msg))

    def _disconnected(self, link_uri):
        """Callback when the Crazyflie is disconnected (called in all cases)"""
        print('Disconnected from %s' % link_uri)
        self.is_connected = False


    # def remove_dot(target_dict):
    #     temp_dict = {}
    #     for i in target_dict:
    #         temp_dict[i] = target_dict[i]  # element by elemnet copying
    #     temp_keys = list(temp_dict)
    #     for key in temp_keys:
    #         temp_keys[temp_keys.index(key)] = key.replace(".", "_")
    #     return list(temp_keys)