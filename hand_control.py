from enum import Enum
import numpy
import serial

START_FLAG = 0xEE
ESCAPE_FLAG = 0xEA
JOINTS_NUMBER = 22


def frame_type_control():
    return 0x00

def frame_type_sensor_read():
    return 0xA0

class Frame:

    def __init__(self, type, payload=None):
        self._type = type
        self._payload = payload
        self._zeros = [0, 5, 9, 13, 17, 18, 19, 20, 21]  # payload indexes to elements sent always as zero

    def get_payload(self):
        return self._payload

    def get_type(self):
        return self._type


class Port:

    def __init__(self, deviceName):
        self._port = serial.Serial(deviceName)

    def __del__(self):
        self._port.close()

    def send(self, frame):
        buffer = [START_FLAG, frame.get_type()]
        buffer += self._convertPayload(frame.get_payload())
        buffer.append(frame.get_type())
        print(buffer)
        self._port.write(buffer)

    def _convertPayload(self, payload):
        convertedPayload = []
        for angle in payload:
            angleEncoded = numpy.uint16(angle * 4095.0 / 360.0)

            if (angleEncoded & 0XFF) == START_FLAG or (angleEncoded & 0xFF) == ESCAPE_FLAG:
                convertedPayload.append(ESCAPE_FLAG)
            convertedPayload.append(angleEncoded & 0XFF)

            if ((angleEncoded >> 8) & 0XFF) == START_FLAG or ((angleEncoded >> 8) & 0xFF) == ESCAPE_FLAG:
                convertedPayload.append(ESCAPE_FLAG)
            convertedPayload.append((angleEncoded >> 8) & 0xFF)
        return convertedPayload


class RoboHand:
    class Finger:
        """
        Finger class. Joints are numbered since centre to tip of the hand.
        """

        def __init__(self, joints_nr=3):
            self._joints_nr = joints_nr
            self._joints = [0 for _ in range(joints_nr)]

        def set_configuration(self, configuration):
            assert len(configuration) == self._joints_nr, "Configuration length is not correct, finger has " + str(
                self._joints_nr) + " joints."
            for angle in configuration:
                assert type(angle) == int, "Type of all angles must be int"
            self._joints = configuration

        def get_configuration(self):
            return self._joints

    def __init__(self, port, fingers=None):
        """
        Creates object of robotic hand used to operate on physical hand object by using abstraction.

        :param port: Port used to communicate with hand
        :param fingers: dict<finger_name, Finger>
        """
        self._port = port
        self._fingers = fingers

        if self._fingers is None:
            self._configure_hand()

        self._finger_names = list(self._fingers.keys())


    def _configure_hand(self):
        """
        Creates default hand configuration: 1 thumb with 4 joints and 3 fingers with 3 joints.

        :return: None
        """
        self._fingers = {
            'Thumb': RoboHand.Finger(joints_nr=4),
            'Finger1': RoboHand.Finger(),
            'Finger2': RoboHand.Finger(),
            'Finger3': RoboHand.Finger(),
        }

    def _send_configuration(self):
        payload = [0 for _ in range(JOINTS_NUMBER)]
        thumb_joints_nrs = [3, 4, 1, 2]  # inside nrs of joints numbered from the centre to the tip of the hand
        for nr in thumb_joints_nrs:
            payload[nr] = self._fingers['Thumb'].get_configuration()[thumb_joints_nrs.index(nr)]

        finger1_joints_nrs = [8, 7, 6]
        for nr in finger1_joints_nrs:
            payload[nr] = self._fingers['Finger1'].get_configuration()[finger1_joints_nrs.index(nr)]

        finger2_joints_nrs = [12, 11, 10]
        for nr in finger2_joints_nrs:
            payload[nr] = self._fingers['Finger2'].get_configuration()[finger2_joints_nrs.index(nr)]

        finger3_joints_nrs = [16, 15, 14]
        for nr in finger3_joints_nrs:
            payload[nr] = self._fingers['Finger3'].get_configuration()[finger3_joints_nrs.index(nr)]

        frame = Frame(frame_type_control(), payload=payload)
        self._port.send(frame)

    def get_finger_names(self):
        """
        Returns all configured fingers names.

        :return: [str] - list of string representing names of fingers
        """
        return self._finger_names

    def get_finger_configuration(self, name):
        return self._fingers[name].get_configuration()

    def read_state(self):
        """
        TODO: reading from USB state - frame SENSOR_READ

        :return: state of hand - parsed sensors info
        """
        raise Exception("Not implemented")

    def set_position(self, fingersConfiguration):
        """
        Sends new configuration of fingers.

        :param fingersConfiguration: [int] - list of ints representing angles in joints from the centre to the tip of the hand
        :return: None
        """
        for fingerName, jointsAngles in fingersConfiguration.items():
            self._fingers[fingerName].set_configuration(jointsAngles)
        self._send_configuration()

gestures = {
	'open_hand':
	{
		'Thumb': [0, 0, 0, 0],
		'Finger1': [0, 0, 0],
		'Finger2': [0, 0, 0],
		'Finger3': [0, 0, 0]
    },
	'cylindrical':
	{
		'Thumb': [70, 0, -40, 10],
		'Finger1': [0, 20, 30],
		'Finger2': [0, 32, 30],
		'Finger3': [0, 37, 30]
    },
    'pinch':
	{
		'Thumb': [90, 0, -35, 10],
		'Finger1': [0, 60, 30],
		'Finger2': [0, 32, 30],
		'Finger3': [0, 37, 30]
    },
    'flat_grip':
    {
		'Thumb': [0, 50, 20, 0],
		'Finger1': [0, 60, 20],
		'Finger2': [0, 80, 90],
		'Finger3': [0, 80, 90]
	},
}


if __name__ == '__main__':
    hand = RoboHand(Port('/dev/ttyACM0'))
    newConf = {
        'Thumb': [0, 50, 10, 0],
        'Finger1': [0, 90, 80],
        'Finger2': [0, 90, 80],
        'Finger3': [0, 90, 80]
    }
    hand.set_position(newConf)
