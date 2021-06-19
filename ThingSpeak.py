import urllib.request as urllib2
import json

UPDATE_URL = 'https://api.thingspeak.com/update?api_key='
READ_URL = 'https://api.thingspeak.com/channels/[channel]/feeds.json?api_key='
my_key = open('key', 'r').read()


class ThingSpeakCommunication:
    def __init__(self, api_key=my_key, channel=1389545):
        self._api_key = api_key
        self._channel = channel
        self._update_url = UPDATE_URL + self._api_key
        self._read_url = READ_URL.replace('[channel]', str(self._channel)) + self._api_key

    def update_key(self, api_key):
        self._api_key = api_key
        self._update_url = UPDATE_URL + self._api_key

    def send(self, **kwargs):
        suffix = ''
        for key in kwargs:
            suffix += f'&{key}={kwargs[key]}'
        url = self._update_url + suffix
        try:
            print("ThingSpeak - Attempt to transmit data:", url)
            with urllib2.urlopen(url) as conn:
                print("ThingSpeak - Connection return code = " + str(conn.read()))
            print("ThingSpeak - Transmit looks ok")
        except TypeError:
            print("ThingSpeak - Error! ")

    def read(self, **kwargs):
        suffix = ''
        for key in kwargs:
            suffix += f'&{key}={kwargs[key]}'
        url = self._read_url + suffix
        try:
            print("ThingSpeak - reading data: ", url)
            with urllib2.urlopen(url) as conn:
                message = conn.read()
                return message.decode('utf-8')
        except TypeError:
            print("ThingSpeak - Error!")
            return False


if __name__ == "__main__":
    teamspeak = ThingSpeakCommunication()
    data = teamspeak.read()
    data = json.dumps([json.loads(data)], indent=4)
    print(data)
