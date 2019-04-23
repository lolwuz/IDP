from kivy.app import App
from kivy.lang import Builder
from kivy.uix.screenmanager import ScreenManager, Screen
from bluetooth import *
from Bluetoothctl import *
import json

mac = "B8:27:EB:9B:5F:C0"

# number_of_buttons = 9
#
# menu_buttons = ["Entering
# the arena", "Fast and the furious", "So you can think you can dance", "Line dance",
#                 "Obstacle course", "Cannon", "Transport and rebuild", "Besturing",
#                 "Emoticons"]
#
# function_names = ["module1", "module2", "module3", "module4", "module5", "module6", "module7", "besturing", "emoticons"]
# screen_names = ["Module1Screen", "Module12Screen", "Module3Screen", "Module4Screen", "Module5Screen", "Module6Screen",
#                 "Module7Screen", "Besturing", "Emoticons"]


class MainScreen(Screen):
    pass


class MenuScreen(Screen):
    pass
    # @mainthread
    # def on_enter(self):
    #     for i in xrange(number_of_buttons):
    #         button = Button(text=menu_buttons[i])
    #         self.ids.grid.add_widget(button)


class Module1Screen(Screen):
    pass


class Module2Screen(Screen):
    pass


class Module3Screen(Screen):
    pass


class Module4Screen(Screen):
    pass


class Module5Screen(Screen):
    pass


class Module6Screen(Screen):
    pass


class Module7Screen(Screen):
    pass


class Besturing(Screen):
    pass


class Emoticons(Screen):
    pass


class AnotherScreen(Screen):
    pass


class ScreenManagement(ScreenManager):
    pass


presentation = Builder.load_file("main.kv")



class MainApp(App):

    def testCalculate(self, n1, n2):
        print(n1 + n2)

    def build(self):
        return presentation

    def sendMessage(self, msg):
        print(str(mac) + " " + str(msg))
        try:
            bctl = Bluetoothctl()
            print(str(bctl))

            with open('test.json') as f:
                data = json.load(f)

            print(data)
            msg=data

            port = 3
            sock = BluetoothSocket(RFCOMM)
            sock.connect((mac, port))
            sock.send(json.dumps(msg))
            print(sock.recv(1024))
            sock.close()
        except BluetoothError as e:
            print("Something went wrong connecting to bluetooth, BluetoothError: " + e.message)

    #def establishBluetoothConnection(self):


MainApp().run()
