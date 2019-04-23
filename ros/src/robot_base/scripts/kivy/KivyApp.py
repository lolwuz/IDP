from kivy.uix.dropdown import DropDown
from kivy.uix.button import Button
from kivy.base import runTouchApp
from kivy.uix.gridlayout import GridLayout


class KivyApp():

    def MenuScreen(self):

        layout = GridLayout(cols=2)
        layout.add_widget(Button(text='Hello 1'))
        layout.add_widget(Button(text='World 1'))
        layout.add_widget(Button(text='Hello 2'))
        layout.add_widget(Button(text='World 2'))

    def LandingScreen(self):
        layout = GridLayout(cols=2)
        menuscreenbutton = Button(text='menuscreen')
        menuscreenbutton.bind()
        layout.add_widget(menuscreenbutton)
        layout.add_widget(Button(text='World 1'))
        layout.add_widget(Button(text='Hello 2'))
        layout.add_widget(Button(text='World 2'))

    def callback(instance, value):
        print('My button <%s> state is <%s>' % (instance, value))

    btn1 = Button(text='Hello world 1')
    btn1.bind(state=callback)




# create a dropdown with 10 buttons

dropdown = DropDown()
for index in range(10):
    # When adding widgets, we need to specify the height manually
    # (disabling the size_hint_y) so the dropdown can calculate
    # the area it needs.

    btn = Button(text='Value %d' % index, size_hint_y=None, height=44)

    # for each button, attach a callback that will call the select() method
    # on the dropdown. We'll pass the text of the button as the data of the
    # selection.
    btn.bind(on_release=lambda btn: dropdown.select(btn.text))

    # then add the button inside the dropdown
    dropdown.add_widget(btn)

# create a big main button
mainbutton = Button(text='Hello', size_hint=(None, None))

# show the dropdown menu when the main button is released
# note: all the bind() calls pass the instance of the caller (here, the
# mainbutton instance) as the first argument of the callback (here,
# dropdown.open.).
mainbutton.bind(on_release=dropdown.open)

# one last thing, listen for the selection in the dropdown list and
# assign the data to the button text.
dropdown.bind(on_select=lambda instance, x: setattr(mainbutton, 'text', x))

runTouchApp(mainbutton)